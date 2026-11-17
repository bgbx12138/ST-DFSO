"""Four-component response model with exact-input caching."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import tempfile
import threading
import time

from .asset_loader import AssetLoader, REPO
from .linear_response import FixedLinearResponse
from .wave_heading import wave_heading_statistics
from .shared_cache import input_lock

SCHEMA = 'STDFSO_WAVE_HEADING_FOUR_COMPONENT_V1'
POOL_SPEC = 'config/wave_response_initial_pool.json'
# Asset modules temporarily replace shared numerical callbacks. Serialise all
# instances in a process, not only calls through an individual provider.
_LOCK = threading.RLock()


def encode(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def identity():
    names = ['config/model_files.json', 'config/linear_response.json',
             'config/unified_response.json', 'src/main/python/navigation_safety/unified_response.py',
             'src/main/python/navigation_safety/provider.py',
             'src/main/python/navigation_safety/response_dynamics.py',
             'src/main/python/navigation_safety/linear_response.py',
             'src/main/python/navigation_safety/wave_heading.py']
    hashes = {p: hashlib.sha256((REPO/p).read_bytes()).hexdigest() for p in names}
    return dict(schema=SCHEMA, files=hashes)


class SafetyFunction:
    def __init__(self, cache, *, diagnostics=None):
        self.cache_path = Path(cache).resolve()
        if any(self.cache_path.is_relative_to(REPO/n) for n in ('data', 'config', 'src', '.git')):
            raise ValueError('Runtime cache must be outside published inputs')
        self.diagnostics = diagnostics
        self._identity = identity()
        self.revision = hashlib.sha256(encode(self._identity).encode()).hexdigest()
        self._identity.update(physical_parent=self.revision, weighted_parent=self.revision)
        self.config = json.loads((REPO/'config/unified_response.json').read_text('utf8'))
        if self.config['schema'] != SCHEMA:
            raise ValueError('Unknown four-component configuration')
        self.stats = dict.fromkeys(('requests', 'memory_hits', 'persistent_hits', 'component_calls',
                                   'uncached_component_evaluations', 'physical_solve_attempts', 'failed'), 0)
        self._memo = {}
        self._closed = False
        if self.cache_path.exists():
            with sqlite3.connect(self.cache_path.as_uri()+'?mode=ro', uri=True) as db:
                try:
                    meta = dict(db.execute('SELECT name,value FROM unified_metadata'))
                except sqlite3.DatabaseError as error:
                    raise ValueError('OLD_OR_UNOWNED_CACHE_REJECTED') from error
                if meta != dict(schema=SCHEMA, revision=self.revision):
                    raise ValueError('CACHE_MODEL_IDENTITY_MISMATCH')
        self.cache_path.parent.mkdir(parents=True, exist_ok=True)
        self._db = sqlite3.connect(self.cache_path, timeout=30)
        self._db.execute('PRAGMA journal_mode=WAL')
        self._db.execute('CREATE TABLE IF NOT EXISTS unified_metadata(name TEXT PRIMARY KEY,value TEXT NOT NULL)')
        self._db.executemany('INSERT OR IGNORE INTO unified_metadata VALUES(?,?)',
                             [('schema', SCHEMA), ('revision', self.revision)])
        self._db.execute('CREATE TABLE IF NOT EXISTS unified_responses(input_sha256 TEXT PRIMARY KEY,payload TEXT NOT NULL)')
        self._db.commit()
        self._temporary = tempfile.TemporaryDirectory(prefix='stdfso_assets_')
        try:
            self._loader = AssetLoader(Path(self._temporary.name)/'assets.sqlite3')
            self._base = self._loader._base
            self._contract = copy.deepcopy(self._loader._contract)
            self._model = FixedLinearResponse(self._loader, REPO/'config/linear_response.json')
            self._contract['physical_contract']['revision'] = self.revision
        except Exception:
            self._db.close()
            self._temporary.cleanup()
            raise

    def contract(self):
        return dict(revision=self.revision, **self.config,
                    weights=self._contract['weights'], reference_scales=self._contract['reference_scales'],
                    score_formula='sum(weight * SD / reference_scale)',
                    score_meaning='motion cost; not accident probability',
                    historical_full_heading_cache_compatible=False)

    def validate_record(self, value):
        if value.get('revision') != self.revision or value.get('schema') != SCHEMA:
            raise ValueError('Response revision or semantics mismatch')
        q = value['physics_input']
        if hashlib.sha256(self._base.encode(q).encode()).hexdigest() != value['input_sha256']:
            raise ValueError('Response input identity mismatch')
        sd = value['sd']
        if value['status'] != 'OK' or len(sd) != 4 or any(not math.isfinite(x) or x < 0 for x in sd):
            raise ValueError('Only finite four-component responses can be cached')
        expected = sum(w*s/q for w,s,q in zip(self._contract['weights'],sd,self._contract['reference_scales']))
        if not math.isclose(expected,value['cost'],rel_tol=1e-12,abs_tol=1e-14):
            raise ValueError('Weighted response cost mismatch')
        if not value.get('numerical_validation', {}).get('passed'):
            raise ValueError('Numerical validation missing')

    def import_record(self, value):
        self.validate_record(value)
        self._db.execute('INSERT INTO unified_responses VALUES(?,?)', (value['input_sha256'],encode(value)))

    def evaluate(self, request, *, context=None):
        with _LOCK:
            if self._closed:
                raise RuntimeError('SAFETY_FUNCTION_CLOSED')
            start, cpu = time.perf_counter(), time.process_time()
            self.stats['requests'] += 1
            try:
                q = self._base.physics_input(request)
            except (ValueError, KeyError, TypeError) as error:
                self.stats['failed'] += 1
                return self._result(dict(status='UNSUPPORTED_INPUT',reason=str(error),sd=None,cost=None),None,False,False,start,cpu)
            key = hashlib.sha256(self._base.encode(q).encode()).hexdigest()
            with input_lock(self.cache_path,key):
                if key in self._memo:
                    self.stats['memory_hits'] += 1
                    return self._result(self._memo[key],key,True,False,start,cpu)
                row = self._db.execute('SELECT payload FROM unified_responses WHERE input_sha256=?',(key,)).fetchone()
                if row:
                    value = json.loads(row[0]); self.validate_record(value)
                    self._memo[key] = value
                    self.stats['persistent_hits'] += 1
                    return self._result(value,key,True,False,start,cpu)
                self.stats['component_calls'] += 1
                self.stats['uncached_component_evaluations'] += 1
                self.stats['physical_solve_attempts'] += 1
                try:
                    if q.get('Hm0_m') == 0:
                        sd, diagnostic = [0.,0.,0.,0.], dict(calm=True,roll_closure_relative_residual=0.)
                    else:
                        full = self._model.evaluate(q,dt=self.config['dt_s'],seed=self.config['seed'])
                        heading = wave_heading_statistics(self._model,q)
                        sd = full['sd'][:]
                        sd[1] = heading['wave_induced_heading_sd_deg']
                        diagnostic = dict(roll_closure_relative_residual=full['roll_closure_relative_residual'],
                            roll_damping=full['roll_damping'], heading_equivalent_roll_damping=heading['roll_damping'],
                            heading_frequency_nodes=heading['frequency_nodes'],
                            full_record_heading_sd_deg_diagnostic_only=full['sd'][1])
                        if diagnostic['roll_closure_relative_residual'] > 1e-10:
                            raise ArithmeticError('Equivalent roll residual exceeded')
                    cost = sum(w*s/q for w,s,q in zip(self._contract['weights'],sd,self._contract['reference_scales']))
                    value = dict(status='OK',reason='WAVE_HEADING_FOUR_COMPONENT_FINITE',sd=sd,cost=cost,physics_input=q,input_sha256=key,
                        revision=self.revision,schema=SCHEMA,seed=self.config['seed'],input_unchanged=True,
                        numerical_validation=dict(passed=True,definition='finite fixed linear response and unique scalar roll closure',**diagnostic))
                    self.validate_record(value)
                    self._db.execute('INSERT INTO unified_responses VALUES(?,?)',(key,encode(value)))
                    self._db.commit(); self._memo[key] = value
                except (ArithmeticError,ValueError,RuntimeError) as error:
                    self.stats['failed'] += 1
                    value = dict(status='SOLVER_FAILURE',reason=repr(error),sd=None,cost=None,physics_input=q)
                return self._result(value,key,False,True,start,cpu)

    def _result(self,value,key,hit,attempted,start,cpu):
        value = copy.deepcopy(value)
        value.update(revision=self.revision,input_sha256=key,cache_hit=hit,score=value.get('cost'),
                     wall_seconds=time.perf_counter()-start,python_cpu_seconds=time.process_time()-cpu)
        value['provenance'] = dict(physical_parent=self.revision,weighted_parent=self.revision,
            physical_solve_attempted=attempted,physical_unsafe_inferred=False,interpolated=False,
            runtime_policy=SCHEMA,heading_definition='first-order wave-induced heading SD',
            full_heading_sd_used_in_score=False,original_nonlinear_residual_not_applicable=True)
        return value

    def close(self):
        with _LOCK:
            if self._closed:return
            try:
                if identity()['files'] != self._identity['files']:
                    raise RuntimeError('Response source/configuration changed during session')
            finally:
                self._db.close(); self._loader.close(); self._temporary.cleanup(); self._closed=True

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
