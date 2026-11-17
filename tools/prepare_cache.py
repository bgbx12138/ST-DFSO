"""Install exact common-input responses for the selected model revision."""
from pathlib import Path
import argparse,hashlib,json,os,sqlite3,sys,time
ROOT=Path(__file__).resolve().parents[1]

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def encode(v):return json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False)

def prepare(cache):
    cache=Path(cache).resolve()
    if any(cache.is_relative_to(ROOT/n) for n in ('src','config','data','.git')):raise ValueError('Cache must be outside published inputs')
    if cache.exists():raise FileExistsError('Initial-cache import requires a new cache: '+str(cache))
    revised=(ROOT/'config/voyage_speed.json').exists()
    spec_path=ROOT/'config/wave_response_initial_pool.json'
    if not spec_path.is_file():
        raise RuntimeError('Revised voyage model initial pool is not prepared; full experiments are not ready')
    spec=json.loads(spec_path.read_text('utf-8'))
    if sha(ROOT/'config/model_files.json')!=spec['compatible_model_manifest_sha256']:
        raise ValueError('The model differs from the verified publication model; historical response import is not authorised for a changed model')
    path=ROOT/spec['data_file']
    if sha(path)!=spec['data_sha256']:raise ValueError('Initial response data changed')
    rows=json.loads(path.read_text('utf-8'));assert len(rows)==spec['records'] and len(rows)>0
    if revised:
        assert set(spec['input_routes'])=={f'S{i}' for i in range(1,7)}
        assert sha(ROOT/'config/voyage_speed.json')==spec['speed_config_sha256']
        for scene,expected in spec['input_routes'].items():
            assert sha(ROOT/f'config/seed_routes/{scene}.json')==expected
    os.chdir(ROOT);sys.path.insert(0,str(ROOT/'src/main/python'))
    from release_integrity import verify_model_files
    verify_model_files()
    from navigation_safety.unified_response import SafetyFunction
    start=time.process_time()
    with SafetyFunction(cache) as f:
        revision=f._contract['physical_contract']['revision']; runtime=f.revision
    if revised and (revision!=spec['physical_revision'] or runtime!=spec['runtime_revision']):
        raise RuntimeError('Initial pool does not match this revised response model')
    with SafetyFunction(cache) as f:
        for row in rows:
            f.import_record(row)
        f._db.commit()
    receipt=dict(records=len(rows),source_data_sha256=spec['data_sha256'],
                 historical_payload_revision=None if revised else spec['historical_physical_revision'],lookup_revision=revision,
                 runtime_revision=runtime,import_cpu_seconds=time.process_time()-start,
                 fresh_physical_solves=0,scope=spec['cost_scope'])
    cache.with_suffix('.initial.json').write_text(json.dumps(receipt,indent=2),'utf-8')
    return receipt

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    print(json.dumps(prepare(a.output),indent=2))
if __name__=='__main__':main()
