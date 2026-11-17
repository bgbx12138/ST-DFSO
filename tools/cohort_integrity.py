"""Identity of the source, configuration and loaded Java code for a new cohort."""
from pathlib import Path
import hashlib, json, importlib.metadata as metadata,os,sys
ROOT=Path(__file__).resolve().parents[1]
PACKAGES=('numpy','scipy','capytaine','threadpoolctl','pandas','psutil')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def files():
    paths=set()
    for folder,patterns in [('src',['*.java','*.py']),('tools',['*.py']),('analysis',['*.py']),('config',['*.json']),('target/classes',['*.class']),('target/dependency',['*.jar'])]:
        for pattern in patterns:
            paths.update((ROOT/folder).rglob(pattern))
    paths.update([ROOT/'pom.xml',ROOT/'requirements.txt'])
    # Hash model assets, not just the manifest that describes them.
    for row in json.loads((ROOT/'config/model_files.json').read_text('utf8')):paths.add(ROOT/row['file'])
    spec=ROOT/'config/wave_response_initial_pool.json'
    if spec.exists():paths.add(ROOT/json.loads(spec.read_text('utf8'))['data_file'])
    for scene in json.loads((ROOT/'config/environment_inventory.json').read_text('utf8')):
        paths.update(ROOT/scene['directory']/item['name'] for item in scene['required_files'])
    return sorted(p for p in paths if '__pycache__' not in p.parts)
def identity():
    # The retained hydrodynamic assets use two numerical-library threads.
    # Establish that effective setting before hashing or importing NumPy.
    for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
        if os.environ.get(name) not in (None,'2'):
            raise RuntimeError('Frozen numerical thread setting must be 2; unset '+name)
        os.environ[name]='2'
    rows={p.relative_to(ROOT).as_posix():sha(p) for p in files()}
    packages={p:metadata.version(p) for p in PACKAGES}
    for key in ('JAVA_TOOL_OPTIONS','_JAVA_OPTIONS','JDK_JAVA_OPTIONS'):
        if os.environ.get(key):raise RuntimeError('Unset Java override before a frozen run: '+key)
    weights=[float(x) for x in os.environ.get('R1_COMPONENT_RELATIVE_WEIGHTS','1.5,1.25,1,1').split(',')]
    if weights!=[1.5,1.25,1.,1.]:raise RuntimeError('Frozen paper weights must be 1.5,1.25,1,1')
    environment={k:os.environ.get(k) for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS','PYTHONHASHSEED')}
    value=dict(files=rows,python_packages=packages,python_version=sys.version,environment=environment)
    return dict(signature=hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest(),**value)
def signature():return identity()['signature']
def verify_freeze():
    frozen=json.loads((ROOT/'experiment_freeze.json').read_text('utf8'));actual=identity()
    expected=frozen['identity']
    # Class-file debug tables and compiler output can differ between valid Java
    # 11 builds. Verify distributable sources/assets/dependencies here; identity()
    # still records the actual class bytes for each run and resume decision.
    public=lambda values:{p:h for p,h in values.items() if not p.startswith('target/classes/')}
    wanted,found=public(expected['files']),public(actual['files'])
    changed=sorted(p for p in set(wanted)|set(found) if wanted.get(p)!=found.get(p))
    metadata_changed=[k for k in ('python_packages','python_version','environment') if expected[k]!=actual[k]]
    if changed or metadata_changed:
        raise RuntimeError('Frozen source, input or environment differs: '+repr(changed[:20])+repr(metadata_changed))
    missing=[p for p in found if p.startswith('src/main/java/') and p.endswith('.java')
             and not (ROOT/'target/classes'/p[len('src/main/java/'):].replace('.java','.class')).is_file()]
    if missing:raise RuntimeError('Build the Java sources before running: '+repr(missing[:10]))
    if not frozen.get('preflight_passed'):raise RuntimeError('Preflight has not passed')
    return frozen
