"""Run one user-selected configuration. Default: print settings, do not execute."""
from pathlib import Path
import argparse,hashlib,json,os,re,shutil,subprocess,sys,uuid

ROOT=Path(__file__).resolve().parents[1]
ALGORITHMS={'ST-DFSO':'FSA_SELECTED','ACO':'ACO','IAFSA':'ASV_IAFSA','ILS':'WINDOW_ILS'}
OBJECTIVES={'TIME':'TIME','ENERGY':'FUEL','MEAN_MOTION':'SAFETY_DENSITY',
            'EXPOSURE':'SAFETY_EXPOSURE','COMPOSITE':'COMPOSITE'}
VARIANTS={'FULL':'OFF_SF','MINUS_LL':'MINUS_LL','MINUS_AV':'MINUS_AV',
          'MINUS_FG':'MINUS_FG','MINUS_ER':'MINUS_ER','CORE':'CORE'}
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser()
    selection=ap.add_mutually_exclusive_group(required=True)
    selection.add_argument('--job-id',help='A configuration ID in config/experiment.json')
    selection.add_argument('--scenario',choices=[f'S{i}' for i in range(1,7)],help='Scenario number in the paper')
    ap.add_argument('--objective',choices=OBJECTIVES,help='Default: TIME')
    ap.add_argument('--algorithm',choices=ALGORITHMS,help='Default: ST-DFSO')
    ap.add_argument('--replicate',type=int,choices=[1,2,3],help='Recorded paired seed, default: 1')
    ap.add_argument('--variant',choices=VARIANTS,help='ST-DFSO mechanism setting, default: FULL')
    ap.add_argument('--output',type=Path,required=True,help='New output directory')
    ap.add_argument('--execute',action='store_true')
    ap.add_argument('--cache',type=Path,help='Shared cohort response cache; default: a fresh cache for this run')
    ap.add_argument('--smoke',action='store_true',help='One iteration with a reduced work cap; not a paper experiment')
    ap.add_argument('--cold',action='store_true',help='Start without the paper initial response pool; a separate computational condition')
    a=ap.parse_args()
    jobs=json.loads((ROOT/'config/experiment.json').read_text('utf-8'))['jobs']
    if a.job_id:
        if any(v is not None for v in (a.objective,a.algorithm,a.replicate,a.variant)):
            ap.error('Use either --job-id alone or --scenario with algorithm/objective options')
        job=next((x for x in jobs if x['id']==a.job_id),None)
    else:
        method=ALGORITHMS[a.algorithm or 'ST-DFSO'];arm=VARIANTS[a.variant or 'FULL']
        if method!='FSA_SELECTED' and arm!='OFF_SF':
            ap.error('Mechanism variants apply only to ST-DFSO')
        matches=[x for x in jobs if x['scene']==a.scenario
                 and x['goal']==OBJECTIVES[a.objective or 'TIME']
                 and x['method']==method and x['arm']==arm
                 and x['replicate']==(a.replicate or 1)]
        if len(matches)!=1:ap.error('Expected exactly one matching recorded configuration')
        job=matches[0]
    if job is None:ap.error('Unknown configuration ID')
    print(json.dumps({'configuration':job,'execute':a.execute,'output':str(a.output),'smoke':a.smoke},indent=2))
    if not a.execute:return 0
    return run_job(job,a.output,cache_path=a.cache,smoke=a.smoke,cold=a.cold)

def run_job(job,output,cache_path=None,smoke=False,cold=False,smoke_millis=1000):
    if os.name!='nt':raise RuntimeError('Paper CPU measurements require Windows and Java 11; see docs/RUNNING.md')
    from cohort_integrity import signature
    source_signature=signature()
    job=dict(job)
    if smoke:job.update(population=2,iterations=1,work=1000,millis=smoke_millis)
    scene=next(x for x in json.loads((ROOT/'config/environment_inventory.json').read_text('utf-8')) if x['scenario_id']==job['scene'])
    out=Path(output).resolve()
    if any(out.is_relative_to(ROOT/name) for name in ('data','config','src','.git')):
        raise RuntimeError('Choose an output directory outside the source, configuration and published data directories')
    if out.exists():raise RuntimeError('Output directory already exists; use a new directory')
    java=shutil.which('java')
    jars=sorted((ROOT/'target/dependency').glob('*.jar'))
    if java is None or not jars or not (ROOT/'target/classes/Navigation/Navigation3D/fsa/TableEntry211.class').is_file():
        raise RuntimeError('Install Java 11 and run the Maven build/dependency command first')
    files=[ROOT/scene['directory']/f['name'] for f in scene['required_files']]
    missing=[str(p) for p in files if not p.is_file()]
    if missing:raise RuntimeError('Missing environmental inputs: '+', '.join(missing))
    # Hash the files actually supplied; do not substitute historical identities.
    inputs={p.name:digest(p) for p in files}
    env_hash=hashlib.sha256(json.dumps(inputs,sort_keys=True).encode()).hexdigest()
    graph_hash=inputs['obstacle_map.json']
    route=ROOT/'config/seed_routes'/f"{job['scene']}.json"
    version=subprocess.run([java,'-version'],capture_output=True,text=True)
    if version.returncode or not re.search(r'(?:version\s+"|openjdk\s+)11(?:[.\s"]|$)',version.stderr+version.stdout):
        raise RuntimeError('The measured launcher requires Java 11 on PATH')
    out.mkdir(parents=True)
    os.chdir(ROOT)
    sys.path.insert(0,str(ROOT/'src/main/python'))
    from navigation_safety.unified_response import SafetyFunction
    cache=Path(cache_path).resolve() if cache_path else out/'responses.sqlite3'
    if any(cache.is_relative_to(ROOT/name) for name in ('data','config','src','.git')):raise RuntimeError('Cache must be outside published inputs')
    cache.parent.mkdir(parents=True,exist_ok=True)
    cache_existed=cache.exists()
    if not cache_existed and not cold:
        from prepare_cache import prepare
        prepare(cache)
    initial_receipt=cache.with_suffix('.initial.json')
    if cache_existed and not cold and not initial_receipt.is_file():
        raise RuntimeError('This existing cache has no paper initial-pool receipt; use a new output/cache or explicitly select --cold')
    if not cold:
        spec_path=ROOT/'config/wave_response_initial_pool.json'
        expected_pool=json.loads(spec_path.read_text('utf8'))
        actual_pool=json.loads(initial_receipt.read_text('utf8'))
        if actual_pool['source_data_sha256']!=expected_pool['data_sha256'] or actual_pool['records']!=expected_pool['records']:
            raise RuntimeError('Existing cache was not initialised with the selected common-input pool')
    with SafetyFunction(cache) as f:revision=f.revision
    run_id='PUBLIC_'+uuid.uuid4().hex
    cmd=[java,'-Dfsa.auxRng.time=false','-Dnav.inputRoot='+str(ROOT/'data/environment'),'-Dnav.missingPolicy=ocean_idw_v1','-Dnav220.screen=false','-Dnav.r1.rpcTimeoutSeconds=330','-Dnav.r1.tableRevision='+revision,'-cp',os.pathsep.join([str(ROOT/'target/classes')]+[str(x) for x in jars]),'Navigation.Navigation3D.fsa.TableEntry211',sys.executable,str(ROOT/'src/main/python/navigation_safety/provider.py'),str(cache),str(out/'backend.json'),'Navigation.benchmark.NativeFinalRunner351']
    opts={'scenario':job['scene'],'objective':job['goal'],'algorithm':job['method'],'arm':job['arm'],'run-id':run_id,'seed-json':str(route),'output':str(out/'case'),'seed':job['seed'],'subsegments':job['work'],'population':job['population'],'iterations':job['iterations'],'millis':job['millis'],'environment-sha':env_hash,'graph-sha':graph_hash,'route-sha':digest(route),'supply-sha':digest(ROOT/'config/model_files.json'),'evaluator-sha':source_signature}
    for k,v in opts.items():cmd.extend(['--'+k,str(v)])
    env=os.environ.copy()
    env['FSA_AVAILABILITY_PHASE']=str(out/'phase.txt')
    env['FSA_REQUEST_CONTEXT211']=str(out/'context.json')
    env['FSA_RESPONSE_TRACE']=str(out/'responses.jsonl')
    env['FSA_AVAILABILITY_CALLS']=str(out/'calls.jsonl')
    receipt={'run_id':run_id,'configuration_id':job['id'],'configuration':job,'smoke':smoke,'cache':str(cache),'cache_initially_exists':cache_existed,'environment_files':inputs,'runtime_revision':revision,'java_version':version.stderr.strip() or version.stdout.strip(),'command':cmd,'note':'New measured run; cumulative child Job CPU includes Java, the Python response provider and exited descendants. Observer CPU is separate.'}
    receipt['initial_response_pool']=json.loads(initial_receipt.read_text('utf-8')) if initial_receipt.is_file() else None
    receipt['cold_requested']=cold
    receipt['source_runtime_signature']=source_signature
    from cohort_recovery import checkpoint,write_json
    receipt['cache_checkpoint']=checkpoint(cache)
    receipt['voyage_speed_model']=json.loads((ROOT/'config/voyage_speed.json').read_text('utf8'))['model']
    (out/'launch.json').write_text(json.dumps(receipt,indent=2),'utf-8')
    from cpu_observer import run_with_qpc
    cost=run_with_qpc(cmd,ROOT,out/'console.log',env,86400,out/'cpu_trace.jsonl')
    (out/'cost.json').write_text(json.dumps(cost,indent=2),'utf-8')
    if signature()!=source_signature:
        raise RuntimeError('Source, configuration or model assets changed during this run; inspect its retained output')
    if cost['returncode']!=0 or cost['observer_error'] or not cost['complete_job_accounting']:
        print(json.dumps({'configuration':job['id'],'failed':True,'cost':cost}),flush=True)
        return 1
    from collect_run import collect
    result=collect(out)
    write_json(out/'outcome.json',result)
    print(json.dumps({'configuration':job['id'],'completed':True,'cpu_seconds':cost['process_tree_cpu_seconds'],'smoke':smoke}),flush=True)
    return 0
if __name__=='__main__':raise SystemExit(main())
