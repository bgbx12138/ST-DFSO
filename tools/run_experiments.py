"""Sequential paper runs, with one method-independent response cache per cohort."""
from pathlib import Path
import argparse, hashlib, json, os
from cohort_recovery import cohort_lock,recover,write_json
from run_case import ROOT, run_job, OBJECTIVES

def runtime_signature():
    from cohort_integrity import signature
    return signature()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--suite',choices=['all','main','ablation'],default='all')
    ap.add_argument('--scenario',choices=[f'S{i}' for i in range(1,7)])
    ap.add_argument('--objective',choices=OBJECTIVES)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--execute',action='store_true')
    ap.add_argument('--resume',action='store_true',help='Continue the identical selection and shared cache after interruption')
    ap.add_argument('--smoke',action='store_true',help='Short integration checks, not paper experiments')
    ap.add_argument('--job-id',action='append',help='Explicit subset for preflight only')
    ap.add_argument('--cold',action='store_true',help='Separate empty-cache condition; not the paper initial state')
    a=ap.parse_args()
    config=ROOT/'config/experiment.json'; protocol=json.loads(config.read_text('utf-8'))
    jobs=[j for j in protocol['jobs'] if (a.suite=='all' or a.suite.upper() in j['views']) and (not a.scenario or j['scene']==a.scenario) and (not a.objective or j['goal']==OBJECTIVES[a.objective])]
    if a.job_id:
        by_id={j['id']:j for j in jobs};jobs=[by_id[k] for k in a.job_id]
    assert jobs and len({j['id'] for j in jobs})==len(jobs)
    selection=dict(suite=a.suite,scenario=a.scenario,objective=a.objective,smoke=a.smoke,cold=a.cold,
                   configuration_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),runtime_signature=runtime_signature(),job_ids=[j['id'] for j in jobs])
    print(json.dumps(dict(runs=len(jobs),selection=selection,execute=a.execute),indent=2),flush=True)
    if not a.execute:return 0
    if os.name!='nt':ap.error('Full Job CPU accounting requires Windows and Java 11')
    out=a.output.resolve()
    if any(out.is_relative_to(ROOT/n) for n in ('data','config','src','.git')):ap.error('Use outputs/ or a directory outside the repository')
    manifest=out/'selection.json'; cache=out/'responses.sqlite3'
    if a.resume:
        if not manifest.is_file() or json.loads(manifest.read_text('utf-8'))!=selection:ap.error('Resume requires the identical saved selection')
        if any(out.glob('runs/*/launch.json')) and not cache.exists():ap.error('The shared response cache is missing; start a new cohort')
    else:
        out.mkdir(parents=True,exist_ok=False)
        manifest.write_text(json.dumps(selection,indent=2),'utf-8')
    (out/'runs').mkdir(exist_ok=True)
    with cohort_lock(out):
        return execute_jobs(jobs,out,cache,selection,a)

def execute_jobs(jobs,out,cache,selection,a):
    for index,job in enumerate(jobs,1):
        if runtime_signature()!=selection['runtime_signature']:
            raise RuntimeError('Source, configuration or model assets changed during the cohort')
        dest=out/'runs'/job['id']
        if dest.exists():
            result=dest/'outcome.json'
            if not a.resume:raise RuntimeError('Output exists: '+str(dest))
            if not result.is_file():
                action=recover(dest,cache,out)
                print('Recovery: '+action+' '+job['id'],flush=True)
                if not result.exists():
                    if run_job(job,dest,cache_path=cache,smoke=a.smoke,cold=a.cold):return 1
                    continue
            saved=json.loads(result.read_text('utf-8'))
            expected=dict(job)
            if a.smoke:expected.update(population=2,iterations=1,work=1000,millis=1000)
            if saved['job']!=expected or saved['smoke']!=a.smoke or not saved['scientific_success_received']:raise RuntimeError('Invalid saved outcome')
            print(f'[{index}/{len(jobs)}] completed: {job["id"]}',flush=True)
            continue
        print(f'[{index}/{len(jobs)}] running: {job["id"]}',flush=True)
        if run_job(job,dest,cache_path=cache,smoke=a.smoke,cold=a.cold):return 1
    rows=[json.loads((out/'runs'/j['id']/'outcome.json').read_text('utf-8')) for j in jobs]
    write_json(out/'outcomes.json',rows)
    print(f'Completed {len(rows)} runs: {out}',flush=True)
    return 0

if __name__=='__main__':raise SystemExit(main())
