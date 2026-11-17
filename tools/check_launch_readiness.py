"""Nine original-parameter checks plus real hard-interruption/resume; not 810."""
from pathlib import Path
import argparse,json,os,subprocess,sys,time,hashlib
import psutil
from run_case import ROOT
from cohort_recovery import checkpoint,write_json
from check_unified_route_coverage import check_audit

def read(p):return json.loads(Path(p).read_text('utf8'))
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    os.chdir(ROOT);audit=a.output.resolve();audit.mkdir(parents=True,exist_ok=False);cohort=audit/'cohort'
    jobs=read(ROOT/'config/experiment.json')['jobs']
    variants=list(dict.fromkeys((j['method'],j['arm']) for j in jobs))
    goals=['TIME','FUEL','SAFETY_DENSITY','SAFETY_EXPOSURE','COMPOSITE']
    selected=[next(j for j in jobs if (j['method'],j['arm'])==v and j['scene']=='S'+str(i%6+1) and j['goal']==goals[i%5] and j['replicate']==1) for i,v in enumerate(variants)]
    cmd=[sys.executable,'tools/run_experiments.py','--suite','all','--output',str(cohort),'--execute']
    for j in selected:cmd.extend(['--job-id',j['id']])
    with (audit/'interrupted_console.log').open('wb') as log:
        p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        limit=time.monotonic()+180
        while time.monotonic()<limit:
            if p.poll() is not None:raise RuntimeError('Pilot exited before interruption')
            phases=list(cohort.glob('runs/*/phase.txt'))
            if phases and phases[0].read_text()=='SEARCH_EXECUTE':
                dest=phases[0].parent;time.sleep(8);break
            time.sleep(.2)
        else:p.terminate();raise RuntimeError('No search start for interruption check')
        launch=read(dest/'launch.json');before=checkpoint(cohort/'responses.sqlite3')
        first=json.loads((dest/'cpu_trace.jsonl').read_text().splitlines()[0]);root_pid=first['root_pid']
        children=psutil.Process(root_pid).children(recursive=True)
        p.terminate();p.wait(timeout=10)
        try:children.append(psutil.Process(root_pid))
        except psutil.NoSuchProcess:pass
        gone,alive=psutil.wait_procs(children,timeout=10)
        if alive:raise RuntimeError('Run descendants survived owner exit: '+str([x.pid for x in alive]))
    write_json(audit/'interruption.json',dict(passed=True,run=dest.name,cache_start=launch['cache_checkpoint'],cache_at_interruption=before,descendants_terminated=True))
    print('Hard interruption stopped the job tree; resuming original-parameter checks',flush=True)
    with (audit/'resumed_console.log').open('wb') as log:
        result=subprocess.run(cmd+['--resume'],stdout=log,stderr=subprocess.STDOUT)
    if result.returncode:raise RuntimeError('Original-parameter checks failed; inspect resumed_console.log')
    rows=read(cohort/'outcomes.json');assert len(rows)==9
    checks=[]
    for row in rows:
        j=row['job'];d=cohort/'runs'/j['id'];r=read(d/'case/report.json')
        assert j==next(x for x in selected if x['id']==j['id']) and not row['smoke']
        check_audit(r['audit']);check_audit(read(d/'case/seed_audit.json'))
        calls=[json.loads(s) for s in (d/'calls.jsonl').read_text().splitlines()]
        bad=[x for x in calls if x['status']!='OK' and not x.get('control_stop')]
        assert not bad,(j['id'],bad[:1])
        checks.append(dict(id=j['id'],wall_seconds=row['measured_cpu']['wall_seconds'],cpu_seconds=row['measured_cpu']['process_tree_cpu_seconds'],
            cpu60_available=row['observations']['endpoints']['60']['available'],
            fresh=sum(x['physical_solve_attempted'] for x in calls),candidate_counts=r['candidate_evaluations217']['counts']))
    recovery=list((cohort/'interrupted').rglob('recovery.json'));assert len(recovery)==1
    recovered=read(recovery[0]);assert recovered['removed_cache_rows']>0
    # A second resume must skip the nine finished runs, preserving outcomes/cache.
    sha=lambda:hashlib.sha256((cohort/'outcomes.json').read_bytes()).hexdigest()
    old=sha();old_cache=checkpoint(cohort/'responses.sqlite3')
    with (audit/'completed_resume_console.log').open('wb') as log:
        subprocess.run(cmd+['--resume'],stdout=log,stderr=subprocess.STDOUT,check=True)
    assert sha()==old and checkpoint(cohort/'responses.sqlite3')==old_cache
    write_json(audit/'passed.json',dict(passed=True,runs=checks,hard_interruption_recovered=True,recovery=recovered,
        finished_runs_skipped=True,all_810_executed=False,scope='Original-parameter preflight only; separate output and cache from the forthcoming paper cohort'))
    print('Nine original-parameter runs and interruption/resume checks passed',flush=True)
if __name__=='__main__':main()
