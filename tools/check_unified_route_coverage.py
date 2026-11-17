"""Full Cartesian reduced coverage: six scenes x five goals x nine variants.

Concurrent runs test integration only; their outcomes are not scientific
comparisons. All measured process CPU is retained, and no 810-run batch starts.
"""
from pathlib import Path
import argparse,json,os,math,time
from concurrent.futures import ProcessPoolExecutor,as_completed
from run_case import ROOT,run_job
from cohort_integrity import signature

def read(p):return json.loads(Path(p).read_text('utf8'))
def check_audit(a):
    assert a['complete'] and a['practical'] and a['native_five_goal_match']
    close=lambda x,y:math.isclose(x,y,rel_tol=1e-8,abs_tol=1e-8)
    assert close(a['risk_density']*a['distance_nm'],a['risk_total'])
    assert close(sum(a['channel_exposure_partial']),a['risk_total'])
    w=a['objective_weights']
    assert close(w['safety']*a['risk_total']+w['time']*a['normalized_time_integral']+w['fuel']*a['normalized_fuel_integral'],a['composite'])
    assert close(a['time_hours'],a['navigation_time_hours']) and close(a['fuel_j'],a['navigation_fuel_j'])
    assert all(x['provenance']['full_heading_sd_used_in_score'] is False for x in a['response_queries'].values())

def run_one(job,out,stamp,millis):
    os.chdir(ROOT);assert signature()==stamp
    dest=Path(out)/job['id']
    code=run_job(job,dest,cache_path=Path(out)/'responses.sqlite3',smoke=True,smoke_millis=millis)
    if code:raise RuntimeError('Coverage execution failed: '+job['id'])
    outcome=read(dest/'outcome.json');report=read(dest/'case/report.json')
    assert outcome['smoke'] and outcome['scientific_success_received']
    check_audit(report['audit']);check_audit(read(dest/'case/seed_audit.json'))
    calls=[json.loads(s) for s in (dest/'calls.jsonl').read_text().splitlines()]
    bad=[x for x in calls if x['status']!='OK' and not x.get('control_stop')]
    assert not bad,bad[:2]
    seed=read(dest/'case/seed_points.json');seedkey=''.join(f'{p["i"]},{p["j"]};' for p in seed)
    candidates=report['candidate_evaluations217']
    complete_new=[x for x in candidates['rows'] if x['status']=='COMPLETE' and x['route_key']!=seedkey]
    obs=read(dest/'case/common_observations.json')
    return dict(id=job['id'],scene=job['scene'],goal=job['goal'],method=job['method'],arm=job['arm'],
        revision=report['provider_revision'],cpu_seconds=outcome['measured_cpu']['process_tree_cpu_seconds'],
        wall_seconds=outcome['measured_cpu']['wall_seconds'],candidate_counts=candidates['counts'],
        complete_changed_route_observations=len(complete_new),improvements=len(obs['improvements']),
        search_requests=sum(x['phase']=='SEARCH_EXECUTE' for x in calls),
        fresh_search_responses=sum(x['phase']=='SEARCH_EXECUTE' and x['physical_solve_attempted'] for x in calls),
        failures=len(bad),five_goal_and_cpu_audits_passed=True)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--workers',type=int,default=3);ap.add_argument('--millis',type=int,default=5000)
    ap.add_argument('--limit',type=int);ap.add_argument('--job-id',action='append');a=ap.parse_args();os.chdir(ROOT)
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    jobs=[x for x in read(ROOT/'config/experiment.json')['jobs'] if x['replicate']==1]
    assert len(jobs)==270 and len({(x['scene'],x['goal'],x['method'],x['arm']) for x in jobs})==270
    if a.job_id:
        jobs=[j for j in jobs if j['id'] in a.job_id]
        assert {j['id'] for j in jobs}==set(a.job_id)
    if a.limit:jobs=jobs[:a.limit]
    from prepare_cache import prepare
    prepare(out/'responses.sqlite3')
    stamp=signature();start=time.perf_counter()
    (out/'selection.json').write_text(json.dumps(dict(jobs=jobs,signature=stamp,smoke=True,workers=a.workers,millis=a.millis),indent=2),'utf8')
    results=[];errors=[]
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        pending={pool.submit(run_one,job,str(out),stamp,a.millis):job for job in jobs}
        for future in as_completed(pending):
            try:results.append(future.result())
            except Exception as error:
                errors.append(dict(id=pending[future]['id'],error=repr(error)))
                (out/'failures.json').write_text(json.dumps(errors,indent=2),'utf8')
            (out/'progress.json').write_text(json.dumps(results,indent=2),'utf8')
            print(f'Coverage {len(results)}/{len(jobs)} complete',flush=True)
    assert len({r['revision'] for r in results})<=1 and signature()==stamp
    summary=dict(passed=not errors,full_cartesian=len(results)==270,runs=len(results),errors=errors,signature=stamp,
        revision=results[0]['revision'] if results else None,seconds=time.perf_counter()-start,
        fresh_search_responses=sum(r['fresh_search_responses'] for r in results),
        complete_changed_route_observations=sum(r['complete_changed_route_observations'] for r in results),
        runs_with_changed_complete_routes=sum(r['complete_changed_route_observations']>0 for r in results),
        runs_with_improvements=sum(r['improvements']>0 for r in results),
        numerical_failures=sum(r['failures'] for r in results),full_810_executed=False,
        scope='Reduced integration tests with shared exact cache; not performance comparisons or paper results')
    (out/'passed.json').write_text(json.dumps(summary,indent=2),'utf8');print(json.dumps(summary,indent=2))
    if errors:raise RuntimeError('Coverage failures retained; see failures.json')
if __name__=='__main__':main()
