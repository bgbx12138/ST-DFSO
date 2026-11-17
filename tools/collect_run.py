"""Read a newly completed run; join measured candidate times to full Job CPU."""
from pathlib import Path
import json, math
import cpu_curve

def read(path):
    return json.loads(Path(path).read_text('utf-8-sig'))

def collect(output):
    out=Path(output); case=out/'case'
    launch=read(out/'launch.json'); job=launch['configuration']; cost=read(out/'cost.json')
    report=read(case/'report.json'); clock=read(case/'search_clock164.json')
    obs=read(case/'common_observations.json'); route=read(case/'result_points.json')
    assert report['run_id']==launch['run_id'] and report['provider_revision']==launch['runtime_revision']
    assert (report['scenario'],report['objective'],report['algorithm'],report['final_arm'],report['seed'])==(job['scene'],job['goal'],job['method'],job['arm'],job['seed'])
    assert obs['closed'] is True and obs['clock_source']=='SYSTEM_NANO_TIME'
    assert clock['pid']==obs['pid']==cost['pid'] and obs['best_route']==route
    assert math.isclose(obs['best_cost'],report['external_cost'],rel_tol=1e-8)
    assert cost['observer_error'] is None and cost['complete_job_accounting'] and cost['returncode']==0
    milestones=[dict(wall_nanos=0,subsegments=0,cost=report['seed_cost'],checkpoint='paid_seed_at_ledger_start')]
    previous=clock['ledger_start_nano']; last=report['seed_cost']; work=0
    for item in obs['improvements']:
        assert previous<=item['nano_lower']<=item['nano_upper']<=clock['after']['nano_after']
        assert work<=item['subsegments']<=job['work'] and math.isfinite(item['objective']) and item['objective']<last
        previous=item['nano_upper']; last=item['objective']; work=item['subsegments']
        milestones.append(dict(wall_nanos=previous-clock['ledger_start_nano'],subsegments=work,cost=last,checkpoint='measured_complete_incumbent'))
    assert math.isclose(last,obs['best_cost'],rel_tol=1e-8)
    timeline=[json.loads(x) for x in (out/'cpu_trace.jsonl').read_text('utf-8').splitlines()]
    assert timeline and all(math.isfinite(p['process_tree_cpu_seconds']) and p['process_tree_cpu_seconds']>=0 for p in timeline)
    assert all(a['process_tree_cpu_seconds']<=b['process_tree_cpu_seconds'] for a,b in zip(timeline,timeline[1:]))
    assert all(p['qpc_before_ticks']<=p['qpc_after_ticks'] for p in timeline)
    assert math.isclose(timeline[-1]['process_tree_cpu_seconds'],cost['process_tree_cpu_seconds'],abs_tol=1e-9)
    if clock['search_started']:
        curve=cpu_curve.curve(clock,timeline,milestones,cost,report['external_cost'])
        points=curve['full_job_points']
    else:
        assert not obs['improvements']
        points=[dict(cost=report['external_cost'],checkpoint='validated_process_completion',full_job_cpu_upper_seconds=cost['process_tree_cpu_seconds'],full_job_cpu_lower_seconds=cost['process_tree_cpu_seconds'])]
        curve=dict(search_started=False,full_job_points=points)
    ends={}
    for cap in (60,10,30,120):
        eligible=[p for p in points if p['full_job_cpu_upper_seconds']<=cap]
        p=eligible[-1] if eligible else None
        ends[str(cap)]=dict(available=p is not None,cost=p['cost'] if p else None,cpu_upper=p['full_job_cpu_upper_seconds'] if p else None,checkpoint=p['checkpoint'] if p else None)
    def metrics(audit):
        assert audit['complete'] and audit['practical'] and audit['native_five_goal_match']
        values=audit['native_five_goal_costs']
        assert all(math.isfinite(v) and v>0 for v in values.values())
        distance=audit['distance_nm']
        assert math.isfinite(distance) and distance>0
        return dict(five_goal_values=values,distance_n_mile=distance,route_complete=True,route_practical=True)
    return dict(source_runtime_signature=launch['source_runtime_signature'],voyage_speed_model=launch['voyage_speed_model'],
                job=job,cohort='new_user_run',smoke=launch['smoke'],scientific_success_received=True,
                final=metrics(report['audit']),seed=metrics(read(case/'seed_audit.json')),
                observations=dict(curve=curve,endpoints=ends),measured_cpu=cost,route=route)
