"""Regression cases for the integrated response; no optimisation batch."""
from pathlib import Path
import argparse,json,os,sys,time,math,sqlite3
from run_case import ROOT

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    os.chdir(ROOT);out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(ROOT/'src/main/python'))
    from navigation_safety.unified_response import SafetyFunction
    cases=json.loads((ROOT/'config/response_checks.json').read_text('utf8'))
    base=dict(U_m_s=9.27188031610877,mu_deg=176.31394124820474,Hm0_m=2.33,Tp_s=9.25)
    for heading in (0.,.01,45.,90.,135.,179.99,180.):
        cases.append(dict(id=f'heading_{heading}',query=dict(base,mu_deg=heading)))
    for delta in (-.02,-.001,.001,.02):
        cases.append(dict(id=f'speed_delta_{delta}',query=dict(base,U_m_s=base['U_m_s']+delta)))
    cases.append(dict(id='calm',query=dict(base,Hm0_m=0.)))
    seen=set();cases=[x for x in cases if not (json.dumps(x['query'],sort_keys=True) in seen or seen.add(json.dumps(x['query'],sort_keys=True)))]
    rows=[];start=time.perf_counter()
    with SafetyFunction(out/'responses.sqlite3') as f:
        (out/'contract.json').write_text(json.dumps(f.contract(),indent=2),'utf8')
        for case in cases:
            value=f.evaluate(case['query']);assert value['status']=='OK',value
            assert not value['cache_hit'];f.validate_record(value)
            assert value['provenance']['full_heading_sd_used_in_score'] is False
            rows.append(dict(**case,result=value))
        for row in reversed(rows):
            value=f.evaluate(row['query']);assert value['cache_hit'] and value['sd']==row['result']['sd']
        assert rows[-1]['result']['sd']==[0.,0.,0.,0.]
        for invalid in [dict(base,U_m_s=0.),dict(base,mu_deg=181.),dict(base,Hm0_m=-1.)]:
            value=f.evaluate(invalid);assert value['status']=='UNSUPPORTED_INPUT' and value['cost'] is None
        revision=f.revision
    with SafetyFunction(out/'responses.sqlite3') as f:
        for row in rows:
            value=f.evaluate(row['query']);assert value['cache_hit'] and value['sd']==row['result']['sd']
    with sqlite3.connect(out/'old_cache.sqlite3') as db:
        db.execute('CREATE TABLE responses(key TEXT PRIMARY KEY,payload TEXT)')
    try:
        SafetyFunction(out/'old_cache.sqlite3')
    except ValueError as error:assert 'OLD_OR_UNOWNED' in str(error)
    else:raise AssertionError('Historical cache accepted')
    (out/'cases.json').write_text(json.dumps(rows,indent=2,allow_nan=False),'utf8')
    summary=dict(passed=True,revision=revision,cases=len(rows),seconds=time.perf_counter()-start,
                 fresh_seconds=[r['result']['wall_seconds'] for r in rows],
                 old_cache_rejected=True,memory_and_reopen_identical=True,invalid_inputs_unpriced=True,
                 full_810_executed=False)
    (out/'passed.json').write_text(json.dumps(summary,indent=2),'utf8');print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
