"""Calculate the current model response at the published irregular-wave conditions."""
from pathlib import Path
import argparse, csv, json, os, sys
ROOT=Path(__file__).resolve().parents[1]


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();out=args.output.resolve()
    if any(out.is_relative_to(ROOT/n) for n in ('src','config','data','.git')):
        ap.error('Choose outputs/ or a directory outside the repository')
    out.mkdir(parents=True,exist_ok=False);os.chdir(ROOT)
    sys.path.insert(0,str(ROOT/'src/main/python'))
    from navigation_safety.unified_response import SafetyFunction
    cases=[x for x in json.loads((ROOT/'config/response_checks.json').read_text('utf8')) if x['id'].startswith('source_')]
    observed=json.loads((ROOT/'config/response_measurements.json').read_text('utf8'))
    rows=[];responses=[]
    with SafetyFunction(out/'responses.sqlite3') as model:
        for case in cases:
            result=model.evaluate(case['query'])
            if result['status']!='OK':raise RuntimeError(result)
            responses.append(dict(**case,result=result))
            direction=str(int(case['query']['mu_deg']))
            for i in (0,2,3):
                target=observed['standard_deviations'][direction][i]
                prediction=result['sd'][i]
                rows.append(dict(direction_deg=int(direction),component=observed['component_order'][i],unit=observed['units'][i],measured_SD=target,predicted_SD=prediction,relative_difference_percent=100*(prediction/target-1)))
        contract=model.contract()
    with (out/'comparisons.csv').open('w',encoding='utf8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (out/'responses.json').write_text(json.dumps(dict(contract=contract,source=observed,rows=responses),indent=2),'utf8')
    print('Calculated four source conditions. All roll/sway/surge differences are in comparisons.csv.')
    print('Total measured heading SD and wave-induced yaw SD have different definitions and are not compared.')
    print('This calculation reports errors; it does not certify safety or declare all components validated.')


if __name__=='__main__':main()
