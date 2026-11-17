"""Reproduce comparisons from the released accepted outcomes; no optimisation."""
from pathlib import Path
import argparse,json
import statistical_methods as f

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--input',type=Path,help='Optional outcomes.json from a new complete 810-run cohort')
    args=ap.parse_args()
    root=Path(__file__).resolve().parents[1]
    rows=f.read(args.input or root/'data/results/accepted_runs.json');protocol=f.read(root/'config/experiment.json')
    if len(rows)!=810 or any(r.get('smoke',False) for r in rows):ap.error('Analysis requires all 810 paper configurations, not smoke runs or subsets')
    expected={j['id']:j for j in protocol['jobs']}
    if len({r['job']['id'] for r in rows})!=810 or any(r['job']!=expected.get(r['job']['id']) for r in rows):
        ap.error('Each planned configuration must occur exactly once with unchanged settings')
    if args.input:
        identities={r.get('source_runtime_signature') for r in rows}
        if len(identities)!=1 or None in identities:ap.error('Mixed or missing runtime identities')
        if any(not r.get('scientific_success_received') for r in rows):ap.error('Incomplete run outcomes')
    cohort='paper_810'
    args.output.mkdir(parents=True,exist_ok=False)
    comparisons,pairs=f.contrast_analysis(rows,protocol,cohort)
    detail,cross,crosspairs=f.cross_goals(rows,protocol,cohort)
    for name,data in [('CPU_COMPARISONS160',comparisons),('PLANNED_PAIRS2880',pairs),('FINAL_CROSS_GOAL_ROWS1800',detail),('FINAL_CROSS_GOAL_SUMMARY100',cross),('FINAL_CROSS_GOAL_PAIRED75',crosspairs)]:f.csvout(args.output/(name+'.csv'),data)
if __name__=='__main__':main()
