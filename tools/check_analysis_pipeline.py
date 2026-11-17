"""Regression checks on recorded paper results and statistical formulas."""
from pathlib import Path
import json, math, sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'analysis'))
import statistical_methods as s

def main():
    protocol=s.read(ROOT/'config/experiment.json');rows=s.read(ROOT/'data/results/accepted_runs.json')
    jobs=protocol['jobs'];assert len(jobs)==len(rows)==810
    assert len({(j['scene'],j['goal'],j['method'],j['arm'],j['replicate']) for j in jobs})==810
    assert sum('MAIN' in j['views'] for j in jobs)==360
    assert sum('ABLATION' in j['views'] for j in jobs)==540
    assert sum(set(j['views'])=={'MAIN','ABLATION'} for j in jobs)==90
    comparisons,pairs=s.contrast_analysis(rows,protocol,'PIPELINE_REGRESSION_ONLY')
    detail,cross,crosspairs=s.cross_goals(rows,protocol,'PIPELINE_REGRESSION_ONLY')
    assert (len(comparisons),len(pairs),len(detail),len(cross),len(crosspairs))==(160,2880,1800,100,75)
    for c in comparisons:
        pp=[p for p in pairs if all(p[k]==c[k] for k in ('family','endpoint_seconds','goal','target'))]
        assert len(pp)==18 and len({(p['scene'],p['replicate']) for p in pp})==18
        gains=[]
        for p in pp:
            if p['paired_available']:
                g=100*(p['target_cost']-p['reference_cost'])/p['target_cost'];gains.append(g)
                assert math.isclose(g,p['gain_percent'],abs_tol=1e-12)
            else:assert p['gain_percent'] is None
        if gains:
            assert math.isclose(np.mean(gains),c['mean_gain_percent'],abs_tol=1e-12)
            assert math.isclose(np.median(gains),c['median_gain_percent'],abs_tol=1e-12)
        assert c['wins']+c['losses']+c['ties']==c['available_pairs']
    fixture=[dict(wilcoxon_two_sided_p=p) for p in [.01,.04,.03]]
    s.holm(fixture,3);assert np.allclose([x['holm_p'] for x in fixture],[.03,.06,.06])
    zero=s.estimate([0]*18,[0]*18,[[0]*3]*6,np.random.default_rng(1))
    assert zero['wilcoxon_two_sided_p']==1 and zero['ties']==18
    # Counterexample: negative extremes are retained in the mean, not removed.
    extreme=s.estimate([1]*17+[-100],[1]*17+[-100],[[1]*3]*5+[[1,1,-100]],np.random.default_rng(1))
    assert extreme['median_gain_percent']==1 and extreme['mean_gain_percent']<0
    out=dict(passed=True,scope='Recorded-data regression and statistical formula checks',
             planned_runs=810,main_runs=360,ablation_rows=540,shared_runs=90,
             comparisons=160,paired_rows=2880,final_measure_rows=1800,
             paired_percentage_formula=True,mean_and_median=True,holm=True,extremes_retained=True)
    (ROOT/'outputs/analysis_pipeline_check.json').write_text(json.dumps(out,indent=2),'utf8');print(json.dumps(out,indent=2))
if __name__=='__main__':main()
