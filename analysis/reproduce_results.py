"""Reproduce manuscript statistics from the complete recorded experiment.

Pair classification and signed-rank tests use the same symmetric tolerance.
Percentage gains, means, medians and bootstrap intervals retain unmodified costs.
"""
from pathlib import Path
import csv, json, hashlib, warnings, shutil, sys
from collections import defaultdict
import numpy as np
from scipy.stats import wilcoxon
import scipy

import argparse, tempfile, subprocess
ROOT = Path(__file__).resolve().parents[1]
HERE = None
BATCH = None
INPUT = None
GOALS = ['TIME','FUEL','SAFETY_DENSITY','SAFETY_EXPOSURE','COMPOSITE']
LABEL = dict(zip(GOALS,['T','E','D','R','C']))
TARGET_LABEL = {'ACO':'ACO','ASV_IAFSA':'IAFSA','WINDOW_ILS':'window ILS',**{'MINUS_'+x:'without '+x for x in ['LL','AV','FG','ER']},'CORE':'CORE'}
RULE = 'abs(B-S) <= 1e-12 * max(abs(B),abs(S),1)'

def readcsv(p):
    with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def writecsv(name,rows):
    with (HERE/'data'/name).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def classify(s,b):
    diff=b-s; tol=1e-12*max(abs(s),abs(b),1.)
    return (0. if abs(diff)<=tol else diff),tol
def pval(d):
    if not any(d):return 1.
    with warnings.catch_warnings():
        warnings.simplefilter('ignore',UserWarning)
        return float(wilcoxon(d,zero_method='wilcox',correction=False,alternative='two-sided',mode='approx').pvalue)
def holm(rows):
    order=sorted(range(len(rows)),key=lambda i:rows[i]['wilcoxon_two_sided_p'])
    run=0.
    for rank,i in enumerate(order):
        run=max(run,min(1.,(len(rows)-rank)*rows[i]['wilcoxon_two_sided_p']))
        rows[i]['holm_p']=run; rows[i]['significant_holm_05']=run<.05

def main():
    sources=[INPUT/n for n in ['CPU_COMPARISONS160.csv','PLANNED_PAIRS2880.csv','FINAL_CROSS_GOAL_PAIRED75.csv','FINAL_CROSS_GOAL_ROWS1800.csv']]+[ROOT/'data/results/accepted_runs.json']
    before={str(p):sha(p) for p in sources}
    old=readcsv(sources[0]); pairs=readcsv(sources[1]); finalold=readcsv(sources[2])
    outcomes=json.loads((ROOT/'data/results/accepted_runs.json').read_text(encoding='utf-8-sig'))
    assert len(outcomes)==810 and all(r['scientific_success_received'] and not r['smoke'] for r in outcomes)
    assert len({r['source_runtime_signature'] for r in outcomes})==1
    index={(r['job']['scene'],r['job']['goal'],r['job']['replicate'],r['job']['method'],r['job']['arm']):r for r in outcomes}
    groups=defaultdict(list)
    for p in pairs:groups[(p['family'],p['endpoint_seconds'],p['goal'],p['target'])].append(p)
    rng=np.random.default_rng(35120261026); published=[]; changed=[]
    max_mean=max_median=max_ci=0.
    for original in old:
        key=tuple(original[k] for k in ['family','endpoint_seconds','goal','target'])
        pp=groups[key]; gains=[]; diffs=[]; strata=defaultdict(list); roundoff=0
        for p in pp:
            if p['paired_available']!='True':continue
            s,b=float(p['reference_cost']),float(p['target_cost']);gain=100*(b-s)/b
            assert abs(gain-float(p['gain_percent']))<1e-12
            d,tol=classify(s,b);roundoff+=int(d==0 and b-s!=0)
            gains.append(gain);diffs.append(d);strata[p['scene']].append(gain)
        boot=np.concatenate([rng.choice(strata[s],size=(5000,len(strata[s])),replace=True) for s in ['S1','S2','S3','S4','S5','S6']],axis=1).mean(axis=1)
        lo,hi=np.quantile(boot,[.025,.975]);mean=float(np.mean(gains));median=float(np.median(gains))
        max_mean=max(max_mean,abs(mean-float(original['mean_gain_percent'])))
        max_median=max(max_median,abs(median-float(original['median_gain_percent'])))
        max_ci=max(max_ci,abs(lo-float(original['bootstrap95_low'])),abs(hi-float(original['bootstrap95_high'])))
        r={**original,'endpoint_seconds':int(original['endpoint_seconds']),'available_pairs':len(gains),'mean_gain_percent':mean,'median_gain_percent':median,'bootstrap95_low':float(lo),'bootstrap95_high':float(hi),'wins':sum(d>0 for d in diffs),'losses':sum(d<0 for d in diffs),'ties':sum(d==0 for d in diffs),'wilcoxon_two_sided_p':pval(diffs),'holm_p':None,'sign_convention':'positive: ST-DFSO performs better on the measured cost','objective_symbol':LABEL[original['goal']],'target_label':TARGET_LABEL[original['target']],'tolerance_rule':RULE,'roundoff_pairs_classified_tie':roundoff,'significant_holm_05':False}
        published.append(r)
    for family in ['main','ablation']:
        for budget in [10,30,60,120]:
            rows=[r for r in published if r['family']==family and r['endpoint_seconds']==budget]
            assert len(rows)==(15 if family=='main' else 25);holm(rows)
    for a,b in zip(old,published):
        if any(float(a[k])!=b[k] for k in ['wins','losses','ties','wilcoxon_two_sided_p','holm_p']):
            changed.append({'family':b['family'],'budget':b['endpoint_seconds'],'goal':b['goal'],'target':b['target'],'old_wlt':[int(a[k]) for k in ['wins','losses','ties']],'new_wlt':[b[k] for k in ['wins','losses','ties']],'old_holm_p':float(a['holm_p']),'new_holm_p':b['holm_p']})
    assert max(max_mean,max_median,max_ci)<1e-11
    assert all((float(a['holm_p'])<.05)==b['significant_holm_05'] for a,b in zip(old,published))
    writecsv('CPU_COMPARISONS160_PUBLICATION.csv',published)
    writecsv('MAIN60_PUBLICATION15.csv',[r for r in published if r['family']=='main' and r['endpoint_seconds']==60])
    finals=[]; max_final=0.
    for original in finalold:
        trained,measured,target=[original[k] for k in ['training_goal','evaluated_goal','target']]
        gains=[];diffs=[];roundoff=0
        for scene in ['S1','S2','S3','S4','S5','S6']:
            for rep in [1,2,3]:
                s=index[(scene,trained,rep,'FSA_SELECTED','OFF_SF')]['final']['five_goal_values'][measured]
                b=index[(scene,trained,rep,target,'OFF_SF')]['final']['five_goal_values'][measured]
                gains.append(100*(b-s)/b);d,_=classify(s,b);diffs.append(d);roundoff+=int(d==0 and b!=s)
        mean=float(np.mean(gains));median=float(np.median(gains))
        max_final=max(max_final,abs(mean-float(original['mean_gain_percent'])),abs(median-float(original['median_gain_percent'])))
        finals.append({**original,'mean_gain_percent':mean,'median_gain_percent':median,'wins':sum(d>0 for d in diffs),'losses':sum(d<0 for d in diffs),'ties':sum(d==0 for d in diffs),'training_symbol':LABEL[trained],'evaluated_symbol':LABEL[measured],'target_label':TARGET_LABEL[target],'tolerance_rule':RULE,'roundoff_pairs_classified_tie':roundoff})
    assert max_final<1e-11
    writecsv('FINAL_PAIRED75_PUBLICATION.csv',finals)
    writecsv('FINAL_SELECTED_PUBLICATION15.csv',[r for r in finals if r['training_goal']==r['evaluated_goal']])
    decomposition=[]
    for goal in GOALS[2:]:
        for rep in [1,2,3]:
            s=index[('S6',goal,rep,'FSA_SELECTED','OFF_SF')];b=index[('S6',goal,rep,'WINDOW_ILS','OFF_SF')]
            row={'scene':'S6','training_goal':goal,'training_symbol':LABEL[goal],'replicate':rep,'st_job':s['job']['id'],'ils_job':b['job']['id']}
            for symbol,key in [('L',None),('D','SAFETY_DENSITY'),('R','SAFETY_EXPOSURE')]:
                sv=s['final']['five_goal_values'][key] if key else s['final']['distance_n_mile']
                bv=b['final']['five_goal_values'][key] if key else b['final']['distance_n_mile']
                row.update({symbol+'_ST':sv,symbol+'_ILS':bv,symbol+'_change_percent':100*(sv-bv)/bv,symbol+'_gain_percent':100*(bv-sv)/bv})
            row['multiplicative_identity_residual']=row['R_ST']/row['R_ILS']-row['L_ST']/row['L_ILS']*row['D_ST']/row['D_ILS']
            sv=s['final']['five_goal_values'][goal];bv=b['final']['five_goal_values'][goal]
            row['selected_cost_ST']=sv;row['selected_cost_ILS']=bv
            row['selected_gain_percent']=100*(bv-sv)/bv
            row['scope']='Final routes; L in nautical miles; change positive=ST greater, gain positive=ST better'
            decomposition.append(row)
    writecsv('S6_DECOMPOSITION9.csv',decomposition)
    initial=[]
    for trained in GOALS:
        for measured in GOALS:
            gains=[]
            for scene in ['S1','S2','S3','S4','S5','S6']:
                for rep in [1,2,3]:
                    r=index[(scene,trained,rep,'FSA_SELECTED','OFF_SF')]
                    sv=r['final']['five_goal_values'][measured];bv=r['seed']['five_goal_values'][measured]
                    gains.append(100*(bv-sv)/bv)
            initial.append(dict(training_goal=trained,evaluated_goal=measured,n=18,
                mean_gain_vs_initial_percent=float(np.mean(gains)),median_gain_vs_initial_percent=float(np.median(gains)),
                minimum_gain_percent=min(gains),maximum_gain_percent=max(gains),
                scope='Descriptive final ST-DFSO routes relative to each run initial route; not CPU endpoint and not comparator-relative.'))
    max_initial=0.
    for r in initial:
        vals=[]
        for scene in ['S1','S2','S3','S4','S5','S6']:
            for rep in [1,2,3]:
                x=index[(scene,r['training_goal'],rep,'FSA_SELECTED','OFF_SF')];k=r['evaluated_goal']
                sv=x['final']['five_goal_values'][k];bv=x['seed']['five_goal_values'][k];vals.append(100*(bv-sv)/bv)
        max_initial=max(max_initial,abs(np.mean(vals)-float(r['mean_gain_vs_initial_percent'])),abs(np.median(vals)-float(r['median_gain_vs_initial_percent'])))
    assert max_initial<1e-11
    writecsv('STDFSO_FINAL_VS_INITIAL25_PUBLICATION.csv',initial)
    after={str(p):sha(p) for p in sources};assert before==after
    qa={'source_hashes':before,'source_runtime_signature':outcomes[0]['source_runtime_signature'],'scipy_version':scipy.__version__,'tolerance_rule':RULE,'bootstrap_draws':5000,'bootstrap_rng_seed':35120261026,'maximum_abs_mean_difference':max_mean,'maximum_abs_median_difference':max_median,'maximum_abs_ci_difference':max_ci,'maximum_abs_final_summary_difference':max_final,'maximum_abs_initial_summary_difference':max_initial,'holm_significance_changes':0,'input_files_unchanged':True,'row_counts':{'cpu':160,'main60':15,'final':75,'final_selected':15,'s6':9,'initial':25}}
    (HERE/'qa/statistics_recalculation.json').write_text(json.dumps(qa,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in qa.items() if k not in ['source_hashes','changed_inference_rows']},ensure_ascii=True,indent=2))

if __name__=='__main__':
    ap=argparse.ArgumentParser(description='Reproduce manuscript statistics from all 810 recorded runs.')
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();HERE=args.output.resolve()
    HERE.mkdir(parents=True,exist_ok=False);(HERE/'data').mkdir();(HERE/'qa').mkdir()
    with tempfile.TemporaryDirectory(prefix='stdfso_statistics_') as tmp:
        INPUT=Path(tmp)/'tables'
        subprocess.run([sys.executable,str(ROOT/'analysis/complete_set_analysis.py'),'--output',str(INPUT)],check=True)
        main()

