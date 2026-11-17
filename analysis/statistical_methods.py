# Statistical functions used by the paper; call explicitly on supplied result rows.

from pathlib import Path

import argparse,csv,datetime,hashlib,json,math,os,warnings

import numpy as np

import scipy

from scipy.stats import wilcoxon

def read(p):return json.loads(Path(p).read_text('utf-8-sig'))

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def save(p,z):
    with Path(p).open('x',encoding='utf-8',newline='\n') as f:json.dump(z,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())

def csvout(p,rows):
    assert rows
    with Path(p).open('x',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows);f.flush();os.fsync(f.fileno())

def endpoint(row,cap):
    if not row['scientific_success_received']:return None
    e=row['observations']['endpoints'][str(cap)];return e['cost'] if e['available'] else None

def estimate(diffs,gains,strata,rng):
    n=len(gains);ci=None
    if all(strata):
        boot=np.concatenate([rng.choice(x,size=(5000,len(x)),replace=True) for x in strata],axis=1).mean(axis=1)
        ci=[float(x) for x in np.quantile(boot,[.025,.975])]
    if not n:p=None
    elif not any(d!=0 for d in diffs):p=1.
    else:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore',UserWarning)
            p=float(wilcoxon(diffs,zero_method='wilcox',correction=False,alternative='two-sided',mode='approx').pvalue)
        assert math.isfinite(p)
    return dict(available_pairs=n,missing_pairs=18-n,mean_gain_percent=float(np.mean(gains)) if n else None,median_gain_percent=float(np.median(gains)) if n else None,bootstrap95_low=ci[0] if ci else None,bootstrap95_high=ci[1] if ci else None,wins=sum(x>0 for x in gains),losses=sum(x<0 for x in gains),ties=sum(x==0 for x in gains),wilcoxon_two_sided_p=p)

def holm(rows,count):
    assert len(rows)==count
    order=sorted(range(count),key=lambda i:rows[i]['wilcoxon_two_sided_p'] if rows[i]['wilcoxon_two_sided_p'] is not None else 1.)
    running=0.
    for k,i in enumerate(order):
        p=rows[i]['wilcoxon_two_sided_p'];running=max(running,min(1.,(count-k)*(p if p is not None else 1.)))
        rows[i]['holm_p']=running if p is not None else None
        rows[i]['holm_family_size']=count

def contrast_analysis(rows,protocol,cohort):
    ix={(r['job']['scene'],r['job']['goal'],r['job']['replicate'],r['job']['method'],r['job']['arm']):r for r in rows}
    assert len(ix)==810
    comparisons=[];pairs=[];rng=np.random.default_rng(35120261026)
    for kind,targets,method,famsize in [('main',['WINDOW_ILS','ACO','ASV_IAFSA'],None,15),('ablation',['MINUS_LL','MINUS_AV','MINUS_FG','MINUS_ER','CORE'],'FSA_SELECTED',25)]:
        for cap in [60,10,30,120]:
            family=[]
            for goal in protocol['goals']:
                for target in targets:
                    diffs=[];gains=[];strata=[];local=[]
                    for scene in protocol['scenes']:
                        block=[]
                        for rep in [1,2,3]:
                            f=ix[(scene,goal,rep,'FSA_SELECTED','OFF_SF')]
                            x=ix[(scene,goal,rep,target,'OFF_SF')] if kind=='main' else ix[(scene,goal,rep,method,target)]
                            assert f['job']['seed']==x['job']['seed'],'PAIRED_SEED_CHANGED'
                            a,b=endpoint(f,cap),endpoint(x,cap);available=a is not None and b is not None
                            if available:assert b>0 and a>0
                            gain=100*(b-a)/b if available else None
                            row=dict(cohort=cohort,family=kind,endpoint_seconds=cap,goal=goal,target=target,scene=scene,replicate=rep,paired_seed=f['job']['seed'],planned_pair=True,reference_available=a is not None,target_available=b is not None,paired_available=available,reference_cost=a,target_cost=b,gain_percent=gain,reference_job=f['job']['id'],target_job=x['job']['id'])
                            local.append(row);pairs.append(row)
                            if available:diffs.append(b-a);gains.append(gain);block.append(gain)
                        strata.append(block)
                    z=dict(cohort=cohort,family=kind,endpoint_seconds=cap,primary=cap==60,goal=goal,target=target,planned_pairs=18,reference_available=sum(x['reference_available'] for x in local),target_available=sum(x['target_available'] for x in local),**estimate(diffs,gains,strata,rng),holm_p=None,holm_family_size=famsize,sign_convention='positive: ST-DFSO OFF_SF lower cost',bootstrap=5000,six_scenario_stratification=True)
                    family.append(z)
            holm(family,famsize);comparisons+=family
    assert len(pairs)==(15+25)*18*4 and len(comparisons)==160
    return comparisons,pairs

def cross_goals(rows,protocol,cohort):
    ix={(r['job']['scene'],r['job']['goal'],r['job']['replicate'],r['job']['method'],r['job']['arm']):r for r in rows}
    details=[];summary=[]
    for trained in protocol['goals']:
        for measured in protocol['goals']:
            for method in ['FSA_SELECTED','ACO','ASV_IAFSA','WINDOW_ILS']:
                values=[]
                for scene in protocol['scenes']:
                    for rep in [1,2,3]:
                        row=ix[(scene,trained,rep,method,'OFF_SF')];ok=row['scientific_success_received'];v=row['final']['five_goal_values'][measured] if ok else None
                        details.append(dict(cohort=cohort,training_goal=trained,evaluated_goal=measured,method=method,scene=scene,replicate=rep,job=row['job']['id'],available=ok,value=v,scope='final stored same-route five-goal audit; not CPU60 endpoint'))
                        if ok:values.append(v)
                summary.append(dict(cohort=cohort,training_goal=trained,evaluated_goal=measured,method=method,planned=18,available=len(values),missing=18-len(values),mean=float(np.mean(values)) if values else None,median=float(np.median(values)) if values else None,minimum=min(values) if values else None,maximum=max(values) if values else None,scope='descriptive complete final routes; no new significance family'))
    comparisons=[]
    for trained in protocol['goals']:
        for measured in protocol['goals']:
            for method in ['ACO','ASV_IAFSA','WINDOW_ILS']:
                gains=[]
                for scene in protocol['scenes']:
                    for rep in [1,2,3]:
                        a=ix[(scene,trained,rep,'FSA_SELECTED','OFF_SF')];b=ix[(scene,trained,rep,method,'OFF_SF')]
                        if a['scientific_success_received'] and b['scientific_success_received']:
                            av=a['final']['five_goal_values'][measured];bv=b['final']['five_goal_values'][measured];assert bv>0;gains.append(100*(bv-av)/bv)
                comparisons.append(dict(cohort=cohort,training_goal=trained,evaluated_goal=measured,target=method,planned_pairs=18,available_pairs=len(gains),missing_pairs=18-len(gains),mean_gain_percent=float(np.mean(gains)) if gains else None,median_gain_percent=float(np.median(gains)) if gains else None,wins=sum(v>0 for v in gains),losses=sum(v<0 for v in gains),ties=sum(v==0 for v in gains),scope='descriptive same-route final endpoint; not confirmatory CPU60'))
    return details,summary,comparisons
