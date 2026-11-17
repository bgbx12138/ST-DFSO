# -*- coding: utf-8 -*-
"""Matched small-response replay and evidence admission, not parameter fitting."""
from pathlib import Path
import os,sys,json,hashlib,time,math
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='2'
import numpy as np
P=Path(__file__).resolve().parent;O=P.parent
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
A=read(O/'active_aggregate_fit_v1/parameter_seal.json')['active_anchor']
D=read(O/'wave_group_rate_repair_v1/parameter_seal.json')['D_kg_s']
TAU=read(O/'control_coupling_repair_v1/controller_port_check.json')['tau_s']
CASES={c['test']:c for c in read(O/'matched_wave_evidence_v1/matched_cases.json')['cases']}
SRC=read(P/'source_statistics.json')['rows']
def save(p,x):
    assert not p.exists(),p;p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
def verify():
    for r in read(P/'backup_manifest.json'):assert sha(r['path'])==r['sha256'],r['path']
    if (P/'execution_contract.json').exists():
        for q,h in read(P/'execution_contract.json')['hashes'].items():assert sha(q)==h,q
def prepare():
    verify();fs=[Path(__file__),P/'plan.md',P/'source_statistics.json',P/'backup_manifest.json']
    save(P/'execution_contract.json',dict(date='2026-10-19',hashes={str(q):sha(q) for q in fs},
        cases=[215,205,206,218],seeds=list(range(20261018,20261026)),duration_s=8192.,discard_s=512.,
        primary_dt=.05,fine_dt=.025,short_budget_s=180.,no_BEM=True,new_parameters=0,
        primary_input='same band-limited original JONSWAP with source wave SD; actual mean speed; same signed encounter bins',
        heading_inputs={'215':'frozen D/tau group only','205':'frozen fine LE plus D/tau group',
            '206':'original C*k beam-wave slope port; not a new full LE beam calculation','218':'frozen D/tau group only'},
        alternatives_not_reselected=['source_window','C_reference','linear diagnostic','old fitted response gains'],
        EFD_uncertainty_use='reported compatibility diagnostic only; no posthoc promotion or new tolerance',
        mean_angle_modelled=False,exact_controller_gains_known=False,physical_validation_passed=False))
    print('Matched replay contract sealed, 36 short records, no new fit.',flush=True)

def rk4(force,dt):
    n=len(force);x=np.empty(n);v=np.empty(n);x[0]=v[0]=0.;xx=vv=0.
    I,C,alpha,beta=A['I'],A['C'],A['alpha'],A['beta'];inv=1/I
    for j in range(n-1):
        f0=float(force[j]);f1=float(force[j+1]);fm=.5*(f0+f1)
        k1x=vv;k1v=(f0-C*xx-alpha*vv-beta*abs(vv)*vv)*inv
        a=xx+.5*dt*k1x;b=vv+.5*dt*k1v;k2x=b;k2v=(fm-C*a-alpha*b-beta*abs(b)*b)*inv
        a=xx+.5*dt*k2x;b=vv+.5*dt*k2v;k3x=b;k3v=(fm-C*a-alpha*b-beta*abs(b)*b)*inv
        a=xx+dt*k3x;b=vv+dt*k3v;k4x=b;k4v=(f1-C*a-alpha*b-beta*abs(b)*b)*inv
        xx+=dt/6*(k1x+2*k2x+2*k3x+k4x);vv+=dt/6*(k1v+2*k2v+2*k3v+k4v)
        x[j+1]=xx;v[j+1]=vv
    assert np.isfinite(x).all();return x,v
def stats(x,dt,discard=512.):
    a=x[round(discard/dt):];mean=float(np.mean(a));z=a-mean
    ix=np.flatnonzero((z[:-1]<=0)&(z[1:]>0));tc=(ix-z[ix]/(z[ix+1]-z[ix]))*dt
    return dict(mean=float(mean),SD=float(np.std(a)),RMS=float(np.sqrt(np.mean(a*a))),
        period=float(np.mean(np.diff(tc))) if len(tc)>1 else None,crossings=len(ix))

def one(case_id,seed,dt):
    case=CASES[case_id];t0=time.perf_counter();duration=8192.;n=round(duration/dt)
    w=np.geomspace(.005,6.,32769);cut=read(O/'wave_group_rate_repair_v1/input_band_seal.json')['omega_cut_rad_s']
    wp=2*np.pi/10.4;sg=np.where(w<=wp,.07,.09)
    spectrum=w**-5*np.exp(-1.25*(wp/w)**4)*3.3**np.exp(-.5*((w-wp)/(sg*wp))**2)*((w>=.4)&(w<=cut))
    spectrum*=case['measured_wave_SD_m']**2/np.trapz(spectrum,w)
    tw=np.r_[np.diff(w)[0]/2,(w[2:]-w[:-2])/2,np.diff(w)[-1]/2];weight=tw*spectrum
    co=0. if case_id==206 else math.cos(math.radians(case['wave_from_deg']))
    we=w+case['mean_speed_m_s']*co*w*w/9.81
    assert max(abs(we))<np.pi/dt
    bins=np.rint(we*duration/(2*np.pi)).astype(int)%n;bw=np.rint(we*duration/(2*np.pi))*2*np.pi/duration
    rng=np.random.default_rng(seed);z=(rng.normal(size=len(w))+1j*rng.normal(size=len(w)))*np.sqrt(weight)
    def signal(mult):
        f=np.zeros(n,complex);np.add.at(f,bins,z*mult);return np.fft.ifft(f)*n
    eta=signal(np.ones(len(w))).real
    u=signal(w*co);du=signal(1j*bw*w*co)
    omega=2*np.pi*np.fft.rfftfreq(n,d=dt);unit=np.real(np.conj(u)*du)
    oldF=D*np.fft.irfft(np.fft.rfft(unit)/(1+1j*omega*TAU),n)
    if case_id==205:
        rows=read(O/'leading_edge_input_v1/wave_forcing_seal.json')['rows']['2']
        grid=np.array([v['encounter_omega'] for v in rows]);g=np.array([complex(*v['nominal']['force']) for v in rows])
        ins=weight>0;assert min(we[ins])>=grid[0] and max(we[ins])<=grid[-1]
        direct=signal(np.conj(np.interp(we,grid,g.real)+1j*np.interp(we,grid,g.imag))).real
    elif case_id==206:direct=signal(A['C']*w*w/9.81).real
    else:direct=np.zeros(n)
    x,v=rk4(direct+oldF,dt);r=stats(x,dt)
    for k in ['mean','SD','RMS']:r[k+'_deg']=float(np.degrees(r.pop(k)))
    wave=stats(eta,dt)
    return dict(case=case_id,seed=seed,dt=dt,duration=duration,discard=512.,prediction=r,
        wave_SD_m=wave['SD'],wave_crossing_period_s=wave['period'],
        mean_angle_modelled=False,wall_seconds=time.perf_counter()-t0)

def run():
    verify();c=read(P/'execution_contract.json');t0=time.perf_counter();paths=[]
    tasks=[(k,s,.05) for k in c['cases'] for s in c['seeds']]+[(k,c['seeds'][0],.025) for k in c['cases']]
    for k,s,dt in tasks:
        q=P/'records'/f'case{k}_seed{s}_dt{dt}.json';paths.append(q)
        if q.exists():continue
        if time.perf_counter()-t0>c['short_budget_s']:
            print('Short budget reached. Existing records preserved. Resume with same run command.',flush=True);return
        save(q,one(k,s,dt));print('Completed',k,s,dt,'wall',round(time.perf_counter()-t0,2),flush=True)
    rows=[read(q) for q in paths];out={}
    oldag=read(O/'group_control_finished_audit_v1/completed_audit.json')['independently_aggregated']
    for k in c['cases']:
        rr=[v for v in rows if v['case']==k and v['dt']==.05];src=SRC[str(k)]
        summary={}
        for field in ['SD_deg','period','mean_deg','RMS_deg']:
            vals=np.array([r['prediction'][field] for r in rr])
            summary[field]=float(vals.mean());summary[field+'_seed_std']=float(vals.std(ddof=1))
        summary.update(SD_error_percent=100*(summary['SD_deg']/src['SD']-1),
            period_error_percent=100*(summary['period']/src['period']-1),
            mean_angle_modelled=False,source_total_RMS_deg=math.hypot(src['mean'],src['SD']),
            total_RMS_error_percent_if_zero_mean_used=100*(summary['RMS_deg']/math.hypot(src['mean'],src['SD'])-1),
            source_SD_U_SS_percent=src['SD_U_SS_percent'],source_SD_U_TS_percent=src['SD_U_TS_percent'],
            source_period_U_percent=src['period_U_percent'],
            generated_wave_SD_mean_m=float(np.mean([r['wave_SD_m'] for r in rr])),
            generated_wave_period_mean_s=float(np.mean([r['wave_crossing_period_s'] for r in rr])),
            physical_validation_passed=False)
        coarse=rr[0]['prediction'];fine=next(v['prediction'] for v in rows if v['case']==k and v['dt']==.025)
        summary['dt_SD_relative']=abs(coarse['SD_deg']/fine['SD_deg']-1)
        summary['dt_period_relative']=abs(coarse['period']/fine['period']-1)
        if k!=206:
            old=oldag[f'{k}_s2']['original']['baseline']
            summary['old_primary_SD_replay_absolute']=abs(summary['SD_deg']-old['SD_deg'])
            summary['old_primary_period_replay_absolute']=abs(summary['period']-old['period_s'])
            assert summary['old_primary_SD_replay_absolute']<1e-10 and summary['old_primary_period_replay_absolute']<1e-10
        else:
            summary['old_full_band_results_not_merged']=True
            summary['beam_port_is_original_analytic_not_full_LE']=True
        out[str(k)]=summary
    verify();save(P/'summary.json',dict(cases=out,records=36,new_fit_parameters=0,new_BEM_RHS=0,
        numerical_gate=all(max(v['dt_SD_relative'],v['dt_period_relative'])<.005 for v in out.values()),
        matched_recipe_is_new_observation=False,source_wave_moments_used_as_inputs=True,
        same_controller_gains_across_tests_proven=False,held_out_is_retrospective=True,
        physical_validation_passed=False,wall_seconds=time.perf_counter()-t0))
    print(json.dumps({k:{f:v[f] for f in ['SD_deg','period','SD_error_percent','period_error_percent']} for k,v in out.items()},indent=2),flush=True)
