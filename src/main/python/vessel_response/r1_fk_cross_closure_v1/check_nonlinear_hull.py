# -*- coding: utf-8 -*-
"""Existing linear inverse + fixed source nonlinear terms; no new coefficient fit."""
from pathlib import Path
import json,time,datetime,math
import numpy as np
from run_prediction import P,O,m,h,g,a,context,geometry_waterline_force,stats
from geometry_integrals import save

def forces(x,U,coeff):
    u,v,psi,r=x;vp=(v+.683*r)/U;rp=r*m.L/U
    scale=.5*h.RHO*U*U*m.L*m.L
    X=scale*(coeff['X']['vv']*vp*vp+coeff['X']['rr']*rp*rp+coeff['X']['vr']*vp*rp)
    Y=scale*(coeff['Y']['v_abs_v']*vp*abs(vp)+coeff['Y']['r_abs_r']*rp*abs(rp)+coeff['Y']['v2r']*vp*vp*rp+coeff['Y']['vr2']*vp*rp*rp)
    N=scale*m.L*(coeff['N']['v_abs_v']*vp*abs(vp)+coeff['N']['r_abs_r']*rp*abs(rp)+coeff['N']['v2r']*vp*vp*rp+coeff['N']['vr2']*vp*rp*rp)+.683*Y
    f=np.array([X,Y,N]);return f-f.mean(axis=1,keepdims=True)

def predict(c,seed,folder):
    start=time.perf_counter();cfg=h.read(P/'nonlinear_hull_protocol.json');n=c['n'];sel=c['sel'];mask=c['mask'];H=c['H'];mu=c['case'];drop=c['drop']
    zgeo=dict(np.load(P/'quadrature_geometry_s2.npz'));C=np.load(P/f'cross_gauss_s2_mu{mu}.npz')['C']
    phi,z,bins,signed=m.frozen_trace(c['weights'],c['U'],mu,seed);baseforces=np.empty((3,n));rollforces=np.empty((3,n))
    for j in range(3):
        ff=np.zeros(n,complex);np.add.at(ff,bins[sel],z[sel]*np.where(signed[sel]>=0,c['F'][:,j].conj(),c['F'][:,j]));baseforces[j]=(np.fft.ifft(ff)*n).real
        ff=np.zeros(n,complex);np.add.at(ff,bins[sel],z[sel]*C[:,j].conj());rollforces[j]=phi*(np.fft.ifft(ff)*n).real
    wlforces=geometry_waterline_force(zgeo,c['w'],z,bins,sel,mu,n);external=baseforces+rollforces+wlforces
    traces=np.vstack([external,phi]);traces-=traces.mean(axis=1,keepdims=True);ft=np.fft.rfft(traces,axis=1).T
    out=np.zeros((len(c['nuall']),4),complex);out[mask]=np.einsum('nki,ni->nk',H,ft[mask]);initial=np.fft.irfft(out.T,n,axis=1);current=initial.copy();history=[]
    conv=False
    for it in range(60):
        if time.perf_counter()-start>1100 or datetime.datetime.now(datetime.timezone.utc)>=datetime.datetime(2026,10,20,5,15,tzinfo=datetime.timezone.utc):break
        f=forces(current,c['U'],cfg);nf=np.fft.rfft(f,axis=1).T;new=np.zeros_like(out);new[mask]=np.einsum('nki,ni->nk',H[:,:,:3],nf[mask]);target=initial+np.fft.irfft(new.T,n,axis=1)
        residual=np.sqrt(np.mean((target-current)**2,axis=1))/np.maximum(np.sqrt(np.mean(current**2,axis=1)),[1e-5,1e-5,1e-6,1e-6])
        if not np.isfinite(target).all() or abs(target[:2]).max()>100 or abs(target[2:]).max()>10:
            history.append(dict(iteration=it,failed='Nonlinear fixed point left finite-domain diagnostic guard'));break
        history.append(dict(iteration=it,relative_RMS_fixed_point_residual=residual.tolist()))
        current=.75*current+.25*target
        if residual.max()<1e-3:conv=True;break
    obs=h.read(m.P/'auxiliary_observations.json')['targets'][str(mu)];s=stats(current[:,drop:]);errors={k:100*(s[k]/obs[k]-1) for k in ['surge_SD_m_s','sway_SD_m_s','yaw_SD_deg','yaw_mean_crossing_period_s']}
    row=dict(case=mu,seed=seed,converged=conv,iterations=len(history),history=history,geometric_only=stats(initial[:,drop:]),with_sourced_nonlinear_hull=s,
        error_percent=errors,wall_seconds=time.perf_counter()-start,max_vprime=float(abs((current[1]+.683*current[3])/c['U']).max()),
        max_rprime=float(abs(current[3]*m.L/c['U']).max()),no_parameter_fitting=True,physical_validation_passed=False)
    q=folder/f'case{mu}_seed{seed}.json'
    with q.open('x',encoding='utf-8') as f:json.dump(row,f,ensure_ascii=False,indent=2,allow_nan=False)
    np.savez_compressed(folder/f'response_case{mu}_seed{seed}.npz',response=current,dt=.05,discard=drop)
    print('NL hull',mu,'converged',conv,'iterations',len(history),'seconds',round(time.perf_counter()-start,2),s,flush=True)
    return row

def main():
    start=time.perf_counter();folder=P/'nonlinear_hull_pilot';folder.mkdir();rows=[]
    assert h.read(P/'pilot_summary.json')['numerical_response_mesh_gate_passed']
    for case in [45,90]:rows.append(predict(context(case),20261018,folder))
    for r in h.read(P/'backup_manifest.json'):assert h.sha(r['path'])==r['sha256']
    save('nonlinear_hull_pilot_summary.json',dict(results=rows,all_converged=all(r['converged'] for r in rows),new_free_parameters=0,
        no_new_model_states_or_BEM=True,wall_seconds=time.perf_counter()-start,physical_validation_passed=False))

