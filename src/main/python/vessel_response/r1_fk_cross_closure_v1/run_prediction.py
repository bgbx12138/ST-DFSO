# -*- coding: utf-8 -*-
"""Bounded geometry nonlinear-FK increments on the already existing4DOF model."""
from pathlib import Path
import os,sys,time,json,math,datetime
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='2'
import numpy as np
from geometry_integrals import P,O,h,save
sys.path.insert(0,str(O/'r1_global_appendage_closure_v1'));import global_operator as g
sys.path.insert(0,str(O/'r1_port_closure_v2'));import check_closure as a
m=g.m

def geometry_waterline_force(z,w,zeta,bins,sel,mu,n):
    # Intrinsic spatial phase is NOT conjugated when signed encounter crosses0.
    # The positive-time FFT spatial amplitude is conjugate native exp(+ik.r).
    beta=math.radians(180-mu);k=w[sel]**2/9.81;coords=z['wl'];weights=z['wl_g'];out=np.zeros((3,n))
    for start in range(0,len(coords),8):
        pts=coords[start:start+8];phase=pts[:,0]*math.cos(beta)+pts[:,1]*math.sin(beta)
        amp=zeta[sel][None,:]*np.exp(-1j*phase[:,None]*k[None,:]);ft=np.zeros((len(pts),n),complex)
        np.add.at(ft,(np.arange(len(pts))[:,None],bins[sel][None,:]),amp)
        eta=(np.fft.ifft(ft,axis=1)*n).real
        out+=(-.5*h.RHO*9.81)*(weights[start:start+8].T@(eta*eta))
    if mu in [0,180]:out[1:]=0
    if mu==90:out[0]=0
    out-=out.mean(axis=1,keepdims=True)
    return out

def context(case):
    U=m.C['profiles']['s'+str(case)];profile='s'+str(case);weights=m.roll.DATA['case'+str({0:215,45:205,90:206,180:218}[case])]
    w=m.roll.W;sel=np.flatnonzero(weights>0);n=round(8192/.05);drop=round(512/.05);nuall=2*np.pi*np.fft.rfftfreq(n,d=.05);mask=(nuall>=.0001)&(nuall<=12);nu=nuall[mask]
    transfer,cond=g.operator(profile,U,nu,m.damping(U,np.zeros(3)),dict(P=-.8,D_s=0.));T=transfer.conj()
    H=np.stack([1j*nu[:,None]*T[:,0],1j*nu[:,None]*T[:,1]-U*T[:,2],T[:,2],1j*nu[:,None]*T[:,2]],axis=1)
    return dict(case=case,U=U,weights=weights,w=w,sel=sel,n=n,drop=drop,nuall=nuall,mask=mask,H=H,F=m.wave_external(profile,case,U,w[sel]),condition=cond)

def stats(x):
    sd=x.std(axis=1);r=np.sqrt(np.mean(x[3]**2));return dict(surge_SD_m_s=float(sd[0]),sway_SD_m_s=float(sd[1]),
        yaw_SD_deg=float(sd[2]*180/np.pi),yaw_rate_RMS_deg_s=float(r*180/np.pi),yaw_mean_crossing_period_s=a.audit.crossing_period(x[2],.05))

def predict(c,stride,seed):
    start=time.perf_counter();mu=c['case'];n=c['n'];drop=c['drop'];sel=c['sel'];mask=c['mask'];H=c['H']
    zgeo=dict(np.load(P/f'quadrature_geometry_s{stride}.npz'));cache=np.load(P/f'cross_gauss_s{stride}_mu{mu}.npz');assert np.array_equal(cache['w'],c['w'][sel]);C=cache['C']
    phi,z,bins,signednu=m.frozen_trace(c['weights'],c['U'],mu,seed)
    force=np.empty((3,n));fphi=np.empty((3,n))
    for j in range(3):
        ff=np.zeros(n,complex);np.add.at(ff,bins[sel],z[sel]*np.where(signednu[sel]>=0,np.conj(c['F'][:,j]),c['F'][:,j]));force[j]=(np.fft.ifft(ff)*n).real
        ff=np.zeros(n,complex);np.add.at(ff,bins[sel],z[sel]*C[:,j].conj());fphi[j]=phi*(np.fft.ifft(ff)*n).real
    fphi-=fphi.mean(axis=1,keepdims=True)
    fwl=geometry_waterline_force(zgeo,c['w'],z,bins,sel,mu,n)
    responses=[];input_retained=[]
    for j in range(3):
        traces=np.zeros((4,n))
        if j==0:traces[:3]=force;traces[3]=phi
        else:traces[:3]=fphi if j==1 else fwl
        traces-=traces.mean(axis=1,keepdims=True);ft=np.fft.rfft(traces,axis=1).T
        out=np.zeros((len(c['nuall']),4),complex);out[mask]=np.einsum('nki,ni->nk',H,ft[mask]);responses.append(np.fft.irfft(out.T,n,axis=1))
        p=abs(ft)**2;input_retained.append((p[mask].sum(axis=0)/np.maximum(p.sum(axis=0),1e-300)).tolist())
    total=sum(responses);window=total[:,drop:];base=responses[0][:,drop:]
    stats_rows=dict(baseline=stats(base),roll_wave_only_increment=stats(responses[1][:,drop:]),waterline_wave_squared_only_increment=stats(responses[2][:,drop:]),combined=stats(window))
    cc=np.array([t[:,drop:]-t[:,drop:].mean(axis=1,keepdims=True) for t in responses]);cov=np.einsum('ikn,jkn->kij',cc,cc)/cc.shape[-1]
    sd=window.std(axis=1);closure=float(abs(cov.sum(axis=(1,2))-sd**2).max());assert closure<1e-8*max(float((sd**2).max()),1.)
    fft=np.fft.rfft(total-total.mean(axis=1,keepdims=True),axis=1)/n;power=abs(fft)**2;power[:,1:-1]*=2
    bands=(power[:,(c['nuall']>0)&(c['nuall']<.2)].sum(axis=1)/np.maximum(power[:,1:].sum(axis=1),1e-300)).tolist()
    obs=h.read(m.P/'auxiliary_observations.json')['targets'][str(mu)]
    error={key:100*(stats_rows['combined'][key]/obs[key]-1) for key in ['surge_SD_m_s','sway_SD_m_s','yaw_SD_deg','yaw_mean_crossing_period_s']}
    return dict(case=mu,stride=stride,seed=seed,statistics=stats_rows,error_percent=error,
        input_retained_spectral_mass=input_retained,output_below_point2_rad_s_fraction=bands,component_covariance=cov.tolist(),variance_closure_error=closure,
        roll_SD_deg=float(phi[drop:].std()*180/np.pi),roll_max_abs_deg=float(abs(phi[drop:]).max()*180/np.pi),
        roll_fraction_abs_gt10deg=float(np.mean(abs(phi[drop:])>np.deg2rad(10))),condition=c['condition'],wall_seconds=time.perf_counter()-start,
        no_parameter_fitting=True,partial_geometric_FK_only=True,physical_validation_passed=False)

def main():
    start=time.perf_counter();gate=h.read(P/'spectral_gate_gauss.json');assert gate['all_passed']
    mode=sys.argv[1] if len(sys.argv)>1 else 'pilot';assert mode in ['pilot','full']
    if mode=='full':assert h.read(P/'pilot_summary.json')['numerical_response_mesh_gate_passed']
    seeds=[m.C['development']['seeds'][0]] if mode=='pilot' else m.C['development']['seeds'];folder=P/mode;folder.mkdir()
    save(mode+'_execution_contract.json',dict(mode=mode,cases=[0,45,90,180],seeds=seeds,pressure_meshes=[4,2] if mode=='pilot' else [2],
        no_free_parameters=True,no_new_BEM_or_geometry=True,max_wall_seconds=2400,deadline_UTC='2026-10-20T05:18:00+00:00',
        candidate='Sum of predeclared roll-wave geometric increment and waterline wave-squared increment; individual contributions are diagnostics, never selected by measured fit.',
        physical_scope='Centered fluctuations only; mean force not used to update source equilibrium speed/heading. Total nonlinear wave potential omitted.',
        phase='Same native signed encounter and raw Gaussian wave phases; exact first-order force array kept once.',
        protocol_sha256=h.sha(P/'protocol.json')))
    results=[];comparisons=[]
    for case in [0,45,90,180]:
        c=context(case);strides=[4,2] if mode=='pilot' else [2];pair={}
        for stride in strides:
            rows=[]
            for seed in seeds:
                if time.perf_counter()-start>2300 or datetime.datetime.now(datetime.timezone.utc)>=datetime.datetime(2026,10,20,5,15,tzinfo=datetime.timezone.utc):
                    raise RuntimeError('Stop before authorization deadline; completed per-seed records retained')
                r=predict(c,stride,seed);rows.append(r)
                q=folder/f'case{case}_s{stride}_seed{seed}.json'
                with q.open('x',encoding='utf-8') as f:json.dump(r,f,ensure_ascii=False,indent=2,allow_nan=False)
                print('Completed',mode,case,stride,seed,'seconds',round(r['wall_seconds'],2),r['statistics']['combined'],flush=True)
            pair[stride]=rows;results+=rows
        if mode=='pilot':
            a0=pair[4][0]['statistics']['combined'];b0=pair[2][0]['statistics']['combined'];metrics={k:abs(a0[k]-b0[k])/max(abs(b0[k]),1e-6) for k in ['surge_SD_m_s','sway_SD_m_s','yaw_SD_deg','yaw_rate_RMS_deg_s']}
            comparisons.append(dict(case=case,relative_output_SD_differences=metrics,passed=bool(max(metrics.values())<=.1)))
    for r in h.read(P/'backup_manifest.json'):assert h.sha(r['path'])==r['sha256']
    summary=dict(results=results,mesh_comparison=comparisons,numerical_response_mesh_gate_passed=all(r['passed'] for r in comparisons) if mode=='pilot' else True,
        wall_seconds=time.perf_counter()-start,physical_validation_passed=False,old_parameters_unchanged=True)
    save(mode+'_summary.json',summary);print('Saved',mode,'summary, mesh gate',summary['numerical_response_mesh_gate_passed'],'wall',time.perf_counter()-start,flush=True)

