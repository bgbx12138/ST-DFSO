# -*- coding: utf-8 -*-
"""Finite frozen-parameter port/energy audit, no new hydrodynamic solve or fit."""
from pathlib import Path
import os,sys,json,time,hashlib,shutil,math
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='2'
import numpy as np
P=Path(__file__).resolve().parent;O=P.parent
sys.path.insert(0,str(O/'auxiliary_input_control_closure_v1'))
import check_control_ports as controls
audit=controls.audit;m=audit.m;h=audit.h

def sha(q):return hashlib.sha256(q.read_bytes()).hexdigest()
def save(n,x):
    with (P/n).open('x',encoding='utf-8') as f:json.dump(x,f,ensure_ascii=False,indent=2,allow_nan=False)

def components(case,params,controller):
    profile='s'+str(case);U=m.C['profiles'][profile];w=m.roll.W
    weights=m.roll.DATA['case'+str({0:215,45:205,90:206,180:218}[case])];sel=np.flatnonzero(weights>0)
    hull,V=m.wavefields(profile,case,w[sel]);hull=hull[:,m.Q].copy();foil=np.zeros_like(hull)
    foil_rows=[]
    for j,f in enumerate(m.FOILS):
        pos=np.array(f['r']);normal=np.array(f['n']);g=np.r_[normal[:2],np.cross(pos,normal)[[0,2]]]
        lift=.5*h.RHO*U*f['area']*f['lift_slope_per_rad']
        component=lift*(V[:,j]@normal)[:,None]*g[m.Q]
        foil+=component;foil_rows.append(dict(name=f['name'],force_vector_per_velocity=(lift*g[m.Q]).tolist()))
    external=m.wave_external(profile,case,U,w[sel]);difference=float(np.linalg.norm(hull+foil-external)/max(np.linalg.norm(external),1e-300))
    assert difference<1e-12
    n=round(8192/.05);drop=round(512/.05);nuall=2*np.pi*np.fft.rfftfreq(n,d=.05);mask=(nuall>=.0001)&(nuall<=12);nu=nuall[mask]
    B=m.damping(U,params);T,cond=controls.explicit_operator(profile,U,nu,B,controller);T=T.conj()
    H=np.stack([1j*nu[:,None]*T[:,0],1j*nu[:,None]*T[:,1]-U*T[:,2],T[:,2],1j*nu[:,None]*T[:,2]],axis=1)
    seed_rows=[];sum_band=np.zeros((4,3));sum_cov=np.zeros((4,3,3));sum_psd=np.zeros((4,len(nuall)));closure_error=0.
    for seed in m.C['development']['seeds']:
        phi,z,bins,signednu=m.frozen_trace(weights,U,case,seed)
        forcing=[]
        for F in [hull,foil]:
            f=np.empty((3,n))
            for j in range(3):
                ff=np.zeros(n,complex);mult=np.where(signednu[sel]>=0,np.conj(F[:,j]),F[:,j])
                np.add.at(ff,bins[sel],z[sel]*mult);f[j]=(np.fft.ifft(ff)*n).real
            forcing.append(f)
        output=[]
        for j in range(3):
            traces=np.zeros((4,n));
            if j<2:traces[:3]=forcing[j]
            else:traces[3]=phi
            traces-=traces.mean(axis=1,keepdims=True);ft=np.fft.rfft(traces,axis=1).T
            out=np.zeros((len(nuall),4),complex);out[mask]=np.einsum('nki,ni->nk',H,ft[mask])
            output.append(np.fft.irfft(out.T,n,axis=1))
        output=np.array(output);total=output.sum(axis=0);window=total[:,drop:]
        stack=output[:,:,drop:]-output[:,:,drop:].mean(axis=2,keepdims=True)
        cov=np.einsum('ikn,jkn->kij',stack,stack)/stack.shape[-1];sum_cov+=cov
        sd=window.std(axis=1);mean=window.mean(axis=1);rms=np.sqrt(np.mean(window**2,axis=1));period=audit.crossing_period(window[2],.05)
        ft=np.fft.rfft(total-total.mean(axis=1,keepdims=True),axis=1)/n;psd=abs(ft)**2;psd[:,1:-1]*=2;sum_psd+=psd
        bands=[(nuall>0)&(nuall<.2),(nuall>=.2)&(nuall<.4),(nuall>=.4)]
        for j,b in enumerate(bands):sum_band[:,j]+=psd[:,b].sum(axis=1)
        closure_error=max(closure_error,float(np.max(abs(cov.sum(axis=(1,2))-sd**2))))
        seed_rows.append(dict(seed=seed,surge_SD_m_s=float(sd[0]),sway_SD_m_s=float(sd[1]),yaw_SD_deg=float(sd[2]*180/np.pi),
            yaw_rate_RMS_deg_s=float(rms[3]*180/np.pi),yaw_mean_crossing_period_s=period))
    count=len(seed_rows);sum_psd/=count;sum_cov/=count;sum_band/=count
    mean={k:float(np.sqrt(np.mean([r[k]**2 for r in seed_rows]))) for k in ['surge_SD_m_s','sway_SD_m_s','yaw_SD_deg','yaw_rate_RMS_deg_s']}
    mean['yaw_mean_crossing_period_s']=float(np.mean([r['yaw_mean_crossing_period_s'] for r in seed_rows if r['yaw_mean_crossing_period_s'] is not None]))
    obs=h.read(m.P/'auxiliary_observations.json')['targets'][str(case)]
    errors={k:100*(mean[k]/obs[k]-1) for k in ['surge_SD_m_s','sway_SD_m_s','yaw_SD_deg','yaw_mean_crossing_period_s']}
    bands=sum_band/np.maximum(sum_band.sum(axis=1,keepdims=True),1e-300)
    # Shape-limited source comparison, not a proof that measured upcrossing equals spectral T02.
    spectralT=(2*np.pi*np.sqrt(sum_psd[2].sum()/np.sum(sum_psd[2]*nuall**2)))
    np.savez_compressed(P/f"spectrum_{case}_{'source' if np.all(params==0) else 'frozen'}.npz",nu=nuall,power=sum_psd,component_covariance=sum_cov)
    return dict(case=case,controller=controller,mean_prediction=mean,error_percent=errors,seed_rows=seed_rows,
        component_order=['hull_wave','foil_wave','roll_coupling'],channel_order=['u','v','psi','r'],
        component_covariance=sum_cov.tolist(),spectral_power_band_fractions=bands.tolist(),
        spectral_bands_rad_s=['(0,.2)','[.2,.4)','[.4,infinity)'],yaw_spectral_T02_s=float(spectralT),
        external_addition_identity_relative_error=difference,covariance_closure_max_error=closure_error,condition=cond,
        foil_port_geometry=foil_rows,parameters= params.tolist(),
        source_only_nominal_baseline_not_fitted=bool(np.all(params==0)),physical_response_validation_passed=False)

def main():
    start=time.perf_counter();files=[m.P/'model.py',m.P/'hydro.py',m.P/'auxiliary_parameter_seal.json',
        O/'oe_moment_calibration_v1/parameter_seal.json',O/'r1_completion_results_audit_v1/window_consistent_response.py',
        O/'auxiliary_input_control_closure_v1/check_control_ports.py',m.P/'protocol.json',m.P/'auxiliary_observations.json',
        O/'kcs_oe_input_repair_v1/new_parameter_seal.json']
    (P/'before').mkdir();manifest=[]
    for j,q in enumerate(files):
        t=P/'before'/f'{j:02}_{q.name}';shutil.copy2(q,t);assert sha(t)==sha(q);manifest.append(dict(path=str(q),backup=str(t),sha256=sha(q)))
    save('backup_manifest.json',manifest)
    save('execution_contract.json',dict(date='2026-10-20',authorization_window_Beijing='11:48-13:18',
        frozen_parameter_changes=False,new_parameter_fitting=False,new_BEM=False,
        experiments='Four source conditions, frozen auxiliary/roll baseline vs sourced nominal auxiliary damping; exactly eight full-record predictions with same8seeds.',
        source_baseline='Zero log multipliers in existing sourced damping construction; a distinct unfitted comparator, not changes to old sealed coefficients.',
        hypothesis='Resolve port ownership, separate hull-wave/local-foil/roll covariance, locate spectral deficit before considering new inputs.',
        source_control='Use published CFD/PF gains as configuration sensitivity, actual EFD gains remain unknown. Does not pretend exact EFD control matching.',
        cases=[0,45,90,180],max_wall_seconds=3000,old_files=manifest))
    params=np.asarray(h.read(m.P/'auxiliary_parameter_seal.json')['log_parameters']);results=[]
    for name,values in [('frozen',params),('source_nominal',np.zeros(3))]:
        for case in [0,45,90,180]:
            if time.perf_counter()-start>2400:raise RuntimeError('Time budget guard before launching next condition')
            row=components(case,values,controls.CONTROLLERS[case]);row['version']=name;results.append(row)
            save(f'{name}_{case}.json',row)
            print(name,case,json.dumps(row['mean_prediction']),'wall',round(time.perf_counter()-start,2),flush=True)
    for r in manifest:assert sha(Path(r['path']))==r['sha256']
    save('summary.json',dict(results=results,old_files_unchanged=manifest,wall_seconds=time.perf_counter()-start,
        full_safety_model_validated=False,no_new_fitted_parameters=True))
    print('Saved port closure audit',round(time.perf_counter()-start,2),'seconds',flush=True)

