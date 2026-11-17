# -*- coding: utf-8 -*-
"""Apply the frozen linear operator BEFORE cutting the statistics window.

Own nonlinear roll and all parameters remain unchanged. Full-record periodic
wave forcing has known bins. Cropping before the inverse falsely treats its
spectral leakage as a new low-frequency physical input. This module is an
isolated numerical correction, not a new response fit or hydrodynamic solver.
"""
from pathlib import Path
import sys,json,math,csv,hashlib,time
import numpy as np
A=Path(__file__).resolve().parent;P=A.parent/'r1_completion_v1';sys.path.insert(0,str(P))
import model as m
import hydro as h
def save(name,x):
 with (A/name).open('x',encoding='utf-8') as f:json.dump(x,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
def short_known_input():
 n=163840;dt=.05;drop=10240;N=n-drop;k=1001;period=n*dt/k
 t=np.arange(n)*dt;force=np.sin(2*np.pi*k*np.arange(n)/n+.37);a=1e-6
 def output(x):
  nu=2*np.pi*np.fft.rfftfreq(len(x),d=dt);T=1/(a+1j*nu);T[0]=0
  return np.fft.irfft(np.fft.rfft(x)*T,len(x))
 full=output(force)[drop:];cropped=output(force[drop:]-force[drop:].mean())
 exact=np.imag(np.exp(1j*(2*np.pi*k*np.arange(n)/n+.37))/(a+1j*2*np.pi*k/(n*dt)))[drop:]
 return dict(period_s=period,full_record_then_cut_max_error=float(np.max(abs(full-exact))),
  cropped_first_SD=float(cropped.std()),analytic_SD=float(exact.std()),full_record_then_cut_SD=float(full.std()),
  crop_first_relative_SD_error_percent=float(100*(cropped.std()/exact.std()-1)),physical_model_parameters_fitted=False)
def crossing_period(x,dt):
 x=x-x.mean();ix=np.flatnonzero((x[:-1]<=0)&(x[1:]>0))
 if len(ix)<3:return None
 tc=(ix-x[ix]/(x[ix+1]-x[ix]))*dt;return float(np.diff(tc).mean())
def source(case,params):
 U=m.C['profiles']['s'+str(case)];weights=m.roll.DATA['case'+str({0:215,45:205,90:206,180:218}[case])]
 w=m.roll.W;sel=np.flatnonzero(weights>0);F=m.wave_external('s'+str(case),case,U,w[sel]);n=round(8192/.05);drop=round(512/.05)
 nuall=2*np.pi*np.fft.rfftfreq(n,d=.05);mask=(nuall>=.0001)&(nuall<=12);nu=nuall[mask]
 transfer,cond=m.operator('s'+str(case),U,nu,m.damping(U,params));T=transfer.conj()
 H=np.stack([1j*nu[:,None]*T[:,0],1j*nu[:,None]*T[:,1]-U*T[:,2],T[:,2],1j*nu[:,None]*T[:,2]],axis=1)
 rows=[];wave_leakage=[]
 for seed in m.C['development']['seeds']:
  phi,z,bins,signednu=m.frozen_trace(weights,U,case,seed);forces=np.empty((3,n))
  for j in range(3):
   ff=np.zeros(n,complex);np.add.at(ff,bins[sel],z[sel]*np.where(signednu[sel]>=0,np.conj(F[:,j]),F[:,j]));forces[j]=(np.fft.ifft(ff)*n).real
  traces=np.vstack([forces,phi[None,:]]);traces-=traces.mean(axis=1,keepdims=True)
  spectrum=np.fft.rfft(traces,axis=1).T;out=np.zeros((len(nuall),4),complex)
  out[mask]=np.einsum('nki,ni->nk',H,spectrum[mask]);response=np.fft.irfft(out.T,n,axis=1)[:,drop:]
  sd=response.std(axis=1);rRMS=float(np.sqrt(np.mean(response[3]**2))*180/np.pi)
  rows.append(dict(seed=seed,surge_SD_m_s=float(sd[0]),sway_SD_m_s=float(sd[1]),yaw_SD_deg=float(sd[2]*180/np.pi),
   yaw_rate_RMS_deg_s=rRMS,yaw_mean_crossing_period_s=crossing_period(response[2],.05)))
  full_power=abs(spectrum[:,0])**2;cp=np.fft.rfft(forces[0,drop:]-forces[0,drop:].mean());cpnu=2*np.pi*np.fft.rfftfreq(n-drop,d=.05)
  wave_leakage.append(dict(seed=seed,full_input_low_frequency_fraction=float(full_power[(nuall>0)&(nuall<.025)].sum()/full_power[nuall>0].sum()),
   cropped_input_low_frequency_fraction=float((abs(cp[(cpnu>0)&(cpnu<.025)])**2).sum()/(abs(cp[cpnu>0])**2).sum())))
 fields=['surge_SD_m_s','sway_SD_m_s','yaw_SD_deg','yaw_rate_RMS_deg_s'];mean={k:float(np.sqrt(np.mean([x[k]**2 for x in rows]))) for k in fields}
 mean['yaw_mean_crossing_period_s']=float(np.mean([r['yaw_mean_crossing_period_s'] for r in rows if r['yaw_mean_crossing_period_s'] is not None]))
 return dict(case=case,rows=rows,mean_power_prediction=mean,surge_forcing_low_frequency=wave_leakage,condition=cond,
  numerical_correction_only=True,no_refit=True,auxiliary_physical_validation_passed=False)
def run():
 start=time.perf_counter();old=h.sha(P/'auxiliary_parameter_seal.json');rh=h.sha(P.parent/'oe_moment_calibration_v1/parameter_seal.json');seal=h.read(P/'auxiliary_parameter_seal.json')
 known=short_known_input();assert known['full_record_then_cut_max_error']<1e-9
 save('known_input_window_check.json',known);cases={str(c):source(c,np.array(seal['log_parameters'])) for c in [0,45,90,180]}
 obs=h.read(P/'auxiliary_observations.json')['targets'];scores=[]
 for c,v in cases.items():
  for key in ['surge_SD_m_s','sway_SD_m_s','yaw_SD_deg']:
   y=obs[c][key];f=v['mean_power_prediction'][key];scores.append(dict(case=int(c),field=key,observed=y,predicted=f,error_percent=100*(f/y-1)))
 assert h.sha(P/'auxiliary_parameter_seal.json')==old and h.sha(P.parent/'oe_moment_calibration_v1/parameter_seal.json')==rh
 save('window_corrected_source_check.json',dict(cases=cases,scores=scores,auxiliary_seal_sha256_unchanged=old,roll_seal_sha256_unchanged=rh,
  physical_validation_passed=False,old_covariance_or_route_results_not_overwritten=True,wall_seconds=time.perf_counter()-start,
  remaining='Correction does not add missing nonlinear/control excitation or repair source control mismatch; no automatic new fit or full route rerun.'))
 print(json.dumps(dict(known_input=known,scores=scores),ensure_ascii=False,indent=2))
