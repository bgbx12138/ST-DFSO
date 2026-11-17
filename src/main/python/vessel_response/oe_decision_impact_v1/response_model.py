# -*- coding: utf-8 -*-
"""Fixed OE-moment candidate on registered wave inputs. No parameter fitting."""
from pathlib import Path
import os,ast,json,math
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
import numpy as np
P=Path(__file__).resolve().parent;O=P.parent;DURATION=8192.;DISCARD=512.
read=lambda q:json.loads(Path(q).read_text(encoding='utf-8'))
PAR=read(O/'oe_moment_calibration_v1/parameter_seal.json')
LEGACY=read(O/'final_joint_calibration_v1/parameter_seal.json')
A=read(O/'active_aggregate_fit_v1/parameter_seal.json')['active_anchor'];A['alpha']=PAR['B_wave_Nms_rad']
source=O/'small_response_unified_admission_v1/run_unified.py';tree=ast.parse(source.read_text(encoding='utf-8'))
nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='rk4'];assert len(nodes)==1
ns=dict(np=np,A=A);exec(compile(ast.Module(body=nodes,type_ignores=[]),str(source)+'[rk4 only]','exec'),ns);rk4=ns['rk4']
DATA=np.load(O/'wave_input_evidence_v1/input_weights.npz');W=DATA['w']
INPUTS=read(O/'wave_input_evidence_v1/wave_only_inputs.json')['cases']
cut=read(O/'wave_group_rate_repair_v1/input_band_seal.json')['omega_cut_rad_s'];MASK=(W>=.4)&(W<=cut)
TW=np.r_[np.diff(W)[0]/2,(W[2:]-W[:-2])/2,np.diff(W)[-1]/2]
LE=read(O/'leading_edge_input_v1/wave_forcing_seal.json')['rows']['2']
grid=np.array([r['encounter_omega'] for r in LE]);g45=np.array([complex(*r['nominal']['force']) for r in LE])
nu45=W+9.279*math.cos(math.pi/4)*W*W/9.81
G45=np.interp(nu45,grid,g45.real)+1j*np.interp(nu45,grid,g45.imag)
assert min(nu45[MASK])>=grid[0] and max(nu45[MASK])<=grid[-1]
SEEDS=list(range(20261018,20261026))
VERSION='OE_R1_two_parameter_wave_moment_v1_20261019'
def angle_weights(mu):
 if not np.isfinite(mu) or not 0<=mu<=180:raise ValueError('relative incoming wave angle outside[0,180]')
 nodes=np.array([0.,45.,90.,180.]);v=np.zeros(4)
 if mu==180:v[3]=1;return v
 j=int(np.searchsorted(nodes,mu,side='right')-1);f=(mu-nodes[j])/(nodes[j+1]-nodes[j]);v[j]=1-f;v[j+1]=f
 return v
def direct_port(mu):
 a=angle_weights(mu)
 return a[1]*G45+a[2]*1j*A['C']*W*W/9.81
def spectrum(Hm0,Tp):
 if not np.isfinite([Hm0,Tp]).all() or min(Hm0,Tp)<=0:raise ValueError('positive finite Hm0,Tp required')
 wp=2*np.pi/Tp;sg=np.where(W<=wp,.07,.09)
 raw=W**-5*np.exp(-1.25*(wp/W)**4)*3.3**np.exp(-.5*((W-wp)/(sg*wp))**2)
 retained=float(np.trapz(raw*MASK,W)/np.trapz(raw,W));raw=raw*MASK;den=np.trapz(raw,W)
 if den<=0:raise ValueError('No spectral support')
 return TW*raw*(Hm0/4)**2/den,retained
def forward(weight,U,mu,seed,dt=.05):
 if not np.isfinite(U) or U<=0 or not np.isfinite(dt) or dt<=0:raise ValueError('positive U/dt required')
 co=0. if mu==90 else math.cos(math.radians(mu));nu=W+U*co*W*W/9.81
 if max(abs(nu[weight>0]))*2>=np.pi/dt:raise ValueError('quadratic encounter support beyond Nyquist')
 n=round(DURATION/dt);bins=np.rint(nu*DURATION/(2*np.pi)).astype(int)%n
 rng=np.random.default_rng(seed);z=(rng.normal(size=len(W))+1j*rng.normal(size=len(W)))*np.sqrt(weight)
 def signal(mult):
  ff=np.zeros(n,complex);np.add.at(ff,bins,z*mult);return np.fft.ifft(ff)*n
 u=signal(W*co).real;h=np.zeros(n);h[0]=h[n//2]=1;h[1:n//2]=2
 aa=np.fft.ifft(np.fft.fft(u)*h);om=2*np.pi*np.fft.fftfreq(n,d=dt)
 unit=np.real(np.conj(aa)*np.fft.ifft(1j*om*np.fft.fft(aa)))
 wr=2*np.pi*np.fft.rfftfreq(n,d=dt)
 group=LEGACY['D_effective']*np.fft.irfft(np.fft.rfft(unit)/(1+1j*wr*LEGACY['tau_effective_s']),n)
 direct=signal(np.conj(direct_port(mu))).real;force=PAR['Gamma']*(direct+group)
 phi,v=rk4(force,dt);drop=round(DISCARD/dt);x=phi[drop:].copy();x-=x.mean()
 ix=np.flatnonzero((x[:-1]<=0)&(x[1:]>0));tc=(ix-x[ix]/(x[ix+1]-x[ix]))*dt;periods=np.diff(tc)
 if len(periods)<2:raise ValueError('Insufficient crossings; do not substitute a safe response')
 accel=(force-A['C']*phi-A['alpha']*v-A['beta']*abs(v)*v)/A['I']
 gain=math.exp(-LEGACY['frozen_kappa']*.5*co*co*(1-co));deg=180/np.pi
 m0=float(np.var(phi[drop:]))*deg**2*gain**2
 m2=float(np.var(v[drop:]))*deg**2*gain**2;m4=float(np.var(accel[drop:]))*deg**2*gain**2
 return dict(seed=seed,dt=dt,centered_SD_deg=math.sqrt(m0),stationary_variance_deg2=m0,
  crossing_period_s=float(periods.mean()),period_SD_s=float(periods.std(ddof=1)),
  diagnostic_m2_deg2_s2=m2,diagnostic_m4_deg2_s4=m4,
  Gaussian_peak_period_diagnostic_s=float(2*np.pi*math.sqrt(m2/m4)),
  spectral_higher_moments_have_no_independent_EFD_confirmation=True,
  gain_is_SD_mapping_not_validated_whole_trajectory=True,mean_inclination_predicted=False)
def anchor(case,seed,dt=.05):
 c=INPUTS[str(case)];return forward(DATA['case'+str(case)],c['speed'],c['wave_from_deg'],seed,dt)
def forecast(state,seed,dt=.05):
 w,retained=spectrum(state['Hm0_m'],state['Tp_s']);r=forward(w,state['U_m_s'],state['mu_deg'],seed,dt)
 r['spectral_mass_fraction_in_frozen_support']=retained;return r
def all_seeds(state):
 rows=[forecast(state,s) for s in SEEDS]
 return dict(version=VERSION,state=state,rows=rows,
  mean_variance_deg2=float(np.mean([r['stationary_variance_deg2'] for r in rows])),
  mean_SD_deg=float(np.mean([r['centered_SD_deg'] for r in rows])),
  mean_period_s=float(np.mean([r['crossing_period_s'] for r in rows])),
  evidence_role='CONDITIONAL_ROUTE_TRANSFER_NOT_NEW_PHYSICAL_VALIDATION')
