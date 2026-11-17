# -*- coding: utf-8 -*-
"""Own frozen roll equation + finite, response-equivalent horizontal operator.

The three new damping parameters belong ONLY to the auxiliary block. Source
radiation/wave inputs are external ingredients, not a replacement roll model.
"""
from pathlib import Path
import sys,json,math,hashlib
import numpy as np
import hydro as h
import fast_bridge
P=h.P;O=h.O;Q=[0,1,3]
sys.path.insert(0,str(O/'oe_decision_impact_v1'));import response_model as roll
C=h.read(P/'protocol.json');FOILS=h.read(O/'dtmb_lateral_provider_v1/provider_contract.json')['foils']
TAU=h.read(O/'control_coupling_repair_v1/controller_port_check.json')['tau_s']
NU=np.array(C['radiation_grid_nu']);WW=np.array(C['frequency_grid_W']);M=8431800.;L=142.;IY=M*(.25*L)**2
_RAD_CACHE={};_WAVE_CACHE={}
def interp_complex(grid,arr,x):
 arr=np.asarray(arr);out=np.empty((len(x),)+arr.shape[1:],complex)
 for ix in np.ndindex(arr.shape[1:]):
  v=arr[(slice(None),)+ix];out[(slice(None),)+ix]=np.interp(x,grid,v.real)+1j*np.interp(x,grid,v.imag)
 return out
def radians(k):return math.radians(k)
def radfields(profile,nu,stride=4):
 key=(profile,nu.tobytes(),stride)
 if key in _RAD_CACHE:return _RAD_CACHE[key]
 data=[]
 for j in range(len(NU)):
  f=P/f'hydro_cache/r_{profile}_{j:02d}_s{stride}.json';data.append(h.read(f))
 R=np.array([h.unpack(x['R']) for x in data]);V=np.array([h.unpack(x['radiation_velocity']) for x in data])
 # Preserve the zero-frequency scaling of reactive/dissipative force components.
 added=R.real/NU[:,None,None]**2;damp=R.imag/NU[:,None,None]
 out=(interp_complex(NU,added,nu)*nu[:,None,None]**2+1j*interp_complex(NU,damp,nu)*nu[:,None,None],interp_complex(NU,V,nu))
 _RAD_CACHE[key]=out;return out
def wavefields(profile,mu,w):
 tag=f's{mu}' if profile!='route' else f'route{mu}'
 if tag not in _WAVE_CACHE:
  data=[h.read(P/f'hydro_cache/w_{tag}_{j:02d}_s4.json') for j in range(len(WW))]
  # Interpolate in the common native signed-frequency convention. Canonical
  # positive-frequency phasors flip conjugacy at encounter=0, so directly
  # interpolating them would mix two representations of different branches.
  values=[]
  for key in ['F','wave_velocity']:
   a=np.array([h.unpack(x[key]) for x in data]);negative=np.array([x['signed_encounter_rad_s']<0 for x in data]);a[negative]=a[negative].conj();values.append(a)
  _WAVE_CACHE[tag]=values
 F,V=_WAVE_CACHE[tag];outF=interp_complex(WW,F,w);outV=interp_complex(WW,V,w)
 negative=w+C['profiles'][profile]*w*w/9.81*math.cos(radians(mu))<0
 outF[negative]=outF[negative].conj();outV[negative]=outV[negative].conj()
 return outF,outV

def damping(U,parameters):
 D0,B0=source_D(U);cross=.5*(D0[1,2]+D0[2,1]);bu=B0[0]*np.exp(parameters[0]);bv=B0[1]*np.exp(parameters[1])
 return np.array([bu,bv,cross*cross/bv+B0[2]*np.exp(parameters[2])])
def wave_external(profile,mu,U,w):
 F,V=wavefields(profile,mu,w)
 for j,f in enumerate(FOILS):
  pos=np.array(f['r']);n=np.array(f['n']);cross=np.cross(pos,n);g=np.r_[n[:2],cross[[0,2]]]
  lift=.5*h.RHO*U*f['area']*f['lift_slope_per_rad'];F+=lift*(V[:,j]@n)[:,None]*g
 return F[:,Q]
def source_D(U):
 # Franceschi2021 bare-hull Table13, origin correction already source audited.
 Dmid=.5*h.RHO*U*np.array([[-.0117*L**2,-.00321*L**3],[-.00650*L**3,-.00179*L**4]])
 A=np.array([[1.,.683],[0.,1.]]);Dcg=A.T@Dmid@A
 Re=U*L/1e-6;z=math.log10(Re)-2;cf=.075/z**2;dcf=-.15/(math.log(10)*U*z**3)
 Bu=h.RHO*2972.6*U*cf+.5*h.RHO*2972.6*U*U*dcf
 D=np.zeros((3,3));D[0,0]=-Bu;D[1:,1:]=Dcg
 return D,np.abs(np.diag(D))
def operator(profile,U,nu,B):
 nu=np.asarray(nu);R,V=radfields(profile,nu);Z=-nu[:,None,None]**2*np.diag([M,M,0,IY])[None,:,:]-R
 # Bare-hull whole real velocity component replaced once. Not added twice to R.
 Tv=np.zeros((len(nu),3,3),complex);Tv[:,0,0]=Tv[:,1,1]=Tv[:,2,2]=-1j*nu;Tv[:,1,2]=-U
 Dpot=np.linalg.solve(np.swapaxes(Tv,1,2),np.swapaxes(R[:,Q][:,:,Q],1,2)).swapaxes(1,2)
 D,_=source_D(U);D[np.diag_indices(3)]=-np.asarray(B)
 correction=(D[None,:,:]-Dpot.real)@Tv
 Z[:,np.array(Q)[:,None],np.array(Q)[None,:]]-=correction
 for j,f in enumerate(FOILS):
  pos=np.array(f['r']);n=np.array(f['n']);cross=np.cross(pos,n);g=np.r_[n[:2],cross[[0,2]]]
  basis=np.array([[1.,0.,0.],[0.,1.,0.],np.cross([1.,0.,0.],pos),np.cross([0.,0.,1.],pos)])
  local=V[:,:,j,:]@n+1j*nu[:,None]*(basis@n)[None,:];local[:,3]+=U*n[1]
  if f['name'].startswith('rudder'):local[:,3]-=U*(-.8)
  else:local[:,2]-=U*5.*(-1j*nu)/(1-1j*nu*TAU)
  lift=.5*h.RHO*U*f['area']*f['lift_slope_per_rad'];Z-=lift*g[None,:,None]*local[:,None,:]
 Zqq=Z[:,Q][:,:,Q];Zqp=Z[:,Q,2]
 # Force/moment and displacement scales prevent metres/radians from defining condition.
 T=np.diag([L,L,1.]);cond=np.linalg.cond(T[None,:,:]@Zqq@T[None,:,:])
 if not np.isfinite(cond).all() or cond.max()>1e12:raise ValueError('Auxiliary inverse not numerically admitted')
 inv=np.linalg.inv(Zqq);transfer=np.concatenate([inv,(-inv@Zqp[:,:,None])],axis=2)
 return transfer,float(cond.max())
def frozen_trace(weights,U,mu,seed,dt=.05,Vlat=0.,relative_signed=None):
 """Exact existing roll at Vlat=0; only input kinematics may be extended."""
 co=0. if mu==90 else math.cos(radians(mu));signed=mu if relative_signed is None else relative_signed
 adv=U*math.cos(radians(signed))-Vlat*math.sin(radians(signed))
 w=roll.W;nu=w+adv*w*w/9.81;n=round(roll.DURATION/dt);bins=np.rint(nu*roll.DURATION/(2*np.pi)).astype(int)%n
 rng=np.random.default_rng(seed);z=(rng.normal(size=len(w))+1j*rng.normal(size=len(w)))*np.sqrt(weights)
 def signal(mult):
  ff=np.zeros(n,complex);np.add.at(ff,bins,z*mult);return np.fft.ifft(ff)*n
 u=signal(w*co).real;hh=np.zeros(n);hh[0]=hh[n//2]=1;hh[1:n//2]=2
 aa=np.fft.ifft(np.fft.fft(u)*hh);om=2*np.pi*np.fft.fftfreq(n,d=dt)
 unit=np.real(np.conj(aa)*np.fft.ifft(1j*om*np.fft.fft(aa)));wr=2*np.pi*np.fft.rfftfreq(n,d=dt)
 group=roll.LEGACY['D_effective']*np.fft.irfft(np.fft.rfft(unit)/(1+1j*wr*roll.LEGACY['tau_effective_s']),n)
 direct=signal(np.conj(roll.direct_port(mu))).real;force=roll.PAR['Gamma']*(direct+group)
 phi=fast_bridge.trace(force,dt,roll.A);gain=math.exp(-roll.LEGACY['frozen_kappa']*.5*co*co*(1-co))
 # New explicit auxiliary coupling hypothesis: use the frozen SD mapping on its
 # driving trajectory, with no new parameter; auxiliary confirmation tests this use.
 return phi*gain,z,bins,nu
def source_spectrum(case):
 out=P/f'cov_s{case}.npz'
 if out.exists():
  cached=np.load(out)
  assert str(cached['execution_contract_sha256'])==h.sha(P/'execution_contract.json')
  return dict(file=str(out),sha256=h.sha(out),per_channel_retained_spectral_mass=cached['retained'].tolist(),seeds=C['development']['seeds'],physical_new_experiments=0)
 profile='s'+str(case);mu=case;U=C['profiles'][profile];weights=roll.DATA['case'+str({0:215,45:205,90:206,180:218}[case])]
 w=roll.W;sel=np.flatnonzero(weights>0);F=wave_external(profile,mu,U,w[sel]);seeds=C['development']['seeds'];cov=None;frequency=None;drop=round(512/.05)
 for seed in seeds:
  phi,z,bins,signednu=frozen_trace(weights,U,mu,seed);n=len(phi)
  forcing=np.empty((3,n))
  for j in range(3):
   ff=np.zeros(n,complex);mult=np.where(signednu[sel]>=0,np.conj(F[:,j]),F[:,j]);np.add.at(ff,bins[sel],z[sel]*mult);forcing[j]=(np.fft.ifft(ff)*n).real
  traces=np.vstack([forcing[:,drop:],phi[None,drop:]]);traces-=traces.mean(axis=1,keepdims=True)
  N=traces.shape[1];fft=np.fft.rfft(traces,axis=1).T/N;f=2*np.pi*np.fft.rfftfreq(N,d=.05)
  factor=np.full(len(f),2.);factor[[0,-1]]=1
  cc=fft[:,:,None]*fft[:,None,:].conj()*factor[:,None,None]
  cov=cc if cov is None else cov+cc;frequency=f
 cov/=len(seeds);total=np.real(np.diagonal(cov,axis1=1,axis2=2)).sum(axis=0)
 mask=(frequency>=.0001)&(frequency<=12.)
 retained=np.real(np.diagonal(cov[mask],axis1=1,axis2=2)).sum(axis=0)/np.maximum(total,1e-300)
 np.savez_compressed(out,nu=frequency[mask],cov=cov[mask],retained=retained,U=U,mu=mu,execution_contract_sha256=h.sha(P/'execution_contract.json'))
 return dict(file=str(out),sha256=h.sha(out),per_channel_retained_spectral_mass=retained.tolist(),seeds=seeds,physical_new_experiments=0)
def predict(case,parameters):
 data=np.load(P/f'cov_s{case}.npz');nu=data['nu'];cov=data['cov'];U=float(data['U']);profile='s'+str(case);_,B0=source_D(U)
 transfer,condition=operator(profile,U,nu,damping(U,parameters));H=np.conj(transfer)
 outH=np.stack([1j*nu[:,None]*H[:,0],1j*nu[:,None]*H[:,1]-U*H[:,2],H[:,2],1j*nu[:,None]*H[:,2]],axis=1)
 power=np.real(np.einsum('nki,nij,nkj->nk',outH,cov,outH.conj()))
 if power.min()< -1e-9*max(power.max(),1e-30):raise ValueError('Negative response power')
 power=np.maximum(power,0);m0=power.sum(axis=0);m2=(power*nu[:,None]**2).sum(axis=0);deg=180/np.pi
 return dict(surge_SD_m_s=float(np.sqrt(m0[0])),sway_SD_m_s=float(np.sqrt(m0[1])),yaw_SD_deg=float(np.sqrt(m0[2])*deg),
  yaw_rate_RMS_deg_s=float(np.sqrt(m0[3])*deg),yaw_mean_crossing_period_diagnostic_s=float(2*np.pi*np.sqrt(m0[2]/m2[2])),
  condition=condition,retained=data['retained'].tolist(),rRMS_not_directly_measured_in_source=True)
