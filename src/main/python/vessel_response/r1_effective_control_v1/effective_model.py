# -*- coding: utf-8 -*-
"""Existing physical ports with2new effective closed-loop coefficients only."""
from pathlib import Path
import os,sys,json,time,datetime
for n in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[n]='2'
import numpy as np
P=Path(__file__).resolve().parent;O=P.parent
sys.path.insert(0,str(O/'r1_fk_cross_closure_v1'))
import run_prediction as base
import check_nonlinear_hull as nl
m=base.m;h=base.h;g=base.g;FOLDER=base.P
E=np.array([[1,0,0,0],[0,1,0,0],[0,0,0,1.]])
CFG=h.read(FOLDER/'nonlinear_hull_protocol.json')

def save(n,x):
    q=P/n;q.parent.mkdir(exist_ok=True,parents=True)
    with q.open('x',encoding='utf-8') as f:json.dump(x,f,ensure_ascii=False,indent=2,allow_nan=False)

def prepare_input(case,seed):
    q=P/f'input_cache/case{case}_seed{seed}.npz'
    if q.exists():return q
    c=base.context(case);n=c['n'];sel=c['sel'];mu=case
    zgeo=dict(np.load(FOLDER/'quadrature_geometry_s2.npz'));C=np.load(FOLDER/f'cross_gauss_s2_mu{mu}.npz')['C']
    phi,z,bins,signed=m.frozen_trace(c['weights'],c['U'],mu,seed);forces=np.empty((3,n));cross=np.empty((3,n))
    for j in range(3):
        ff=np.zeros(n,complex);np.add.at(ff,bins[sel],z[sel]*np.where(signed[sel]>=0,c['F'][:,j].conj(),c['F'][:,j]));forces[j]=(np.fft.ifft(ff)*n).real
        ff=np.zeros(n,complex);np.add.at(ff,bins[sel],z[sel]*C[:,j].conj());cross[j]=phi*(np.fft.ifft(ff)*n).real
    wl=base.geometry_waterline_force(zgeo,c['w'],z,bins,sel,mu,n)
    traces=np.vstack([forces+cross+wl,phi]);traces-=traces.mean(axis=1,keepdims=True);ft=np.fft.rfft(traces,axis=1).T
    ff=np.zeros(n,complex);np.add.at(ff,bins,z);eta=(np.fft.ifft(ff)*n).real;etaft=np.fft.rfft(eta-eta.mean())
    q.parent.mkdir(exist_ok=True);np.savez_compressed(q,forcing_fft=ft,wave_fft=etaft,nuall=c['nuall'],U=c['U'],n=n,drop=c['drop'],
        execution_source_sha256=h.sha(FOLDER/'protocol.json'),geometric_mesh_gate_sha256=h.sha(FOLDER/'pilot_summary.json'))
    return q

def jacobian(x,U):
    vp=(x[1]+.683*x[3])/U;rp=x[3]*m.L/U;scale=.5*h.RHO*U*U*m.L*m.L
    J=np.zeros((3,3));X=CFG['X'];Y=CFG['Y'];N=CFG['N']
    dxv=scale/U*np.mean(2*X['vv']*vp+X['vr']*rp);dxr=scale*m.L/U*np.mean(2*X['rr']*rp+X['vr']*vp)
    dyv=scale/U*np.mean(2*Y['v_abs_v']*abs(vp)+2*Y['v2r']*vp*rp+Y['vr2']*rp*rp)
    dyr=scale*m.L/U*np.mean(2*Y['r_abs_r']*abs(rp)+Y['v2r']*vp*vp+2*Y['vr2']*vp*rp)
    dnv=scale*m.L/U*np.mean(2*N['v_abs_v']*abs(vp)+2*N['v2r']*vp*rp+N['vr2']*rp*rp)
    dnr=scale*m.L*m.L/U*np.mean(2*N['r_abs_r']*abs(rp)+N['v2r']*vp*vp+2*N['vr2']*vp*rp)
    J[:,1]=[dxv,dyv,dnv+.683*dyv];J[:,2]=np.array([dxr,dyr,dnr+.683*dyr])+.683*J[:,1]
    return J

def solve(case,seed,parameters,maxiter=60):
    start=time.perf_counter();raw=np.load(prepare_input(case,seed));ft=raw['forcing_fft'];nuall=raw['nuall'];U=float(raw['U']);n=int(raw['n']);drop=int(raw['drop']);mask=(nuall>=.0001)&(nuall<=12);nu=nuall[mask]
    controller=dict(P=-float(parameters[0]),D_s=-float(parameters[1]));T,cond=g.operator('s'+str(case),U,nu,m.damping(U,np.zeros(3)),controller);T=T.conj()
    H=np.stack([1j*nu[:,None]*T[:,0],1j*nu[:,None]*T[:,1]-U*T[:,2],T[:,2],1j*nu[:,None]*T[:,2]],axis=1);HF=H[:,:,:3]
    fft0=np.einsum('nki,ni->nk',H,ft[mask]);spectrum=np.zeros((len(nuall),4),complex);spectrum[mask]=fft0;initial=np.fft.irfft(spectrum.T,n,axis=1);x=initial.copy();history=[];converged=False
    for it in range(maxiter):
        if datetime.datetime.now(datetime.timezone.utc)>=datetime.datetime(2026,10,20,5,15,tzinfo=datetime.timezone.utc):break
        force=nl.forces(x,U,CFG);ff=np.fft.rfft(force,axis=1).T;truefft=fft0+np.einsum('nki,ni->nk',HF,ff[mask]);spectrum[:]=0;spectrum[mask]=truefft;target=np.fft.irfft(spectrum.T,n,axis=1)
        residual=np.sqrt(np.mean((target-x)**2,axis=1))/np.maximum(np.sqrt(np.mean(x*x,axis=1)),[1e-5,1e-5,1e-6,1e-6]);history.append(float(residual.max()))
        if not np.isfinite(target).all() or abs(target[:2]).max()>100 or abs(target[2:]).max()>10:break
        if residual.max()<1e-3:converged=True;break
        J=jacobian(x,U);velocity_fft=np.fft.rfft(E@x,axis=1).T
        residual_force=ff[mask]-velocity_fft[mask]@J.T
        rhs=fft0+np.einsum('nki,ni->nk',HF,residual_force)
        loop=np.eye(4)[None]-np.einsum('nki,ij,jl->nkl',HF,J,E)
        try:newfft=np.linalg.solve(loop,rhs[...,None])[...,0]
        except np.linalg.LinAlgError:break
        spectrum[:]=0;spectrum[mask]=newfft;new=np.fft.irfft(spectrum.T,n,axis=1)
        if not np.isfinite(new).all():break
        x=.75*x+.25*new
    stat=base.stats(x[:,drop:]);freq=nuall/(2*np.pi);w0=2*np.pi/10.4;target=abs(w0+U*w0*w0/9.81*np.cos(np.deg2rad(case)))/(2*np.pi);band=abs(freq-target)<=.005
    xf=np.fft.rfft(x-x.mean(axis=1,keepdims=True),axis=1);den=float((abs(raw['wave_fft'][band])**2).sum());rao=np.sqrt((abs(xf[:,band])**2).sum(axis=1)/den);rao[2:]*=180/np.pi
    return dict(case=case,seed=seed,parameters=list(map(float,parameters)),converged=converged,iterations=len(history),true_residual_history=history,
        statistics=stat,RAO_pk=dict(u=float(rao[0]),v=float(rao[1]),psi=float(rao[2]),r=float(rao[3])),condition=cond,
        max_vprime=float(abs((x[1]+.683*x[3])/U).max()),max_rprime=float(abs(x[3]*m.L/U).max()),wall_seconds=time.perf_counter()-start,
        coefficients_are_effective_closed_loop_not_unique_actual_controller=True)

def aggregate(rows):
    out={k:float(np.sqrt(np.mean([r['statistics'][k]**2 for r in rows]))) for k in ['surge_SD_m_s','sway_SD_m_s','yaw_SD_deg','yaw_rate_RMS_deg_s']}
    out['yaw_mean_crossing_period_s']=float(np.mean([r['statistics']['yaw_mean_crossing_period_s'] for r in rows]));return out
