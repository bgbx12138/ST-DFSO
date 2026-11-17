# -*- coding: utf-8 -*-
"""One sourced total passive horizontal velocity replacement, no new fit."""
from pathlib import Path
import sys
import numpy as np
P=Path(__file__).resolve().parent;O=P.parent
sys.path.insert(0,str(O/'auxiliary_input_control_closure_v1'))
import check_control_ports as c
m=c.m;h=c.h

def sourced_global_D(U):
    # Franceschi2021 Table13 Appended. Both lateral sign conventions reverse,
    # leaving these derivatives invariant. Correct midship->CG once by congruence.
    raw=.5*h.RHO*U*np.array([[-.0110*m.L**2,+.000350*m.L**3],[-.00600*m.L**3,-.00279*m.L**4]])
    origin=np.array([[1.,.683],[0.,1.]])
    total=np.zeros((3,3));total[0,0]=m.source_D(U)[0][0,0];total[1:,1:]=origin.T@raw@origin
    return total

def terms(profile,U,nu,B):
    nu=np.asarray(nu);Zo,Zphi,b=c.matrices(profile,U,nu,B)
    Tv=np.zeros((len(nu),3,3),complex);Tv[:,0,0]=Tv[:,1,1]=Tv[:,2,2]=-1j*nu;Tv[:,1,2]=-U
    # Zo=-nu² M-Rtotal, where Rtotal already contains passive hull+foil forces.
    mass=np.diag([m.M,m.M,m.IY]);R=-nu[:,None,None]**2*mass[None]-Zo
    Deff=np.linalg.solve(Tv.swapaxes(1,2),R.swapaxes(1,2)).swapaxes(1,2)
    target=sourced_global_D(U)
    Znew=Zo-(target[None]-Deff.real)@Tv
    return Zo,Znew,Zphi,b,Tv,Deff,target

def operator(profile,U,nu,B,controller):
    Zo,Znew,Zphi,b,Tv,Deff,target=terms(profile,U,nu,B)
    K=controller['P']-1j*np.asarray(nu)*controller['D_s']
    Znew[:,:,2]-=b[None,:]*K[:,None]
    inv=np.linalg.inv(Znew);T=np.concatenate([inv,-np.einsum('nij,nj->ni',inv,Zphi)[:,:,None]],axis=2)
    scale=np.diag([m.L,m.L,1.]);condition=float(np.linalg.cond(scale[None]@Znew@scale[None]).max())
    if not np.isfinite(T).all() or condition>1e12:raise ValueError('Total-appendage operator not numerically admitted')
    return T,condition

def check(case,params):
    U=m.C['profiles']['s'+str(case)];nu=np.array([.02,.05,.1,.2,.4,.6,1.,2.]);B=m.damping(U,params)
    Zo,Znew,Zphi,b,Tv,D,target=terms('s'+str(case),U,nu,B)
    R=-nu[:,None,None]**2*np.diag([m.M,m.M,m.IY])[None]-Znew
    newD=np.linalg.solve(Tv.swapaxes(1,2),R.swapaxes(1,2)).swapaxes(1,2)
    real=float(np.linalg.norm(newD.real-target)/np.linalg.norm(target));imag=float(np.linalg.norm(newD.imag-D.imag)/max(np.linalg.norm(D.imag),1e-300))
    ident=float(np.linalg.norm(Zo-(Zo-(D.real-D.real)@Tv))/np.linalg.norm(Zo))
    T,_=operator('s'+str(case),U,nu,B,c.CONTROLLERS[case]);oi=np.linalg.inv(Znew)
    G=np.concatenate([oi,-np.einsum('nij,nj->ni',oi,Zphi)[:,:,None]],axis=2);gd=np.einsum('nij,j->ni',oi,b)
    loop=np.repeat(np.eye(3,dtype=complex)[None],len(nu),axis=0);K=c.CONTROLLERS[case]['P']-1j*nu*c.CONTROLLERS[case]['D_s'];loop[:,:,2]-=gd*K[:,None]
    direct=np.linalg.solve(loop,G);control=float(np.linalg.norm(direct-T)/np.linalg.norm(T))
    assert max(real,imag,ident,control)<1e-10
    return dict(case=case,real_passive_component_matches_source_relative=real,reactive_component_preservation_relative=imag,
        no_change_identity_relative=ident,independent_control_closure_relative=control,target_D=target.tolist(),
        body_velocity_map='u=xdot,v=ydot-Upsi,r=psidot',added_source_mass=False,frozen_roll_equation_untouched=True)
