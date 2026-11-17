# -*- coding: utf-8 -*-
"""Sourced first-quadrant constant-RPM port; no response fitting.

The nominal friction-only self-propulsion point is an explicitly idealized
reference, not the unpublished MARIN experimental shaft setting.
"""
import math
import numpy as np

RHO=1000.; G=9.81; L=142.; SWET=2972.6; D=2*.02166*L
KT=np.array([.41249,-.31135,-.13264])
KQ=np.array([.052741,-.02917,-.02251])
J_ZERO=float(np.roots(KT[::-1])[1]) if np.roots(KT[::-1])[1]>0 else float(np.roots(KT[::-1])[0])

def kt(j):return KT[0]+KT[1]*j+KT[2]*j*j
def ktprime(j):return KT[1]+2*KT[2]*j
def kq(j):return KQ[0]+KQ[1]*j+KQ[2]*j*j

def friction(u):
    u=np.asarray(u);z=np.log10(u*L/1e-6)-2
    return .5*RHO*SWET*u*u*.075/z**2

def friction_slope(u):
    z=math.log10(u*L/1e-6)-2
    return RHO*SWET*u*.075/z**2-.5*RHO*SWET*u*.15/(math.log(10)*z**3)

def thrust(n,va):
    j=np.asarray(va)/(n*D)
    return 2*RHO*n*n*D**4*kt(j)

def point(U0=18*.5144444444444445):
    # 2rho D^4 [a n^2+b(U/D)n+c(U/D)^2]=R_F(U).
    # w=t=0 is a declared idealization. No measured wave response enters.
    a,b,c=KT;v=U0/D;right=float(friction(U0))/(2*RHO*D**4)
    n=float((-b*v+math.sqrt((b*v)**2-4*a*(c*v*v-right)))/(2*a))
    return dict(U0_m_s=U0,n_rps_full=n,n_rpm_full=60*n,
        n_rps_model35_48=n*math.sqrt(35.48),J0=U0/(n*D),
        R0_N=float(friction(U0)),T0_N=float(thrust(n,U0)),
        wake=0.,thrust_deduction=0.,actual_experimental_workpoint=False,
        origin='Friction-only zero-wake zero-thrust-deduction nominal self-propulsion reference; not total resistance or MARIN RPM')

def tangent(U,n,wake=0.,deduction=0.):
    j=(1-wake)*U/(n*D)
    bp=-2*RHO*n*D**3*ktprime(j)*(1-wake)*(1-deduction)
    return dict(J=j,B_R_N_s_m=friction_slope(U),B_P_N_s_m=float(bp),
        B_total_N_s_m=friction_slope(U)+float(bp),
        d_net_force_du_N_s_m=-(friction_slope(U)+float(bp)))

def replace_operator(g,profile,U,nu,B,control,workpoint):
    """Replace the existing real longitudinal velocity block once.

    Native frequency convention exp(-i nu t); retain reactive and every
    lateral/roll/control entry. The caller adds the relative orbital input once.
    """
    zo,z,zphi,b,tv,deff,target=g.terms(profile,U,nu,B)
    z=z.copy();new=tangent(U,workpoint['n_rps_full'],workpoint['wake'],workpoint['thrust_deduction'])
    delta=-new['B_total_N_s_m']-target[0,0]
    z[:,0,0]-=delta*tv[:,0,0]
    gain=control['P']-1j*np.asarray(nu)*control['D_s']
    z[:,:,2]-=b[None,:]*gain[:,None]
    inv=np.linalg.inv(z)
    return np.concatenate([inv,-np.einsum('nij,nj->ni',inv,zphi)[:,:,None]],axis=2),new
