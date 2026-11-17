# -*- coding: utf-8 -*-
"""Explicit rudder command port, source-configured closure; no parameter fit."""
from pathlib import Path
import sys, json, time, hashlib
import numpy as np

P=Path(__file__).resolve().parent;O=P.parent
sys.path.insert(0,str(O/'r1_completion_results_audit_v1'))
import window_consistent_response as audit
m=audit.m;h=audit.h
OLD=m.operator
# WMTC-2015-152 PDF p16 Table5 CFD/PF, not an undocumented EFD controller.
CONTROLLERS={0:dict(P=-.8,D_s=0.),45:dict(P=-.8,D_s=0.),90:dict(P=-.8,D_s=-24.9),180:dict(P=-.9,D_s=0.)}

def save(name,x):
    with (P/name).open('x',encoding='utf-8') as f:json.dump(x,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

def matrices(profile,U,nu,B):
    T,_=OLD(profile,U,nu,B)
    Z=np.linalg.inv(T[:,:,:3]);Zphi=-np.einsum('nij,nj->ni',Z,T[:,:,3])
    force=np.zeros(3)
    for foil in m.FOILS:
        if not foil['name'].startswith('rudder'):continue
        r=np.asarray(foil['r']);n=np.asarray(foil['n']);g=np.r_[n[:2],np.cross(r,n)[[0,2]]]
        force+=(-.5*h.RHO*U*foil['area']*foil['lift_slope_per_rad']*U)*g[m.Q]
    # Old operator already has P=-.8 feedback exactly once. Remove it first.
    Zopen=Z.copy();Zopen[:,:,2]+=force[None,:]*(-.8)
    return Zopen,Zphi,force

def explicit_operator(profile,U,nu,B,controller):
    Zo,Zphi,b=matrices(profile,U,nu,B);K=controller['P']-1j*np.asarray(nu)*controller['D_s']
    Z=Zo.copy();Z[:,:,2]-=b[None,:]*K[:,None]
    inv=np.linalg.inv(Z);T=np.concatenate([inv,-np.einsum('nij,nj->ni',inv,Zphi)[:,:,None]],axis=2)
    scale=np.diag([m.L,m.L,1.]);return T,float(np.linalg.cond(scale[None]@Z@scale[None]).max())

def main():
    start=time.perf_counter();seal=h.read(m.P/'auxiliary_parameter_seal.json');params=np.asarray(seal['log_parameters'])
    before={str(q):h.sha(q) for q in [m.P/'auxiliary_parameter_seal.json',O/'oe_moment_calibration_v1/parameter_seal.json']}
    checks=[]
    for case in [0,45,90,180]:
        profile='s'+str(case);U=m.C['profiles'][profile];nu=np.array([.02,.1,.4,.8,1.6])
        B=m.damping(U,params);old,_=OLD(profile,U,nu,B)
        matched,_=explicit_operator(profile,U,nu,B,dict(P=-.8,D_s=0.))
        rel=float(np.linalg.norm(matched-old)/np.linalg.norm(old));assert rel<1e-10
        Zo,Zphi,b=matrices(profile,U,nu,B);oi=np.linalg.inv(Zo)
        H=np.concatenate([oi,-np.einsum('nij,nj->ni',oi,Zphi)[:,:,None]],axis=2)
        Hdelta=np.einsum('nij,j->ni',oi,b);c=CONTROLLERS[case];K=c['P']-1j*nu*c['D_s']
        # Independent explicit-command closed-loop identity (no duplicated feedback).
        loop=np.eye(3,dtype=complex)[None].repeat(len(nu),axis=0)
        loop[:,:,2]-=Hdelta*K[:,None]
        independent=np.linalg.solve(loop,H);direct,_=explicit_operator(profile,U,nu,B,c)
        err=float(np.linalg.norm(direct-independent)/np.linalg.norm(direct));assert err<1e-10
        checks.append(dict(case=case,old_P_only_identity_relative_error=rel,
            source_PD_closed_loop_identity_relative_error=err,rudder_generalized_force_per_rad=b.tolist(),
            yaw_rate_feedback_dissipation_coefficient_Nms=num(b[2]*c['D_s']),controller=c))
    results={}
    try:
        for case in [90,180]:
            config=CONTROLLERS[case]
            m.operator=lambda profile,U,nu,B,c=config:explicit_operator(profile,U,nu,B,c)
            results[str(case)]=audit.source(case,params)
    finally:m.operator=OLD
    for q,digest in before.items():assert h.sha(q)==digest
    base=h.read(O/'r1_completion_results_audit_v1/window_corrected_source_check.json')
    comparison=[]
    for case,new in results.items():
        old=base['cases'][case]['mean_power_prediction'];new=new['mean_power_prediction']
        for field in ['surge_SD_m_s','sway_SD_m_s','yaw_SD_deg','yaw_rate_RMS_deg_s','yaw_mean_crossing_period_s']:
            comparison.append(dict(case=int(case),field=field,P_only=old[field],published_CFD_PF_control=new[field],
                change_percent=100*(new[field]/old[field]-1)))
    save('source_control_port_check.json',dict(checks=checks,source_case_predictions=results,comparison=comparison,
        source='WMTC-2015-152 PDF p16 Table5; feedback definition PDF p4',
        prediction_scope='Configuration sensitivity using published CFD/PF gains; actual EFD gains are not documented.',
        no_new_fitted_parameters=True,old_parameter_hashes_verified=before,physical_validation_passed=False,
        window_order='Full-record inverse before statistics crop',wall_seconds=time.perf_counter()-start))
    print(json.dumps(dict(checks=checks,comparison=comparison,wall_seconds=time.perf_counter()-start),indent=2))

def num(x):return float(x)
