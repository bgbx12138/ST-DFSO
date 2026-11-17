# -*- coding: utf-8 -*-
"""Geometry-induced second-order FK input, not a new hydrodynamic solver."""
from pathlib import Path
import os,sys,json,time,hashlib,shutil,math
for n in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[n]='2'
import numpy as np
P=Path(__file__).resolve().parent;O=P.parent
sys.path.insert(0,str(O/'r1_completion_v1'));import hydro as h

def save(n,x):
    with (P/n).open('x',encoding='utf-8') as f:json.dump(x,f,ensure_ascii=False,indent=2,allow_nan=False)
def geometry(stride):
    b=h.body(stride);s=h.S;xyz=b.mesh.faces_centers*s;r=xyz-h.CG*s;n=b.mesh.faces_normals;area=b.mesh.faces_areas*s*s
    mom=np.cross(r,n);g=np.column_stack([n[:,:2],mom[:,2]])
    rot=np.column_stack([np.zeros(len(n)),-n[:,2],mom[:,1]])
    vertices=b.mesh.vertices*s;faces=b.mesh.faces;counts={};owners={}
    for j,f in enumerate(faces):
        seq=list(dict.fromkeys(f.tolist()))
        for a,c in zip(seq,seq[1:]+seq[:1]):
            edge=tuple(sorted((a,c)));counts[edge]=counts.get(edge,0)+1;owners[edge]=j
    wp=[];wg=[];other=[]
    for edge,count in counts.items():
        if count!=1:continue
        a,c=vertices[list(edge)];delta=c-a;length=np.linalg.norm(delta)
        if max(abs(a[2]),abs(c[2]))>1e-6:other.append([a.tolist(),c.tolist()]);continue
        normal=np.array([delta[1],-delta[0],0.])/length
        if normal@n[owners[edge]]<0:normal=-normal
        for t in [.5-.5/np.sqrt(3),.5+.5/np.sqrt(3)]:
            point=a+t*delta;rr=point-h.CG*s;gp=np.array([normal[0],normal[1],np.cross(rr,normal)[2]])
            wp.append(point);wg.append(gp*length/2)
    wp=np.array(wp);wg=np.array(wg)
    assert len(wp)>100,('No resolved mean-waterline boundary',len(wp))
    # Keep the canonical CAD/BEM mesh unchanged. Its zero-area stem seam and
    # sub-millimetre clipping slivers are not additional waterline segments.
    # Quantify them and use the predeclared force-integral identities below.
    return dict(xyz=xyz,r=r,n=n,area=area,g=g,rot=rot,wl=wp,wl_g=wg,stride=stride,
                non_waterline_index_boundary_edges=np.array(other).reshape(-1,2,3))

def roll_wave_coeff(z,w,mu):
    w=np.asarray(w);k=w*w/9.81;beta=math.radians(180-mu);out=[]
    Y=z['r'][:,1,None]*z['g']*z['area'][:,None]
    Z=z['r'][:,2,None]*z['g']*z['area'][:,None]
    R=z['rot']*z['area'][:,None];W=z['wl'][:,1,None]*z['wl_g']
    for a in range(0,len(w),128):
        kk=k[a:a+128];ky=kk*math.sin(beta)
        phase=z['xyz'][:,0]*math.cos(beta)+z['xyz'][:,1]*math.sin(beta)
        pressure=h.RHO*9.81*np.exp(kk[:,None]*(z['xyz'][:,2]+1j*phase)[None,:])
        surf=-(kk[:,None]*(pressure@Y)-1j*ky[:,None]*(pressure@Z)+pressure@R)
        wp=z['wl'][:,0]*math.cos(beta)+z['wl'][:,1]*math.sin(beta)
        line=h.RHO*9.81*np.exp(1j*kk[:,None]*wp[None,:])@W
        out.append(surf+line)
    return np.concatenate(out)

def main():
    start=time.perf_counter();files=[h.P/'model.py',h.P/'hydro.py',h.P/'auxiliary_parameter_seal.json',
        O/'oe_moment_calibration_v1/parameter_seal.json',O/'r1_global_appendage_closure_v1/global_operator.py']
    files +=[O/f'dtmb_passive_wave_prediction_v1/mesh_s{s}.npz' for s in [4,2]]
    backup=[]
    if (P/'backup_manifest.json').exists():
        backup=h.read(P/'backup_manifest.json')
        for r in backup:assert h.sha(r['path'])==r['sha256']
    else:
        (P/'before').mkdir()
        for j,q in enumerate(files):
            t=P/'before'/f'{j:02}_{q.name}';shutil.copy2(q,t);assert h.sha(q)==h.sha(t);backup.append(dict(path=str(q),backup=str(t),sha256=h.sha(q)))
        save('backup_manifest.json',backup)
    contract=dict(date='2026-10-20',scope='Geometry second-order FK increments to horizontal forces only, no radiation/diffraction recalculation',
        pressure='pI=rho*g*eta exp(kz); mean hull rotated about frozen CG. Exact first derivative includes rotated normal/moment and moving waterline.',
        roll_wave_formula='Cphi=-int[pI*(k*y-i*ky*(z-zG))*g+pI*(ex cross g)]dS +rho*g*int_WL[eta*y*gWL]dl',
        wave_wave_formula='DeltaF_WL=-rho*g/2 *int_WL[eta(x,y,t)^2*gWL]dl',
        g_definition='force/moment selector [nx,ny,(r cross n)z]; derivative [0,-nz,(r cross n)y]; pressure acts inward -n',
        no_double_count='Only O(phi*eta) and O(eta^2) changes added to existing linear FK; no static restoring, linear FK or roll feedback added again.',
        omitted='Second-order incident/diffraction potentials, radiation nonlinearities, heave/pitch states and higher-order large-roll terms. Not full second-order hydrodynamics.',
        mesh_gate='Uniform pressure identity relative<1e-3; input-spectrum-weighted s4/s2 Cphi force/moment norm difference<=10percent.',
        full_prediction_mesh_gate='One original seed, responseSD differences<=10percent before eight-seed run; report absolute floor handling.',
        time_budget_seconds=2400,deadline_UTC='2026-10-20T05:18:00+00:00',new_free_parameters=0,
        existing_states='Existing roll trace drives own horizontal operator; no feedback into frozen roll is introduced.',
        phase='Preserve intrinsic signed-encounter branch and native exp(+ik.r-i nu_signed t); physical FFT coefficients use conjugate native spatial amplitude before signed-bin synthesis.')
    if (P/'protocol.json').exists():assert h.read(P/'protocol.json')==contract
    else:save('protocol.json',contract)
    test_w=np.array([0.,.4,2*np.pi/10.4,.8,1.2,1.65]);results=[];values={}
    for stride in [4,2]:
        z=geometry(stride);zero=roll_wave_coeff(z,[0.],90)[0]
        scale=h.RHO*9.81*np.array([142*19.06,142*19.06,142*142*19.06]);err=float(np.linalg.norm(zero/scale));assert err<1e-3
        const=(z['g']*z['area'][:,None]).sum(axis=0);consterr=float(np.linalg.norm(const/(scale/(h.RHO*9.81))));assert consterr<1e-3
        np.savez_compressed(P/f'geometry_s{stride}.npz',**z)
        for mu in [0,45,90,180]:values[stride,mu]=roll_wave_coeff(z,test_w,mu)
        results.append(dict(stride=stride,faces=len(z['area']),waterline_quadrature_points=len(z['wl']),uniform_pressure_derivative=zero.tolist() if not np.iscomplexobj(zero) else np.stack([zero.real,zero.imag],axis=-1).tolist(),uniform_relative_error=err,
            constant_pressure_horizontal_force_relative=consterr,unmerged_nonwaterline_index_edges=z['non_waterline_index_boundary_edges'].tolist()))
    compare=[]
    for mu in [0,45,90,180]:
        scale=np.array([1,1,1/142]);a=values[4,mu][:,]*scale;b=values[2,mu]*scale
        err=float(np.linalg.norm(a-b)/max(np.linalg.norm(b),1e-100));compare.append(dict(mu=mu,pilot_coefficient_relative_difference=err))
    # Pilot unweighted values only diagnose numerical shape; full independent-input
    # weighted check is required before any physical prediction.
    save('geometry_preflight.json',dict(geometries=results,pilot_comparison=compare,full_spectrum_gate_still_required=True,wall_seconds=time.perf_counter()-start))
    print(json.dumps(dict(geometries=results,pilot_comparison=compare,seconds=time.perf_counter()-start),indent=2))

