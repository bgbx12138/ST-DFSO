# -*- coding: utf-8 -*-
"""Standard open-source wave/radiation inputs for our frozen roll + auxiliary model.
No self-written Green solver and no response-target access in this module.
"""
from pathlib import Path
import os,sys,json,hashlib,math,time,argparse,logging
for name in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[name]='2'
import numpy as np
P=Path(__file__).resolve().parent;O=P.parent
import capytaine as cpt
from threadpoolctl import threadpool_limits
logging.getLogger('capytaine').setLevel(logging.ERROR)
S=35.48;ROOT=next(q for q in P.parents if (q/'src').is_dir());CG=np.array([0.,0.,1.36/S]);RHO=1000.;G=9.81
DOFS=['Surge','Sway','Roll','Yaw']
def read(q):return json.loads(Path(q).read_text(encoding='utf-8'))
def sha(q):return hashlib.sha256(Path(q).read_bytes()).hexdigest()
def pack(x):
 z=np.asarray(x,complex);return np.stack([z.real,z.imag],axis=-1).tolist()
def unpack(x):
 z=np.asarray(x);return z[...,0]+1j*z[...,1]
def save(n,x):
 q=P/n;q.parent.mkdir(exist_ok=True,parents=True)
 with q.open('x',encoding='utf-8') as f:json.dump(x,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
def body(stride):
 z=np.load(O/f'dtmb_passive_wave_prediction_v1/mesh_s{stride}.npz')
 mesh=cpt.Mesh(vertices=z['vertices'],faces=z['faces'],auto_check=False)
 lid=cpt.Mesh(vertices=z['lid_vertices'],faces=z['lid_faces'],auto_check=False)
 return cpt.FloatingBody(mesh=mesh,lid_mesh=lid,dofs=cpt.rigid_body_dofs(rotation_center=CG),center_of_mass=CG,mass=8431800/S**3)
def incident(points,omega,beta,U):
 """Canonical exp(-i*|encounter|t) representation for either signed branch."""
 k=omega**2/G;nu=omega-U*k*math.cos(beta);sgn=1 if nu>=0 else -1
 points=np.asarray(points);phase=np.exp(k*points[:,2]+1j*sgn*k*(points[:,0]*math.cos(beta)+points[:,1]*math.sin(beta)))
 potential=-1j*sgn*G/omega*phase
 velocity=potential[:,None]*np.array([1j*sgn*k*math.cos(beta),1j*sgn*k*math.sin(beta),k])[None,:]
 pressure=1j*abs(nu)*RHO*potential+RHO*U*velocity[:,0]
 error=float(np.max(abs(pressure-RHO*G*phase))/max(np.max(abs(RHO*G*phase)),1e-30))
 return velocity,pressure,error,nu
def filename(job):return 'hydro_cache/'+job['id']+'.json'
def worker(job):
 c=read(P/'protocol.json');assert sha(P/'protocol.json')==c['self_reference_excluded_sha'] if 'self_reference_excluded_sha' in c else True
 start=time.perf_counter();b=body(job['stride']);U=job['U']/math.sqrt(S)
 foils=read(O/'dtmb_lateral_provider_v1/provider_contract.json')['foils'];pts=np.array([f['r'] for f in foils])/S+CG
 solver=cpt.BEMSolver();forces=[];velocities=[]
 with threadpool_limits(2):
  if job['kind']=='rad':
   omega=job['nu']*math.sqrt(S)
   for dof in DOFS:
    pb=cpt.RadiationProblem(body=b,omega=omega,forward_speed=U,wave_direction=math.pi/2,rho=RHO,g=G,radiating_dof=dof)
    result=solver.solve(pb,keep_details=True);v=solver.compute_velocity(pts,result);v[:,0]+=U
    factor=np.array([S**2,S**2,S**3,S**3])*(S if dof in ['Roll','Yaw'] else 1)
    forces.append(np.array([result.forces[d] for d in DOFS])*factor);velocities.append(v*(math.sqrt(S) if dof in ['Roll','Yaw'] else 1/math.sqrt(S)))
   raw=dict(R=pack(np.array(forces).T),radiation_velocity=pack(velocities),canonical_frequency_rad_s=job['nu'])
  else:
   omega=job['w']*math.sqrt(S);beta=math.radians(180-job['mu'])
   pb=cpt.DiffractionProblem(body=b,omega=omega,forward_speed=U,wave_direction=beta,rho=RHO,g=G)
   vel,pressure,error,nu=incident(b.mesh.faces_centers,omega,beta,U)
   # The native3.0 negative-encounter Airy velocity is not the gradient of its
   # potential. Supply the explicit canonical gradient, tested by pressure identity.
   pb.boundary_condition[:]=0
   pb.boundary_condition[b.hull_mask]=-np.einsum('ij,ij->i',vel,b.mesh.faces_normals)
   result=solver.solve(pb,keep_details=True)
   incident_forces=b.integrate_pressure(pressure)
   factor=np.array([S**2,S**2,S**3,S**3]);force=np.array([incident_forces[d]+result.forces[d] for d in DOFS])*factor
   vw=solver.compute_velocity(pts,result);vw[:,0]+=U
   inc,_,e2,_=incident(pts,omega,beta,U);vw=(vw+inc)/math.sqrt(S)
   raw=dict(F=pack(force),wave_velocity=pack(vw),signed_encounter_rad_s=nu/math.sqrt(S),incident_pressure_relative_error=max(error,e2))
   assert max(error,e2)<1e-11
 for key in ['R','radiation_velocity','F','wave_velocity']:
  if key in raw and not np.isfinite(np.asarray(raw[key])).all():raise ValueError('Nonfinite hydro input '+key)
 raw.update(job=job,protocol_sha256=sha(P/'protocol.json'),wall_seconds=time.perf_counter()-start,capytaine_version=cpt.__version__)
 save(filename(job),raw);print('Hydro completed',job['id'],round(raw['wall_seconds'],2),'s',flush=True)
 return raw
def verify_inputs():
 c=read(P/'protocol.json')
 hashes=dict(c['hashes'])
 err=P/'implementation_erratum_01.json'
 if err.exists():hashes.update(read(err)['replacement_hashes'])
 for q,v in hashes.items():assert sha(q)==v,q
 return c
def run_job(job):
 verify_inputs();q=P/filename(job)
 if q.exists():
  raw=read(q);assert raw['job']==job and raw['protocol_sha256']==sha(P/'protocol.json');return raw
 return worker(job)
