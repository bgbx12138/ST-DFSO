"""First-order wave-induced heading variance; excludes slow course-keeping excursions."""
import math
import numpy as np
from .response_dynamics import equivalent_roll_damping, group_moment_spectrum


def wave_heading_statistics(model, query, roll_damping=None, group_refinement=1):
    r=model.roll;U=float(query['U_m_s']);mu=float(query['mu_deg'])
    model.adapter.angular_weights(mu)
    if not np.isfinite(U) or U<=0:raise ValueError('Positive water speed required')
    anchor=query.get('source_anchor_case')
    if anchor is not None:
        anchor=int(anchor);profile='s'+str(anchor)
        if anchor not in (0,45,90,180) or mu!=anchor or abs(U-model.e.m.C['profiles'][profile])>1e-10:
            raise ValueError('Source heading and conditioning speed must match')
        weights=r.DATA['case'+str({0:215,45:205,90:206,180:218}[anchor])]
    else:
        if not .95*model.adapter.U_ROUTE<=U<=1.05*model.adapter.U_ROUTE:
            raise ValueError('Outside inherited nominal speed domain')
        profile='route';weights,_=r.spectrum(float(query['Hm0_m']),float(query['Tp_s']))
    sel=np.flatnonzero(weights>0);w=r.W[sel];weights=weights[sel]
    if not len(sel) or w.min()<model.e.m.WW[0]-1e-12 or w.max()>model.e.m.WW[-1]+1e-12:
        raise ValueError('No supported input wave spectrum')
    cosine=0. if mu==90 else math.cos(math.radians(mu))
    signed=w+U*cosine*w*w/9.81;nu=np.abs(signed)
    direct=r.direct_port(mu)[sel]
    moment=r.PAR['Gamma']*np.where(signed>=0,direct.conj(),direct)
    supplied_damping=roll_damping is not None
    if roll_damping is None:
        if group_refinement not in (1,2,4):raise ValueError('Group refinements: 1,2,4')
        # Only the scalar roll group-damping quadrature is compressed. Keep all
        # 6570 source wave nodes for the horizontal transfer and heading moment.
        if cosine==0:
            gn,gp=np.array([]),np.array([])
        else:
            gw=np.linspace(w[0],w[-1],128*group_refinement+1)
            density=np.interp(gw,w,weights/r.TW[sel])
            tw=np.r_[np.diff(gw)[0]/2,(gw[2:]-gw[:-2])/2,np.diff(gw)[-1]/2]
            gweights=tw*density;gweights*=weights.sum()/gweights.sum()
            gnu=abs(gw+U*cosine*gw*gw/9.81)
            gn,gp=group_moment_spectrum(gw,gweights,gnu,cosine,r.PAR['Gamma'],r.LEGACY['D_effective'],r.LEGACY['tau_effective_s'])
        roll_damping,_=equivalent_roll_damping(r.A,np.r_[nu,gn],np.r_[weights*abs(moment)**2,gp])
    gain=math.exp(-r.LEGACY['frozen_kappa']*.5*cosine*cosine*(1-cosine))
    phi=gain*moment/(r.A['C']-r.A['I']*nu**2+1j*roll_damping*nu)
    F=model.adapter._wave_external(profile,U,mu,w).conj().copy()
    centres=np.array([[(.5-.937250345)*142+.683,side*.03274648*142,-.03601247*142] for side in (-1,1)])
    beta=math.radians(180-mu);k=w*w/9.81
    phase=centres[:,0,None]*math.cos(beta)+centres[:,1,None]*math.sin(beta)
    orbital=(w[None,:]*math.cos(beta)*np.exp(k[None,:]*centres[:,2,None]-1j*k[None,:]*phase)).mean(axis=0)
    orbital=np.where(signed>=0,orbital,orbital.conj())
    tangent=model.prop.tangent(U,model.workpoint['n_rps_full'],model.workpoint['wake'],model.workpoint['thrust_deduction'])
    F[:,0]+=tangent['B_P_N_s_m']/(1-model.workpoint['wake'])*orbital
    dynamic=nu>0
    H=model.transfer(profile,U,nu[dynamic])
    forcing=np.column_stack([F,phi])
    heading=np.einsum('ni,ni->n',H[:,2],forcing[dynamic])
    variance=float(np.sum(weights[dynamic]*abs(heading)**2))
    sd=math.sqrt(variance)*180/math.pi
    return dict(wave_induced_heading_sd_deg=sd,frequency_nodes=len(w),roll_damping=roll_damping,
                roll_damping_from_supplied_trace=supplied_damping,
                roll_group_quadrature_nodes=128*group_refinement+1,
                heading_RAO_deg_per_m=(abs(heading)*180/math.pi).tolist(),
                intrinsic_rad_s=w[dynamic].tolist(),encounter_rad_s=nu[dynamic].tolist(),
                wave_variance_weights=weights[dynamic].tolist(),
                definition='First-order wave excitation plus first-order prescribed-roll coupling, integrated on the intrinsic wave grid',
                excluded='Quadratic wave/roll excitation, slow control excursions and between-run offsets; not full heading SD',
                finite_frequency_cutoff_applied=False,zero_encounter_static_offsets_excluded=True,
                exact_intrinsic_to_encounter_mapping=True)
