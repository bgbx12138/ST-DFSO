# -*- coding: utf-8 -*-
"""Finite-coefficient completion below the existing radiation table.

Lu, Gu & Boulougouris (2023), JMSE 11:1555, sec. 2.2/2.6,
eqs. (2),(4),(34),(41): horizontal manoeuvring inertia must be finite at
near-zero encounter frequency; divergent seakeeping added masses are not
used as manoeuvring coefficients. This implementation uses the existing
5415M table endpoint, not the ONRT vessel or coefficients in that paper.

Chen et al. (2026), 41st IWWWFB, Analysis of ship seakeeping in following
seas, eqs. (13),(14): separate static/restoring terms before forming a
zero-frequency inertia limit. Here the existing heading feedback is removed
before completing the body-velocity inertia and is restored exactly once.

The endpoint-constant mass continuation is an explicit interpolation
assumption of this component. It is not an exact reproduction of either
paper's full hydrodynamic solver. No response observation is used here.
"""
import numpy as np


def body_velocity_map(nu, U):
    nu=np.atleast_1d(nu)
    out=np.zeros((len(nu),3,3),complex)
    out[:,0,0]=out[:,1,1]=out[:,2,2]=-1j*nu
    out[:,1,2]=-U
    return out


def complete(e, prop, profile, U, nu, B, controller, workpoint, original):
    """Return the original transfer with only nu < first table node completed."""
    nu=np.asarray(nu)
    lower=float(e.m.NU[0])
    take=nu<lower
    if not take.any():
        return original.copy(),dict(applied=False,lower_rad_s=lower)
    if np.any(nu<=0):raise ValueError('Positive encounter frequency required')
    base,_=prop.replace_operator(e.g,profile,U,np.array([lower]),B,controller,workpoint)
    Z=np.linalg.inv(base[0,:,:3])
    b=e.g.c.matrices(profile,U,np.array([lower]),B)[2]
    Tv=body_velocity_map(np.array([lower]),U)[0]
    Kv=controller['P']-1j*lower*controller['D_s']
    Zopen=Z.copy();Zopen[:,2]+=b*Kv
    Zv=Zopen@np.linalg.inv(Tv)
    mass=-Zv.imag/lower
    velocity=Zv.real
    if not np.isfinite(mass).all():raise ValueError('Nonfinite endpoint inertia')
    # Inertia is not symmetrised or adjusted to satisfy a response target.
    # Non-positive endpoint principal inertias are an input failure.
    if np.any(np.diag(mass)<=0):raise ValueError('Nonpositive endpoint inertia')
    frequencies=nu[take]
    vl=body_velocity_map(frequencies,U)
    zl=(velocity[None]-1j*frequencies[:,None,None]*mass[None])@vl
    feedback=controller['P']-1j*frequencies*controller['D_s']
    zl[:,:,2]-=b[None,:]*feedback[:,None]
    inv=np.linalg.inv(zl)
    # Preserve the original prescribed-roll coupling force at each queried
    # frequency. Only the horizontal dynamic impedance is completed.
    oldz=np.linalg.inv(original[take,:,:3])
    zphi=-np.einsum('nij,nj->ni',oldz,original[take,:,3])
    out=original.copy();out[take,:,:3]=inv
    out[take,:,3]=-np.einsum('nij,nj->ni',inv,zphi)
    reconstructed=(velocity-1j*lower*mass)@Tv
    reconstructed[:,2]-=b*Kv
    scale=np.diag([e.m.L,e.m.L,1.])
    mismatch=np.linalg.norm((reconstructed-Z)@np.linalg.inv(scale))/np.linalg.norm(Z@np.linalg.inv(scale))
    if mismatch>1e-11:raise ValueError('Endpoint completion identity failed')
    return out,dict(applied=True,lower_rad_s=lower,completed_bins=int(take.sum()),
                    endpoint_identity_relative_error=float(mismatch),
                    endpoint_mass=mass.tolist(),endpoint_velocity_coefficient=velocity.tolist(),
                    heading_feedback_unchanged=dict(controller),
                    high_frequency_transfer_byte_identical=bool(np.array_equal(out[~take],original[~take])),
                    roll_force_coupling_retained=True,
                    assumption='Constant finite body-velocity inertia and velocity coefficients below first radiation node',
                    sources=['https://doi.org/10.3390/jmse11081555',
                             'https://iwwwfb2026.fsb.hr/documents/acceptedabstracts/IWWWFB41_05.pdf'])
