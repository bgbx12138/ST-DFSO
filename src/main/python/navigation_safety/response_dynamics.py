"""Equivalent-linear roll and fixed linear horizontal response under sampled forcing."""
import math
import time
import numpy as np
from scipy.optimize import brentq

VERSION = 'stdfso_equivalent_linear_response_v1'


def group_moment_spectrum(w, weights, encounter, cosine, gamma, D, tau):
    """Variance weights of .5*d|analytic orbital velocity|^2/dt.

    Independent Gaussian wave amplitudes have E|z_j|^2=2*weight_j.
    A j<k pair has variance 2*(nu_j-nu_k)^2*w_j*w_k*(W_j*W_k*cos^2)^2.
    Zero difference frequency has exactly zero derivative (not a cutoff).
    """
    j, k = np.triu_indices(len(w), 1)
    delta = np.abs(encounter[j] - encounter[k])
    take = delta > 0
    j, k, delta = j[take], k[take], delta[take]
    power = (2 * delta**2 * weights[j] * weights[k]
             * (w[j] * w[k] * cosine**2)**2
             * (gamma * D)**2 / (1 + (delta * tau)**2))
    return delta, power


def equivalent_roll_damping(A, frequencies, force_variances):
    """Unique nonnegative Gaussian closure; bracketed scalar solve."""
    nu, power = np.asarray(frequencies), np.asarray(force_variances)
    alpha, beta = A['alpha'], A['beta']
    if alpha <= 0 or beta < 0 or A['I'] <= 0 or A['C'] <= 0:
        raise ValueError('Invalid roll inertia/restoring/damping coefficients')
    def rate_sd(B):
        return np.sqrt(np.sum(nu**2 * power / ((A['C'] - A['I'] * nu**2)**2 + (B*nu)**2)))
    c = math.sqrt(8/math.pi)*beta
    upper = alpha + c*rate_sd(alpha)
    B = alpha if upper == alpha else brentq(
        lambda b: b-alpha-c*rate_sd(b), alpha, upper,
        xtol=1e-7, rtol=1e-12)
    residual = abs(B-alpha-c*rate_sd(B))/max(B, alpha)
    if residual > 1e-10:
        raise ArithmeticError('Equivalent roll damping closure failed')
    return float(B), float(residual)


class SpectralResponse:
    """Use SafetyFunction only as an asset loader, never its response evaluator."""
    def __init__(self, loader):
        self.base = loader._base
        self.adapter = self.base._ADAPTER
        self.e, self.prop, _ = self.adapter.engine()
        self.roll = self.e.m.roll
        self.weights = np.asarray(loader._contract['weights'])
        self.scales = np.asarray(loader._contract['reference_scales'])
        self.control = dict(P=-.8, D_s=-24.9)
        self.workpoint = self.adapter.read(self.adapter.O/'r1_following_surge_diagnostic_v1/short_preflight/nominal_workpoint.json')

    def transfer(self, profile, U, nu):
        if np.any(nu <= 0):
            raise ValueError('Positive frequencies required; DC is handled separately')
        e, p = self.e, self.prop
        B = e.m.damping(U, np.zeros(3))
        tr, _ = p.replace_operator(e.g, profile, U, nu, B, self.control, self.workpoint)
        tr, _ = self.base._LOW_FREQUENCY.complete(e, p, profile, U, nu, B, self.control, self.workpoint, tr)
        # Old asset loader caches by full frequency array. Queries here have
        # different pair frequencies, so retaining every array leaks memory.
        e.m._RAD_CACHE.clear()
        t = tr.conj()  # positive-time Fourier convention
        H = np.stack([1j*nu[:,None]*t[:,0],
                      1j*nu[:,None]*t[:,1]-U*t[:,2],
                      t[:,2], 1j*nu[:,None]*t[:,2]], axis=1)
        if not np.isfinite(H).all():
            raise ArithmeticError('Nonfinite horizontal transfer')
        return H



class ForcedLinearResponse(SpectralResponse):
    """Second candidate: retain sampled quadratic forcing and roll-wave coupling.

    This is a hybrid frequency-domain model, not a purely first-order RAO model.
    Uses an isolated, single-threaded diagnostic process; temporary adapter hooks
    are restored even on failure. Never installs a production response provider.
    """

    def evaluate(self, query, dt=.125, seed=20261018, roll_mode='equivalent',closure='zero'):
        if dt not in (.05,.125,.25):
            raise ValueError('Use one of the tested temporal grids')
        start, cpu = time.perf_counter(), time.process_time()
        a, e = self.adapter, self.e
        old_dt, old_context = a.DT, a._LAST_CONTEXT
        old_trace, old_bridge = e.m.frozen_trace, e.m.fast_bridge.trace
        diagnostic = {}
        def roll_trace(force, step, A):
            n = len(force)
            nu = 2*np.pi*np.fft.rfftfreq(n,d=step)
            ff = np.fft.rfft(force)
            power = abs(ff/n)**2
            power[1:-1] *= 2
            power[0] = 0
            B, residual = equivalent_roll_damping(A,nu,power)
            phi_hat = ff/(A['C']-A['I']*nu**2+1j*B*nu)
            phi_hat[0] = 0
            diagnostic.update(roll_damping=B,roll_closure_relative_residual=residual)
            return np.fft.irfft(phi_hat,n)
        try:
            a.DT = dt
            a._LAST_CONTEXT = None
            e.m.frozen_trace = lambda weights,U,mu,s: old_trace(weights,U,mu,s,dt=dt)
            if roll_mode == 'equivalent':
                e.m.fast_bridge.trace = roll_trace
            elif roll_mode != 'original':
                raise ValueError('Unknown roll mode')
            prepared = a.prepare_context(query)
            raw, context = a._input(prepared, seed)
        finally:
            a.DT, a._LAST_CONTEXT = old_dt, old_context
            e.m.frozen_trace, e.m.fast_bridge.trace = old_trace, old_bridge
        nuall = raw['nuall']
        mask = (nuall>=.0001)&(nuall<=12)
        nu = nuall[mask]
        H = self.transfer(prepared['profile'],float(query['U_m_s']),nu)
        solution = np.einsum('nki,ni->nk',H,raw['forcing_fft'][mask])
        linear_solution=solution.copy()
        if closure != 'zero':
            raise ValueError('Only the fixed linear response is supported')
        ff = np.zeros((len(nuall),4),complex)
        ff[mask] = solution
        response = np.fft.irfft(ff.T,int(raw['n']),axis=1)
        variance = np.var(response[:,int(raw['drop']):],axis=1)
        sd = np.array([context['roll_SD_deg'],np.sqrt(variance[2])*180/np.pi,
                       np.sqrt(variance[1]),np.sqrt(variance[0])])
        if not np.isfinite(sd).all():
            raise ArithmeticError('Nonfinite hybrid response')
        # A posteriori discrepancy from the original nonlinear horizontal
        # equation is reported, not disguised as solver residual convergence.
        U = float(query['U_m_s'])
        nonlinear_force = e.nl.forces(response,U,e.CFG)
        nonlinear_hat = np.fft.rfft(nonlinear_force,axis=1).T
        increment = np.zeros_like(ff)
        increment[mask] = np.einsum('nki,ni->nk',H[:,:,:3],nonlinear_hat[mask])
        correction = np.fft.irfft(increment.T,int(raw['n']),axis=1)
        initial_fft=np.zeros_like(ff);initial_fft[mask]=linear_solution
        initial_response=np.fft.irfft(initial_fft.T,int(raw['n']),axis=1)
        discrepancy=initial_response+correction-response
        rms = np.sqrt(np.mean(discrepancy**2,axis=1)) / np.maximum(np.sqrt(np.mean(response**2,axis=1)),1e-15)
        return dict(model='stdfso_fixed_linear_response_v1',input=dict(query),sd=sd.tolist(),
                    cost=float(np.sum(self.weights*sd/self.scales)),seconds=time.perf_counter()-start,
                    cpu_seconds=time.process_time()-cpu,dt=dt,seed=seed,roll_mode=roll_mode,
                    record_seconds=8192,discard_seconds=512,**diagnostic,
                    original_nonlinear_equation_discrepancy_rms=rms.tolist(),
                    status='OK')
