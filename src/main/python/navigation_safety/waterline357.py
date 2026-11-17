"""Unvalidated, mathematically equivalent periodic waterline implementation.

The original N-point forcing has integer signed Fourier support [-B, B].
Its real surface elevation squared has support [-2B, 2B]. Compute the same
824-point geometrical integral on M > 4B equally spaced points over that
SAME period, then reconstruct its complete N-point record. Nothing is fitted,
no physical quadrature point is removed, and no response is interpolated.

Floating point roundoff differs from the original N-point FFTs. Bitwise
forcing equality is NOT claimed; real-input validation is still required.
Import/install never starts processes or evaluates the model.
"""
from pathlib import Path
import hashlib
import math
import time


LAST_DIAGNOSTICS = None
_POLICY = 'EXACT_PERIODIC_WATERLINE_BANDLIMIT_V1'


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _plan(np, next_fast_len, z, w, zeta, bins, sel, mu, n):
    """Return a strict alias-free plan, or defer nonstandard input untouched."""
    if isinstance(n, bool) or not isinstance(n, (int, np.integer)) or n <= 0:
        return None, 'NONSTANDARD_RECORD_LENGTH'
    n = int(n)
    if not np.isfinite(mu):
        return None, 'NONFINITE_DIRECTION'
    w_array, amplitude, index, selected = (np.asarray(x) for x in (w, zeta, bins, sel))
    if (w_array.ndim != 1 or amplitude.shape != w_array.shape or index.shape != w_array.shape
            or selected.ndim != 1 or not np.issubdtype(index.dtype, np.integer)
            or not np.issubdtype(selected.dtype, np.integer)):
        return None, 'NONSTANDARD_SPARSE_SPECTRUM'
    if len(selected) == 0:
        return None, 'EMPTY_SELECTED_SPECTRUM'
    if selected.min() < 0 or selected.max() >= len(w_array):
        return None, 'SELECTED_INDEX_OUTSIDE_SPECTRUM'
    occupied = index[selected]
    if occupied.min() < 0 or occupied.max() >= n:
        return None, 'BIN_OUTSIDE_ORIGINAL_RECORD'
    coords, weights = np.asarray(z['wl']), np.asarray(z['wl_g'])
    if (coords.ndim != 2 or coords.shape[1] != 3 or weights.shape != coords.shape
            or not np.isfinite(coords).all() or not np.isfinite(weights).all()
            or not np.isfinite(w_array[selected]).all() or not np.isfinite(amplitude[selected]).all()):
        return None, 'NONSTANDARD_GEOMETRY_OR_VALUES'
    # Preserve the exact integer frequencies of the original common period.
    # If the original Nyquist is occupied, M cannot be smaller and falls back.
    signed = np.where(occupied <= n//2, occupied, occupied-n).astype(np.int64, copy=False)
    bound = int(np.max(np.abs(signed)))
    if bound == 0:
        return None, 'DC_ONLY_USE_ORIGINAL_ROUNDOFF_PATH'
    internal = int(next_fast_len(4*bound+1, real=True))
    if internal >= n:
        return None, 'NO_SMALLER_ALIAS_FREE_GRID'
    if internal <= 4*bound:
        raise RuntimeError('WATERLINE357_FAST_LENGTH_DID_NOT_RESOLVE_SQUARED_SUPPORT')
    return dict(n=n, m=internal, bound=bound, signed=signed,
                coords=coords, weights=weights), None


def _evaluate(base, np, plan, zeta, w, sel, mu):
    n, m, bound = plan['n'], plan['m'], plan['bound']
    coords, weights = plan['coords'], plan['weights']
    beta = math.radians(180-mu)
    k = w[sel]**2/9.81
    short_bins = plan['signed'] % m
    short_force = np.zeros((3, m))
    # Same original order and 8-point batches; all quadrature points retained.
    for start in range(0, len(coords), 8):
        pts = coords[start:start+8]
        phase = pts[:, 0]*math.cos(beta)+pts[:, 1]*math.sin(beta)
        amp = zeta[sel][None, :]*np.exp(-1j*phase[:, None]*k[None, :])
        ft = np.zeros((len(pts), m), complex)
        np.add.at(ft, (np.arange(len(pts))[:, None], short_bins[None, :]), amp)
        eta = (np.fft.ifft(ft, axis=1)*m).real
        short_force += (-.5*base.h.RHO*9.81)*(weights[start:start+8].T@(eta*eta))
    if mu in [0, 180]:
        short_force[1:] = 0
    if mu == 90:
        short_force[0] = 0
    # Only zero-frequency subtraction, exactly the original centering operation.
    short_force -= short_force.mean(axis=1, keepdims=True)
    normalized = np.fft.rfft(short_force, axis=1)/m
    # The squared signal has NO coefficients beyond 2B in exact arithmetic.
    # Dropping only roundoff outside that proven support avoids interpreting an
    # M-grid Nyquist roundoff term as a new physical mode on the N-grid.
    full_spectrum = np.zeros((3, n//2+1), complex)
    stop = 2*bound+1
    full_spectrum[:, :stop] = normalized[:, :stop]
    full_force = np.fft.irfft(full_spectrum, n=n, axis=1)*n
    full_force -= full_force.mean(axis=1, keepdims=True)
    return full_force


def install(base):
    """Replace only e.base.geometry_waterline_force in this loaded process.

    Diagnostics are available as base._waterline357_last and this module's
    LAST_DIAGNOSTICS. Original callable is retained as
    base._waterline357_original. A successor must bind this source hash in
    its numerical implementation identity and seal before scientific use.
    """
    patch_sha = _sha(__file__)
    installed = getattr(base, '_waterline357_sha256', None)
    if installed is not None:
        if installed != patch_sha:
            raise RuntimeError('WATERLINE357_DIFFERENT_IMPLEMENTATION_ALREADY_INSTALLED')
        return dict(installed=True, source_sha256=patch_sha, validated=False)
    original = base.geometry_waterline_force
    original_source = Path(original.__code__.co_filename)
    original_sha = _sha(original_source)

    def evaluate(z, w, zeta, bins, sel, mu, n):
        global LAST_DIAGNOSTICS
        import numpy as np
        from scipy.fft import next_fast_len
        started = time.monotonic()
        plan, rejection = _plan(np, next_fast_len, z, w, zeta, bins, sel, mu, n)
        if plan is None:
            value = original(z, w, zeta, bins, sel, mu, n)
            proof = dict(policy=_POLICY, accelerated=False, fallback_reason=rejection,
                         source_sha256=patch_sha, original_source_sha256=original_sha,
                         original_n=int(n) if isinstance(n, (int, np.integer)) else None,
                         internal_m=None, elapsed_seconds=time.monotonic()-started,
                         bitwise_identity_claimed=False, validated=False)
        else:
            value = _evaluate(base, np, plan, zeta, w, sel, mu)
            proof = dict(policy=_POLICY, accelerated=True,
                         source_sha256=patch_sha, original_source_sha256=original_sha,
                         original_n=plan['n'], internal_m=plan['m'],
                         input_signed_bin_bound=plan['bound'],
                         squared_signed_bin_bound=2*plan['bound'],
                         strict_no_alias=bool(plan['m'] > 4*plan['bound']),
                         original_common_period_retained=True,
                         full_output_record_retained=True,
                         all_geometry_points_retained=len(plan['coords']),
                         geometry_batch_size=8, centered=True,
                         physical_equations_changed=False,
                         bitwise_identity_claimed=False,
                         elapsed_seconds=time.monotonic()-started,
                         validation='PENDING_REAL_INPUT_FORCING_SD_COST_COMPARISON',
                         validated=False)
        LAST_DIAGNOSTICS = proof
        base._waterline357_last = proof
        return value

    base._waterline357_original = original
    base.geometry_waterline_force = evaluate
    base._waterline357_sha256 = patch_sha
    base._waterline357_last = None
    return dict(installed=True, source_sha256=patch_sha,
                original_source_sha256=original_sha, validated=False)
