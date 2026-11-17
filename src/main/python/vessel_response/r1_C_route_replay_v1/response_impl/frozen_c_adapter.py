# -*- coding: utf-8 -*-
"""Cached hydrodynamic inputs and wave/roll forcing for the response model."""
from pathlib import Path
import atexit
import hashlib
import json
import math
import os
import struct
import subprocess
import sys
import time
import uuid

sys.dont_write_bytecode = True
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
for _name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_name, '2')

P = Path(__file__).resolve().parent
STAGE = P.parent
O = STAGE.parent
NODES = (0, 45, 90, 180)
DT = 0.05
DURATION = 8192.0
DROP = 512.0
U_ROUTE = 0.248 * math.sqrt(9.81 * 142.0)
_ENGINE = None
_BRIDGE = None
_FOILS = None
_GEOMETRY = None
_CROSS = {}
_CACHE_MANIFEST = {}
_WAVE_NODE = {}
_LAST_CONTEXT = None


class MissingFrozenInput(RuntimeError):
    pass


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def cache_file(path):
    path = Path(path)
    if not path.is_file():
        raise MissingFrozenInput('Required cached input absent; no solver fallback: ' + str(path))
    if str(path) not in _CACHE_MANIFEST:
        _CACHE_MANIFEST[str(path)] = sha(path)
    return path


def angular_weights(mu):
    if not math.isfinite(mu) or not 0 <= mu <= 180:
        raise ValueError('mu_deg must be head=0 through beam=90 to following=180')
    if mu in NODES:
        return [(int(mu), 1.0)]
    hi = next(j for j in range(1, 4) if NODES[j] > mu)
    lo = hi - 1
    weight = (mu - NODES[lo]) / (NODES[hi] - NODES[lo])
    return [(NODES[lo], 1.0 - weight), (NODES[hi], weight)]


class RollBridge:
    """Identical precompiled RK4 bridge; logs never touch the source directory."""
    def __init__(self, folder):
        self.folder = Path(folder)
        self.process = None
        self.log = None

    def trace(self, force, dt, anchor):
        import numpy as np
        if self.process is None:
            self.folder.mkdir(parents=True, exist_ok=True)
            self.log = (self.folder / 'roll_bridge.stderr.log').open('xb')
            classes = next(q for q in O.parents if (q/'pom.xml').is_file()) / 'target/classes'
            cache_file(classes / 'FrozenRollBridge.class')
            self.process = subprocess.Popen(
                ['java', '-cp', str(classes), 'FrozenRollBridge'],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log,
                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        n = len(force)
        self.process.stdin.write(struct.pack('>i5d', n, dt, anchor['I'], anchor['C'],
                                              anchor['alpha'], anchor['beta']))
        self.process.stdin.write(np.asarray(force, dtype='>f8').tobytes())
        self.process.stdin.flush()
        remaining = n * 8
        parts = []
        while remaining:
            data = self.process.stdout.read(remaining)
            if not data:
                raise RuntimeError('Frozen roll bridge stopped; see isolated stderr log')
            parts.append(data)
            remaining -= len(data)
        return np.frombuffer(b''.join(parts), dtype='>f8').astype(float)

    def close(self):
        if self.process is not None:
            try:
                self.process.stdin.write(struct.pack('>i', -1))
                self.process.stdin.flush()
                self.process.wait(timeout=5)
            except Exception:
                self.process.terminate()
            self.process = None
        if self.log is not None:
            self.log.close()
            self.log = None


def close():
    global _BRIDGE
    if _BRIDGE is not None:
        _BRIDGE.close()
        _BRIDGE = None


atexit.register(close)


def engine():
    global _ENGINE, _BRIDGE
    if _ENGINE is None:
        # Imports are read-only. Old executable entry points are never called.
        sys.path.insert(0, str(O / 'r1_effective_control_v1'))
        import effective_model as e
        if Path(e.base.__file__).resolve().parent != O / 'r1_fk_cross_closure_v1':
            raise RuntimeError('Legacy module-name collision; run C adapter in a clean worker process')
        sys.path.insert(0, str(O / 'r1_constant_rpm_closure_v1'))
        import propulsion as prop
        # Disable every known entry to the old hydrodynamic solve path. The
        # model's radfields/wavefields functions themselves only read caches.
        def forbidden(*args, **kwargs):
            raise MissingFrozenInput('New hydrodynamic solve forbidden in frozen C replay')
        e.h.worker = forbidden
        e.h.run_job = forbidden
        e.h.cpt.BEMSolver = forbidden
        _BRIDGE = RollBridge(P / ('runtime_' + str(os.getpid()) + '_' + uuid.uuid4().hex[:10]))
        e.m.fast_bridge.trace = _BRIDGE.trace
        _ENGINE = (e, prop, None)
    return _ENGINE




def cache_inventory():
    """No solving; prove the exact cache set exists before starting long work."""
    e, _, _ = engine()
    count = 0
    for profile in ('s0', 's45', 's90', 's180', 'route'):
        for j in range(len(e.m.NU)):
            cache_file(e.m.P / ('hydro_cache/r_%s_%02d_s4.json' % (profile, j)))
            count += 1
    for profile in ('source', 'route'):
        for node in NODES:
            tag = ('s' if profile == 'source' else 'route') + str(node)
            for j in range(len(e.m.WW)):
                cache_file(e.m.P / ('hydro_cache/w_%s_%02d_s4.json' % (tag, j)))
                count += 1
    for node in NODES:
        cache_file(e.FOLDER / ('cross_gauss_s2_mu%d.npz' % node))
    cache_file(e.FOLDER / 'quadrature_geometry_s2.npz')
    return dict(hydro_cache_files=count, new_hydro_solves=0,
                radiation_profiles=list(e.m.C['profiles']), angle_nodes=list(NODES),
                intrinsic_frequency_support_rad_s=[float(e.m.WW[0]), float(e.m.WW[-1])],
                route_water_speed_m_s=U_ROUTE)


def _native_route_node(node):
    import numpy as np
    e, _, _ = engine()
    if node not in _WAVE_NODE:
        data = [read(cache_file(e.m.P / ('hydro_cache/w_route%d_%02d_s4.json' % (node, j))))
                for j in range(len(e.m.WW))]
        result = []
        negative = np.asarray([x['signed_encounter_rad_s'] < 0 for x in data])
        for name in ('F', 'wave_velocity'):
            array = np.asarray([e.h.unpack(x[name]) for x in data])
            array[negative] = array[negative].conj()
            result.append(array)
        _WAVE_NODE[node] = result
    return _WAVE_NODE[node]


def _wave_external(profile, U, mu, w):
    import numpy as np
    e, _, _ = engine()
    if profile != 'route':
        return e.m.wave_external(profile, int(mu), U, w)
    # Common intrinsic-frequency/native convention, never interpolate across a
    # positive-frequency conjugacy switch. Foil flow and hull force share it.
    node_values = [(a, _native_route_node(node)) for node, a in angular_weights(mu)]
    force_nodes = sum(a * value[0] for a, value in node_values)
    velocity_nodes = sum(a * value[1] for a, value in node_values)
    force = e.m.interp_complex(e.m.WW, force_nodes, w)
    velocity = e.m.interp_complex(e.m.WW, velocity_nodes, w)
    co = 0.0 if mu == 90 else math.cos(math.radians(mu))
    negative = w + U * co * w * w / 9.81 < 0
    force[negative] = force[negative].conj()
    velocity[negative] = velocity[negative].conj()
    for j, foil in enumerate(e.m.FOILS):
        pos, normal = np.asarray(foil['r']), np.asarray(foil['n'])
        generalized = np.r_[normal[:2], np.cross(pos, normal)[[0, 2]]]
        lift = 0.5 * e.h.RHO * U * foil['area'] * foil['lift_slope_per_rad']
        force += lift * (velocity[:, j] @ normal)[:, None] * generalized
    return force[:, e.m.Q]


def _cross_coefficients(mu, w):
    import numpy as np
    e, _, _ = engine()
    values = []
    for node, weight in angular_weights(mu):
        if node not in _CROSS:
            with np.load(cache_file(e.FOLDER / ('cross_gauss_s2_mu%d.npz' % node))) as raw:
                _CROSS[node] = (raw['w'].copy(), raw['C'].copy())
        grid, coeff = _CROSS[node]
        if not np.array_equal(grid, w):
            raise MissingFrozenInput('Cross-input frozen frequency support differs; no extrapolation')
        values.append(weight * coeff)
    return sum(values)


def prepare_context(data):
    """Prepare a single state once. Only the most recent state is retained.

    This object contains numpy arrays; use within one worker, not as a JSON
    checkpoint. The checkpoint should retain the original input dictionary.
    """
    global _LAST_CONTEXT
    import numpy as np
    e, prop, _ = engine()
    if data.get('_prepared_C_context') is True:
        return data
    data = dict(data)
    anchor = data.get('source_anchor_case')
    mu = float(data['mu_deg'])
    angular_weights(mu)
    U = float(data['U_m_s'])
    if not math.isfinite(U) or U <= 0:
        raise ValueError('Positive through-water U_m_s required')
    key = (anchor, mu, U, data.get('Hm0_m'), data.get('Tp_s'), data.get('signed_mu_deg'))
    if _LAST_CONTEXT is not None and _LAST_CONTEXT['cache_key'] == key:
        # Retain numerical arrays only. Caller route/id/timestamp provenance
        # must describe this invocation even when the physical state repeats.
        return dict(_LAST_CONTEXT, original_input=data)
    if anchor is not None:
        anchor = int(anchor)
        if anchor not in NODES or mu != anchor:
            raise ValueError('Source anchor must be exactly one of 0/45/90/180')
        profile = 's' + str(anchor)
        if abs(U - e.m.C['profiles'][profile]) > 1e-10:
            raise ValueError('Source anchor must use its observed conditioning speed')
        weights = e.m.roll.DATA['case' + str({0: 215, 45: 205, 90: 206, 180: 218}[anchor])]
        retained = 1.0
    else:
        # Frozen nominal-speed BEM data are a local coefficient approximation.
        # All explicit speed-dependent terms below use the requested water speed.
        if not .95 * U_ROUTE <= U <= 1.05 * U_ROUTE:
            raise MissingFrozenInput('Outside declared +/-5 percent nominal-hydro approximation: ' + str(U))
        profile = 'route'
        weights, retained = e.m.roll.spectrum(float(data['Hm0_m']), float(data['Tp_s']))
    w = e.m.roll.W
    sel = np.flatnonzero(weights > 0)
    if not len(sel):
        raise ValueError('No nonzero wave support; no safe-zero substitute')
    if w[sel].min() < e.m.WW[0] - 1e-12 or w[sel].max() > e.m.WW[-1] + 1e-12:
        raise MissingFrozenInput('Input spectrum outside cached intrinsic frequency support')
    n = round(DURATION / DT)
    nuall = 2 * np.pi * np.fft.rfftfreq(n, d=DT)
    mask = (nuall >= 0.0001) & (nuall <= 12)
    params = read(e.P / 'new_control_parameter_seal.json')['effective_control_parameters']
    ctrl = dict(P=-float(params[0]), D_s=-float(params[1]))
    workpoint = read(O / 'r1_following_surge_diagnostic_v1/short_preflight/nominal_workpoint.json')
    transfer, tangent = prop.replace_operator(e.g, profile, U, nuall[mask],
                                               e.m.damping(U, np.zeros(3)), ctrl, workpoint)
    _LAST_CONTEXT = dict(_prepared_C_context=True, cache_key=key, original_input=data,
                         source_anchor_case=anchor, profile=profile, U=U, mu=mu,
                         weights=weights, w=w, sel=sel, n=n, drop=round(DROP / DT),
                         nuall=nuall, spectrum_mass_fraction=retained,
                         F=_wave_external(profile, U, mu, w[sel]),
                         C=_cross_coefficients(mu, w[sel]), transfer=transfer,
                         tangent=tangent, nominal_workpoint=workpoint, parameters=params)
    return _LAST_CONTEXT


def _input(data, seed):
    global _GEOMETRY
    import numpy as np
    e, prop, _ = engine()
    prepared = prepare_context(data)
    anchor, profile, U, mu = (prepared[k] for k in ('source_anchor_case', 'profile', 'U', 'mu'))
    weights, w, sel = (prepared[k] for k in ('weights', 'w', 'sel'))
    phi, z, bins, signed = e.m.frozen_trace(weights, U, mu, seed)
    n, drop = len(phi), round(DROP / DT)
    # Squared fields generate sum frequencies. Prevent FFT aliasing rather than
    # concealing it by a new highpass/lowpass correction.
    if 2 * abs(signed[sel]).max() >= math.pi / DT:
        raise ValueError('Quadratic encounter support violates frozen estimator Nyquist')
    forces = np.empty((3, n))
    cross = np.empty((3, n))
    F, C = prepared['F'], prepared['C']
    for j in range(3):
        ft = np.zeros(n, complex)
        np.add.at(ft, bins[sel], z[sel] * np.where(signed[sel] >= 0, F[:, j].conj(), F[:, j]))
        forces[j] = (np.fft.ifft(ft) * n).real
        ft = np.zeros(n, complex)
        np.add.at(ft, bins[sel], z[sel] * C[:, j].conj())
        cross[j] = phi * (np.fft.ifft(ft) * n).real
    if _GEOMETRY is None:
        with np.load(cache_file(e.FOLDER / 'quadrature_geometry_s2.npz')) as geometry:
            _GEOMETRY = {k: geometry[k].copy() for k in geometry.files}
    wl = e.base.geometry_waterline_force(_GEOMETRY, w, z, bins, sel, mu, n)
    trace = np.vstack([forces + cross + wl, phi])
    trace -= trace.mean(axis=1, keepdims=True)
    forcing = np.fft.rfft(trace, axis=1).T
    nuall = prepared['nuall']
    # Same two propeller midpoint Airy axial input as sealed C, at query angle.
    centers = np.asarray([[(0.5 - 0.937250345) * 142 + 0.683,
                           side * 0.03274648 * 142, -0.03601247 * 142]
                          for side in (-1, 1)])
    beta = math.radians(180 - mu)
    k = w[sel] ** 2 / 9.81
    phase = centers[:, 0, None] * math.cos(beta) + centers[:, 1, None] * math.sin(beta)
    amp = z[sel][None, :] * w[sel][None, :] * math.cos(beta) * np.exp(
        k[None, :] * centers[:, 2, None] - 1j * k[None, :] * phase)
    ft = np.zeros((2, n), complex)
    np.add.at(ft, (np.arange(2)[:, None], bins[sel][None, :]), amp)
    orbital = np.fft.rfft((np.fft.ifft(ft, axis=1) * n).real.mean(axis=0))
    workpoint = prepared['nominal_workpoint']
    tangent = prepared['tangent']
    forcing[:, 0] += tangent['B_P_N_s_m'] / (1 - workpoint['wake']) * orbital
    raw = dict(forcing_fft=forcing, nuall=nuall, U=U, n=n, drop=drop)
    context = dict(profile=profile, U=U, mu=mu, roll_SD_deg=float(phi[drop:].std() * 180 / np.pi),
                   spectrum_mass_fraction=prepared['spectrum_mass_fraction'], nominal_workpoint=workpoint,
                   propeller_centers_m=centers.tolist(), wave_from_scope='0 head / 180 following',
                   source_anchor_case=anchor, prepared=prepared)
    return raw, context










