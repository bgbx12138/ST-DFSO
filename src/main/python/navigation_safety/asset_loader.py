"""Load the fixed physical inputs used by the four-component response model."""
from pathlib import Path
from types import SimpleNamespace
import importlib.util
import json
import math
import os
import sys

REPO = Path(__file__).resolve().parents[4]
OWN = REPO / 'src/main/python/vessel_response'
CONFIG = '5415M_MARIN35.48_active_coursekeeping'


def encode(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)


def physics_input(request):
    if request.get('configuration', CONFIG) != CONFIG:
        raise ValueError('UNSUPPORTED_CONFIGURATION')
    d = dict(U_m_s=float(request['U_m_s']), mu_deg=float(request['mu_deg']))
    if not math.isfinite(d['U_m_s']) or d['U_m_s'] <= 0:
        raise ValueError('INVALID_WATER_SPEED')
    if not math.isfinite(d['mu_deg']) or not 0 <= d['mu_deg'] <= 180:
        raise ValueError('INVALID_RELATIVE_HEADING')
    if request.get('source_anchor_case') is not None:
        d['source_anchor_case'] = int(request['source_anchor_case'])
    else:
        d.update(Hm0_m=float(request['Hm0_m']), Tp_s=float(request['Tp_s']))
        if not math.isfinite(d['Hm0_m']) or d['Hm0_m'] < 0 or not math.isfinite(d['Tp_s']) or d['Tp_s'] <= 0:
            raise ValueError('INVALID_SEA_STATE')
        nominal = .248 * math.sqrt(9.81 * 142.0)
        if not .95 * nominal <= d['U_m_s'] <= 1.05 * nominal:
            raise ValueError('UNSUPPORTED_WATER_SPEED')
    return d


class AssetLoader:
    def __init__(self, cache):
        from release_integrity import verify_model_files
        from .cross_frequency_index392 import install as install_cross
        from .waterline357 import install as install_waterline
        verify_model_files()
        sys.path.insert(0, str(OWN / 'r1_C_route_replay_v1/response_impl'))
        import frozen_c_adapter as adapter
        install_cross(adapter)
        adapter.cache_inventory()
        e, _, _ = adapter.engine()
        if adapter._BRIDGE is None:
            adapter._BRIDGE = adapter.RollBridge(Path(cache).parent)
        else:
            adapter._BRIDGE.folder = Path(cache).parent
        e.m.fast_bridge.trace = adapter._BRIDGE.trace
        install_waterline(e.base)
        path = OWN / 'r1_segment56_root_cause_v1/repair/low_frequency_operator.py'
        spec = importlib.util.spec_from_file_location('stdfso_low_frequency', path)
        low = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(low)
        policy = json.loads((REPO / 'config/response_scales.json').read_text('utf8'))
        relative = [float(x) for x in os.environ.get('R1_COMPONENT_RELATIVE_WEIGHTS', '1.5,1.25,1,1').split(',')]
        total = sum(relative)
        if len(relative) != 4 or any(not math.isfinite(x) or x < 0 for x in relative) or not math.isfinite(total) or total <= 0:
            raise ValueError('Four finite nonnegative weights with positive sum required')
        self._contract = dict(weights=[x / total for x in relative], reference_scales=policy['reference_scales'], physical_contract={})
        self._base = SimpleNamespace(_ADAPTER=adapter, _LOW_FREQUENCY=low, encode=encode, physics_input=physics_input)

    def close(self):
        self._base._ADAPTER.close()
