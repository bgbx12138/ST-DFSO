"""Fixed-controller horizontal response with prescribed wave and roll forcing."""
import hashlib
import json
from pathlib import Path
from types import MappingProxyType
from .response_dynamics import ForcedLinearResponse

VERSION = 'stdfso_fixed_controller_linear_horizontal_v1'


class FixedLinearResponse(ForcedLinearResponse):
    def __init__(self, loader, config_path):
        super().__init__(loader)
        raw = Path(config_path).read_bytes()
        self.config = json.loads(raw)
        if self.config['schema'] != 'STDFSO_FIXED_LINEAR_RESPONSE_V1':
            raise ValueError('Unknown response configuration')
        if self.config['parameter_fitting']:
            raise ValueError('This response uses fixed parameters')
        if self.config['controller'] != dict(P=-.8, D_s=-24.9):
            raise ValueError('Published controller provenance no longer matches configuration')
        self.control = MappingProxyType(dict(self.config['controller']))
        self.config_sha256 = hashlib.sha256(raw).hexdigest()

    def evaluate(self, query, dt=.125, seed=20261018, roll_mode='equivalent', closure='zero'):
        if closure != 'zero' or roll_mode != 'equivalent':
            raise ValueError('Fixed linear response requires zero horizontal closure and equivalent roll')
        result = super().evaluate(query, dt=dt, seed=seed, roll_mode=roll_mode, closure='zero')
        result.update(model=VERSION, controller=dict(self.control),
                      config_sha256=self.config_sha256,
                      horizontal_nonlinear_iterations=0,
                      horizontal_root_selection=False,
                      horizontal_model='linear in prescribed forcing',
                      published_controller_borrowed=True)
        return result
