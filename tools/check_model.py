"""Initialise the response model and its solvers without evaluating a route."""
from pathlib import Path
import json
import os
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(ROOT / 'src/main/python'))
    # The model's fixed asset references are relative to the project root.
    os.chdir(ROOT)
    from navigation_safety.unified_response import SafetyFunction
    with tempfile.TemporaryDirectory(prefix='stdfso-check-') as folder:
        with SafetyFunction(Path(folder) / 'responses.sqlite3') as model:
            contract = model.contract()
            report = {'initialised': True, 'revision': model.revision,
                      'weights': contract['weights'],
                      'reference_scales': contract['reference_scales'],
                      'stats': model.stats.copy()}
    print(json.dumps(report, indent=2))
    if any(report['stats'].values()):
        raise RuntimeError('Initialisation unexpectedly performed a model evaluation')


if __name__ == '__main__':
    main()
