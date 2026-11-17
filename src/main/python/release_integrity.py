"""Check the relocated source/asset manifest without running a model."""
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[3]
def verify_model_files():
    for row in json.loads((ROOT/'config/model_files.json').read_text('utf-8')):
        p=ROOT/row['file']
        if hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:
            raise RuntimeError('Released model source or asset changed: '+row['file'])

