"""Read published route coordinates without the local detailed run reports."""
import hashlib
import json
import math
from pathlib import Path

ROUTE_FILE = 'data/results/routes.json'
ROUTE_SCHEMA = 'ST_DFSO_ROUTES_V1'


def route_digest(points):
    """SHA-256 of canonical UTF-8 JSON; independent of file indentation."""
    data = json.dumps(points, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True, allow_nan=False).encode('utf-8')
    return hashlib.sha256(data).hexdigest()


def load_routes(root):
    document = json.loads((Path(root) / ROUTE_FILE).read_text('utf-8'))
    if document.get('schema') != ROUTE_SCHEMA:
        raise ValueError('Unsupported route coordinate schema')
    routes = document['routes']
    if not isinstance(routes, dict):
        raise ValueError('Route coordinates must be indexed by run identity')
    for key, points in routes.items():
        if not isinstance(points, list) or len(points) < 2:
            raise ValueError('Incomplete route coordinates: ' + key)
        for point in points:
            if (not isinstance(point, dict) or set(point) != {'i', 'j'}
                    or any(type(value) is not int for value in point.values())):
                raise ValueError('Expected integer grid indices i and j: ' + key)
    return routes


def validate_outcome(root, row, routes):
    """Check stored final measures and route identity, without a model rerun."""
    key = row['original_job_id']
    if row['route_key'] != key or key not in routes:
        raise ValueError('Missing or mismatched final route: ' + key)
    if route_digest(routes[key]) != row['route_sha256']:
        raise ValueError('Route checksum mismatch: ' + key)
    seed_file = 'config/seed_routes/' + row['job']['scene'] + '.json'
    if row['seed_route_file'] != seed_file or not (Path(root) / seed_file).is_file():
        raise ValueError('Missing or mismatched initial route: ' + key)
    final = row['final']
    if not (row['scientific_success_received'] and final['route_complete']
            and final['route_practical']):
        raise ValueError('Incomplete or inadmissible stored outcome: ' + key)
    values = final['five_goal_values']
    if set(values) != {'TIME', 'FUEL', 'SAFETY_DENSITY', 'SAFETY_EXPOSURE', 'COMPOSITE'}:
        raise ValueError('Incomplete objective values: ' + key)
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError('Non-finite objective value: ' + key)
    distance = final['distance_n_mile']
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError('Invalid stored distance: ' + key)
    if not math.isclose(values['SAFETY_DENSITY'] * distance,
                        values['SAFETY_EXPOSURE'], rel_tol=1e-10, abs_tol=1e-10):
        raise ValueError('Distance, motion and exposure do not agree: ' + key)
