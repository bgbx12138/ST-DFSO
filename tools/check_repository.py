"""Check published data and model dependencies without running an experiment."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import sys
import os
import math
from collections import Counter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'analysis'))
from result_data import ROUTE_FILE, load_routes, validate_outcome


def public_result_files(root):
    """Skip detailed local records before descending into their directory."""
    base = root / 'data/results'
    for directory, subdirectories, names in os.walk(base):
        if Path(directory) == base:
            subdirectories[:] = [name for name in subdirectories if name != 'runs']
        for name in names:
            yield Path(directory) / name


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def local_file(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('Path outside repository: ' + relative)
    return path


def check(root):
    failures = []
    hashes = {}
    data = json.loads((root / 'data/checksums.json').read_text('utf-8'))
    model = json.loads((root / 'config/model_files.json').read_text('utf-8'))
    for label, entries in (('results', data['files']), ('model', model)):
        seen = set()
        for entry in entries:
            name = entry['file']
            if label == 'results' and name.startswith('data/results/runs/'):
                failures.append('Local-only report in public checksum list: ' + name)
                continue
            if name in seen:
                failures.append('Duplicate checksum entry: ' + name)
            seen.add(name)
            path = local_file(root, name)
            if not path.is_file():
                failures.append('Missing file: ' + name)
                continue
            hashes[name] = sha256(path)
            if hashes[name] != entry['sha256']:
                failures.append('Checksum mismatch: ' + name)
        if label == 'results':
            actual = {p.relative_to(root).as_posix() for p in public_result_files(root)}
            failures.extend('Unindexed result file: ' + name for name in sorted(actual - seen))

    runs = json.loads((root / 'data/results/accepted_runs.json').read_text('utf-8'))
    coordinates = load_routes(root)
    jobs = json.loads((root / 'config/experiment.json').read_text('utf-8'))['jobs']
    identities = [row['original_job_id'] for row in runs]
    if len(runs) != 810 or len(set(identities)) != 810:
        failures.append('Expected 810 distinct optimisation identities')
    if set(identities) != {job['id'] for job in jobs} or len(jobs) != 810:
        failures.append('Experiment configurations and accepted runs differ')
    if set(coordinates) != set(identities):
        failures.append('Route coordinates and accepted run identities differ')
    scenes = {f'S{i}' for i in range(1,7)}
    inventory = json.loads((root / 'config/environment_inventory.json').read_text('utf-8'))
    if {job['scene'] for job in jobs} != scenes or {x['scenario_id'] for x in inventory} != scenes:
        failures.append('Scenario numbers must match manuscript S1–S6')
    if {p.stem for p in (root / 'config/seed_routes').glob('*.json')} != scenes:
        failures.append('Expected one initial route for each of S1–S6')
    for row in runs:
        scene = row['job']['scene']
        if scene not in scenes or ('_' + scene + '_') not in row['original_job_id']:
            failures.append('Scenario mismatch in run: ' + row['original_job_id'])
        try:
            validate_outcome(root, row, coordinates)
        except (OSError, ValueError, KeyError, TypeError) as error:
            failures.append(str(error))

    with (root / 'data/results/tables/final_route_metrics.csv').open(
            encoding='utf-8-sig', newline='') as stream:
        routes = list(csv.DictReader(stream))
    counts = Counter(row['algorithm'] for row in routes)
    accepted = {row['original_job_id']: row for row in runs}
    if len(routes) != 360 or set(counts.values()) != {90} or len(counts) != 4:
        failures.append('Expected 90 final routes for each of four algorithms')
    for row in routes:
        if row['source_scene'] != row['paper_case'] or row['paper_case'] not in scenes:
            failures.append('Final-route scenario differs from manuscript: ' + row['original_run_id'])
        original = accepted.get(row['original_run_id'])
        if (original is None or 'MAIN' not in original['job']['views']
                or row['route_file'] != ROUTE_FILE or ROUTE_FILE not in hashes
                or row['route_key'] != original['route_key']
                or row['route_sha256'] != original['route_sha256']
                or row['selected_run_id'] != original['job']['id']):
            failures.append('Final-route index mismatch: ' + row['original_run_id'])
            continue
        pairs = [('voyage_time_h', 'TIME'), ('energy_J', 'FUEL'),
                 ('mean_ship_motion_cost', 'SAFETY_DENSITY'),
                 ('cumulative_exposure', 'SAFETY_EXPOSURE'), ('composite_cost', 'COMPOSITE')]
        values = [(field, original['final']['five_goal_values'][goal]) for field, goal in pairs]
        values.append(('distance_n_mile', original['final']['distance_n_mile']))
        for field, expected in values:
            if not math.isclose(float(row[field]), expected, rel_tol=1e-12, abs_tol=1e-12):
                failures.append('Final-route value mismatch: ' + row['original_run_id'] + '/' + field)

    with (root / 'data/results/Supplementary_Data_S1.csv').open(encoding='utf-8-sig',newline='') as stream:
        decomposition = list(csv.DictReader(stream))
    run_ids = set(identities) | {row['job']['id'] for row in runs}
    if len(decomposition) != 270:
        failures.append('Expected 270 route-decomposition pairs in Supplementary Data S1')
    for row in decomposition:
        if row['scene'] not in scenes or any(row[k] not in run_ids for k in ('st_job','target_job')):
            failures.append('Unlinked Supplementary Data S1 pair')
    paper_tables = json.loads((root / 'data/results/paper_tables/index.json').read_text('utf-8'))
    if {row['table'] for row in paper_tables} != {f'Table {i}' for i in range(1,4)}:
        failures.append('Published table index is incomplete')
    for row in paper_tables:
        if row['file'] not in hashes:failures.append('Unindexed published table: ' + row['file'])

    revised=True
    revised_count=0
    if revised:
        spec_path=root/'config/wave_response_initial_pool.json'
        if not spec_path.exists():
            failures.append('Revised common initial pool has not been prepared')
        else:
            spec=json.loads(spec_path.read_text('utf8'));pool_path=local_file(root,spec['data_file'])
            values=json.loads(pool_path.read_text('utf8'));revised_count=len(values)
            if (sha256(pool_path)!=spec['data_sha256'] or len(values)!=spec['records']
                    or len({r['input_sha256'] for r in values})!=len(values)
                    or sha256(root/'config/model_files.json')!=spec['compatible_model_manifest_sha256']
                    or sha256(root/'config/voyage_speed.json')!=spec['speed_config_sha256']):
                failures.append('Revised initial pool identity mismatch')
            if set(spec['input_routes'])!=scenes:
                failures.append('Revised initial pool must cover all six common input routes')
            for scene,h in spec['input_routes'].items():
                if sha256(root/f'config/seed_routes/{scene}.json')!=h:failures.append('Changed initial route: '+scene)
            for row in values:
                encoded=json.dumps(row['physics_input'],sort_keys=True,separators=(',',':'),allow_nan=False)
                if (hashlib.sha256(encoded.encode()).hexdigest()!=row['input_sha256']
                        or row['revision']!=spec['physical_revision'] or row['status']!='OK'
                        or not row['input_unchanged']
                        or row.get('schema')!='STDFSO_WAVE_HEADING_FOUR_COMPONENT_V1'
                        or not row.get('numerical_validation',{}).get('passed')):
                    failures.append('Invalid revised initial response: '+row['input_sha256'])

    return {'result_files': len(data['files']), 'model_files': len(model), 'initial_responses': revised_count,
            'revised_initial_responses':revised_count,
            'optimisation_runs': len(runs), 'final_routes': len(routes),
            'paper_tables': len(paper_tables), 'decomposition_pairs': len(decomposition),
            'algorithms': dict(counts), 'failures': failures}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    args = parser.parse_args()
    try:
        report = check(args.root.resolve())
    except (OSError, ValueError, KeyError, TypeError) as error:
        print('Repository check failed: ' + str(error), file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    return bool(report['failures'])


if __name__ == '__main__':
    raise SystemExit(main())
