"""Spatial-block validation of local ocean-only interpolators on supplied environmental fields.

This is a validation harness only: it reads existing JSON and the physical
obstacle map, writes CSV diagnostics under an explicit output directory, and
does not edit experiment inputs or maps.
"""

from __future__ import annotations

import argparse
import csv
import json
import hashlib
from pathlib import Path

import numpy as np


GROUPS = {
    "current_vector": ("cur", ("currentDataU", "currentDataV"), False),
    "wind_vector": ("wind", ("windDataU", "windDataV"), False),
    "wave_height": ("wave", ("waveDataH",), False),
    "wave_period": ("wave", ("waveDataP",), False),
    "wave_direction": ("wave", ("waveDataD",), True),
}
RADII = (1, 2, 3)
BLOCK_SIZES = (1, 2, 3)
MIN_NEIGHBOURS = 3
MIN_SECTORS = 3


def load_field(path: Path, field: str, shape: tuple[int, int]) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    values = payload[field]
    out = np.empty(len(values), dtype=float)
    for i, value in enumerate(values):
        try:
            out[i] = float(value)
        except (TypeError, ValueError):
            out[i] = np.nan
    out[~np.isfinite(out)] = np.nan
    return out.reshape(shape)


def sample_blocks(valid: np.ndarray, block_size: int, count: int,
                  rng: np.random.Generator, exclusion_radius: int = 3) -> np.ndarray:
    """Sample non-overlapping all-valid water blocks for a spatial holdout fold."""
    ny, nx = valid.shape
    held = np.zeros_like(valid, dtype=bool)
    occupied = np.zeros_like(valid, dtype=bool)
    found = 0
    attempts = 0
    while found < count and attempts < count * 500:
        attempts += 1
        i = int(rng.integers(0, ny - block_size + 1))
        j = int(rng.integers(0, nx - block_size + 1))
        sl = (slice(i, i + block_size), slice(j, j + block_size))
        if not np.all(valid[sl]):
            continue
        y0 = max(0, i - exclusion_radius)
        y1 = min(ny, i + block_size + exclusion_radius)
        x0 = max(0, j - exclusion_radius)
        x1 = min(nx, j + block_size + exclusion_radius)
        if np.any(occupied[y0:y1, x0:x1]):
            continue
        held[sl] = True
        occupied[y0:y1, x0:x1] = True
        found += 1
    return held


def estimate(data: np.ndarray, water: np.ndarray, train_valid: np.ndarray,
             coords: np.ndarray, radius: int, circular: bool,
             method: str) -> tuple[np.ndarray, np.ndarray]:
    """Estimate at target cells from nearby water-only values and local support."""
    n, ncomp = len(coords), data.shape[-1]
    if n == 0:
        return np.empty((0, ncomp)), np.zeros(0, dtype=bool)
    ys0, xs0 = coords[:, 0], coords[:, 1]
    ny, nx = water.shape
    weights = np.zeros(n, dtype=float)
    sums = np.zeros((n, ncomp), dtype=float)
    sins = np.zeros(n, dtype=float)
    coses = np.zeros(n, dtype=float)
    sectors = np.zeros(n, dtype=np.uint8)
    nearest_dist = np.full(n, np.inf)
    nearest_val = np.full((n, ncomp), np.nan)
    neighbour_count = np.zeros(n, dtype=np.int16)

    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            if di == 0 and dj == 0:
                continue
            ys, xs = ys0 + di, xs0 + dj
            inside = (ys >= 0) & (ys < ny) & (xs >= 0) & (xs < nx)
            ok = inside.copy()
            # Requiring the whole local rectangle to be navigable prevents a
            # stencil from borrowing values across a land/obstacle strip.
            for pi in range(min(0, di), max(0, di) + 1):
                for pj in range(min(0, dj), max(0, dj) + 1):
                    py, px = ys0 + pi, xs0 + pj
                    pin = (py >= 0) & (py < ny) & (px >= 0) & (px < nx)
                    ok &= pin
                    safe_y, safe_x = np.clip(py, 0, ny - 1), np.clip(px, 0, nx - 1)
                    ok &= water[safe_y, safe_x]
            safe_y, safe_x = np.clip(ys, 0, ny - 1), np.clip(xs, 0, nx - 1)
            ok &= train_valid[safe_y, safe_x]
            if not np.any(ok):
                continue
            dist2 = float(di * di + dj * dj)
            weight = 1.0 / dist2
            vals = data[safe_y, safe_x, :]
            neighbour_count[ok] += 1
            weights[ok] += weight
            if circular:
                radians = np.deg2rad(vals[ok, 0])
                sins[ok] += weight * np.sin(radians)
                coses[ok] += weight * np.cos(radians)
            else:
                sums[ok] += weight * vals[ok]
            sector = int(np.floor((np.arctan2(di, dj) + np.pi) * (8.0 / (2.0 * np.pi)))) % 8
            sectors[ok] |= np.uint8(1 << sector)
            # Update each held target independently. Equal-distance ties retain
            # the first neighbour in ascending (di, dj) order.
            better = ok & (dist2 < nearest_dist)
            nearest_dist[better] = dist2
            nearest_val[better] = vals[better]

    sector_count = np.unpackbits(sectors[:, None], axis=1).sum(axis=1)
    supported = (neighbour_count >= MIN_NEIGHBOURS) & (sector_count >= MIN_SECTORS) & (weights > 0)
    prediction = np.full((n, ncomp), np.nan)
    if circular:
        supported &= np.hypot(sins, coses) > 1e-10 * weights
        prediction[supported, 0] = np.mod(
            np.rad2deg(np.arctan2(sins[supported], coses[supported])), 360.0
        )
    else:
        prediction[supported] = sums[supported] / weights[supported, None]
    if method == "nearest":
        supported &= np.all(np.isfinite(nearest_val), axis=1)
        prediction[supported] = nearest_val[supported]
    return prediction, supported


def score(truth: np.ndarray, pred: np.ndarray, supported: np.ndarray,
          circular: bool) -> tuple[int, float, float]:
    if not np.any(supported):
        return 0, float("nan"), float("nan")
    delta = pred[supported] - truth[supported]
    if circular:
        delta = (delta + 180.0) % 360.0 - 180.0
    if delta.shape[1] == 1:
        scalar = delta[:, 0]
        return len(scalar), float(np.mean(np.abs(scalar))), float(np.sqrt(np.mean(scalar ** 2)))
    magnitude = np.linalg.norm(delta, axis=1)
    return len(magnitude), float(np.mean(magnitude)), float(np.sqrt(np.mean(magnitude ** 2)))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--exclude-map", type=Path,
                        help="Optional strict map; its extra blocked water cells are excluded from CV support.")
    parser.add_argument("--start", default="20220616")
    parser.add_argument("--days", type=int, default=29)
    parser.add_argument("--blocks-per-day", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20261005)
    args = parser.parse_args()

    data_dir, out_dir = args.data_dir, args.out_dir
    obstacle = json.loads((data_dir / "obstacle_map.json").read_text(encoding="utf-8"))
    shape = (int(obstacle["gridXY"]["ny"]), int(obstacle["gridXY"]["nx"]))
    water = ~np.asarray(obstacle["obstacleMap"], dtype=bool)
    if water.shape != shape:
        raise ValueError(f"obstacle shape {water.shape} != declared {shape}")
    source_union_mask = np.zeros(shape, dtype=bool)
    if args.exclude_map:
        excluded = json.loads(args.exclude_map.read_text(encoding="utf-8"))
        excluded_obstacles = np.asarray(excluded["obstacleMap"], dtype=bool)
        if excluded_obstacles.shape != shape:
            raise ValueError(f"exclude-map shape {excluded_obstacles.shape} != {shape}")
        source_union_mask = water & excluded_obstacles
    out_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    held_records = []
    input_hashes = {}
    records: list[dict[str, object]] = []
    actual_records: list[dict[str, object]] = []

    for offset in range(args.days):
        from datetime import datetime, timedelta
        day = (datetime.strptime(args.start, "%Y%m%d") + timedelta(days=offset)).strftime("%Y%m%d")
        for group, (suffix, fields, circular) in GROUPS.items():
            path = data_dir / f"{day}_{suffix}.json"
            if not path.exists():
                raise FileNotFoundError(path)
            input_hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
            parts = [load_field(path, field, shape) for field in fields]
            values = np.stack(parts, axis=-1)
            finite = np.all(np.isfinite(values), axis=-1) & ~source_union_mask
            true_missing = water & (~np.all(np.isfinite(values), axis=-1) | source_union_mask)
            for radius in RADII:
                coords = np.argwhere(true_missing)
                _, fillable = estimate(values, water, finite, coords, radius, circular, "idw")
                actual_records.append({
                    "date": day,
                    "group": group,
                    "radius_cells": radius,
                    "navigable_missing": int(len(coords)),
                    "fillable": int(fillable.sum()),
                    "residual_unsupported": int(len(coords) - fillable.sum()),
                    "mask_scope": "source_union" if args.exclude_map else "field_values",
                })

            for block_size in BLOCK_SIZES:
                held = sample_blocks(water & finite, block_size, args.blocks_per_day, rng)
                coords = np.argwhere(held)
                held_records.extend({'date': day, 'group': group, 'block_size': block_size,
                                     'row_zero_based': int(i), 'column_zero_based': int(j)}
                                    for i, j in coords)
                train_valid = finite & ~held
                for radius in RADII:
                    for method in ("idw", "nearest"):
                        pred, supported = estimate(values, water, train_valid, coords, radius, circular, method)
                        n, mae, rmse = score(values[held], pred, supported, circular)
                        records.append({
                            "date": day,
                            "group": group,
                            "block_size": block_size,
                            "radius_cells": radius,
                            "method": method,
                            "requested": int(len(coords)),
                            "predicted": n,
                            "coverage": (n / len(coords)) if len(coords) else 0.0,
                            "MAE": mae,
                            "RMSE": rmse,
                        })

    for name, rows in (("spatial_block_cv.csv", records), ("actual_gap_support.csv", actual_records), ("held_target_coordinates.csv", held_records)):
        with (out_dir / name).open("w", encoding="utf-8-sig", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    protocol = {'seed': args.seed, 'rng': 'numpy default_rng / PCG64',
                'numpy_version': np.__version__, 'data_dir': str(data_dir.resolve()),
                'start': args.start, 'days': args.days, 'blocks_per_day': args.blocks_per_day,
                'block_sizes': BLOCK_SIZES, 'radii': RADII, 'min_neighbors': MIN_NEIGHBOURS,
                'min_sectors': MIN_SECTORS, 'case_input_sha256': input_hashes,
                'physical_map_sha256': hashlib.sha256((data_dir/'obstacle_map.json').read_bytes()).hexdigest(),
                'exclusion_map': str(args.exclude_map) if args.exclude_map else None,
                'exclusion_map_sha256': hashlib.sha256(args.exclude_map.read_bytes()).hexdigest() if args.exclude_map else None,
                'targets_sha256': hashlib.sha256((out_dir/'held_target_coordinates.csv').read_bytes()).hexdigest(),
                'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'comparison': 'same held coordinates and identical supported target mask for IDW and NN'}
    (out_dir/'protocol.json').write_text(json.dumps(protocol, indent=2), encoding='utf-8')
    print(f"wrote {len(records)} CV rows and {len(actual_records)} actual-gap rows to {out_dir}")


if __name__ == "__main__":
    main()
