"""Extract per-date/per-field 2026 source missing masks for isolated repair copies."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from netCDF4 import Dataset


CASES = {
    "S2": "S2_south_china_sea",
    "S3": "S3_miyako_strait",
    "S4": "S4_bashi_channel",
    "S6": "S6_qingdao_busan",
}
DATES = tuple(f"202603{day:02d}" for day in range(11, 18))
CURRENT_VARS = {"uo": ("cur", (0, 0)), "vo": ("cur", (0, 0))}
WAVE_VARS = {
    "VHM0": ("wave", (0,)),
    "VTPK": ("wave", (0,)),
    "VMDR": ("wave", (0,)),
}


def grid_round(value: float) -> float:
    return math.floor(value * 100.0 + 0.5) / 100.0


def target_axis(low: float, high: float, count: int) -> np.ndarray:
    return np.asarray([grid_round(low + k * (high - low) / (count - 1)) for k in range(count)])


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def nearest_indices(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    return np.abs(np.asarray(source, dtype=float)[:, None] - target[None, :]).argmin(axis=0)


def variable_mask(dataset: Dataset, variable: str, prefix: tuple[int, ...],
                  lat_indices: np.ndarray, lon_indices: np.ndarray) -> np.ndarray:
    values = dataset.variables[variable][prefix + (slice(None), slice(None))]
    mask = np.ma.getmaskarray(values).copy()
    mask |= ~np.isfinite(np.ma.filled(values, np.nan))
    return mask[np.ix_(lat_indices, lon_indices)]


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    a = p.parse_args()
    root, out_dir = a.root.absolute(), a.out_dir.absolute()
    raw_root = root / "ncData" / "20260311"
    out_dir.mkdir(parents=True, exist_ok=True)
    result: dict[str, object] = {
        "purpose": "source-missing sidecars for isolated interpolation copies; no project inputs are edited",
        "mapping": "nearest source latitude/longitude to Java two-decimal target grid; preserve per-day masks",
        "dates": list(DATES),
        "cases": {},
    }

    for case, folder in CASES.items():
        data_dir = root / folder
        config = json.loads((data_dir / "scenario_config.json").read_text(encoding="utf-8"))
        nx, ny = int(config["grid"]["nx"]), int(config["grid"]["ny"])
        target_lon = target_axis(config["area"]["lonMin"], config["area"]["lonMax"], nx)
        target_lat = target_axis(config["area"]["latMin"], config["area"]["latMax"], ny)
        per_date: dict[str, object] = {}
        file_hashes: dict[str, str] = {}
        totals: dict[str, int] = {}

        for date in DATES:
            entry: dict[str, list[int]] = {}
            for suffix, variables in (("cur", CURRENT_VARS), ("wave", WAVE_VARS)):
                path = raw_root / f"{date}00_{suffix}.nc"
                file_hashes[str(path.relative_to(root))] = file_sha256(path)
                with Dataset(str(path)) as ds:
                    lat_indices = nearest_indices(ds.variables["latitude"][:], target_lat)
                    lon_indices = nearest_indices(ds.variables["longitude"][:], target_lon)
                    for variable, (kind, prefix) in variables.items():
                        mask = variable_mask(ds, variable, prefix, lat_indices, lon_indices)
                        key = f"{kind}:{variable}"
                        entry[key] = np.flatnonzero(mask.ravel()).astype(int).tolist()
                        totals[key] = totals.get(key, 0) + int(mask.sum())
            # A vector field uses common support: if either component is absent,
            # both components will be jointly interpolated at that target cell.
            current_union = np.zeros((ny, nx), dtype=bool)
            for variable in CURRENT_VARS:
                current_union.ravel()[entry[f"cur:{variable}"]] = True
            entry["current_vector_union"] = np.flatnonzero(current_union.ravel()).astype(int).tolist()
            per_date[date] = entry

        result["cases"][case] = {
            "data_folder": folder,
            "grid": {"nx": nx, "ny": ny},
            "source_file_sha256": file_hashes,
            "source_missing_cell_counts_summed_over_dates": totals,
            "per_date_flat_indices": per_date,
        }
        print(case, {k: v for k, v in totals.items()})

    out_path = out_dir / "2026_source_missing_masks_v1.json"
    out_path.write_text(json.dumps(result, separators=(",", ":")), encoding="utf-8")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
