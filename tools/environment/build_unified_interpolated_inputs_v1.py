"""Create isolated input copies with one ocean-constrained missing-data policy."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path

import numpy as np


FOLDERS = {
    "S1_S5_taiwan": ("S1_S5_taiwan", None),  # S1/S5: null/NaN already preserved
    "S2_south_china_sea": ("S2_south_china_sea", "S2"),
    "S3_miyako_strait": ("S3_miyako_strait", "S3"),
    "S4_bashi_channel": ("S4_bashi_channel", "S4"),
    "S6_qingdao_busan": ("S6_qingdao_busan", "S6"),
}
FIELD_MAP = {
    "cur": {"currentDataU": "current_vector", "currentDataV": "current_vector"},
    "wind": {"windDataU": "wind_vector", "windDataV": "wind_vector"},
    "wave": {"waveDataH": "scalar_height", "waveDataP": "scalar_period", "waveDataD": "circular_direction"},
}
SOURCE_WAVE_MASKS = {
    "waveDataH": "wave:VHM0",
    "waveDataP": "wave:VTPK",
    "waveDataD": "wave:VMDR",
}
RADIUS = 1
MIN_NEIGHBOURS = 3
MIN_SECTORS = 3


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def numeric_array(values: list[object], shape: tuple[int, int]) -> np.ndarray:
    out = np.empty(len(values), dtype=float)
    for i, value in enumerate(values):
        try:
            out[i] = float(value)
        except (TypeError, ValueError):
            out[i] = np.nan
    out[~np.isfinite(out)] = np.nan
    if out.size != shape[0] * shape[1]:
        raise ValueError(f"array length {out.size} != expected grid {shape}")
    return out.reshape(shape)


def serialized(values: np.ndarray) -> list[object]:
    return [None if not np.isfinite(v) else round(float(v), 7) for v in values.ravel()]


def direction_sector(di: int, dj: int) -> int:
    return int(math.floor((math.atan2(di, dj) + math.pi) * (8.0 / (2.0 * math.pi)))) % 8


def interpolate_group(arrays: list[np.ndarray], water: np.ndarray,
                      missing: np.ndarray, circular: bool) -> tuple[list[np.ndarray], int, int, int]:
    """Fill only supported water gaps using 8-neighbour inverse-distance weights."""
    values = np.stack(arrays, axis=-1).astype(float, copy=True)
    joint_missing = missing.copy()
    for k in range(values.shape[-1]):
        joint_missing |= ~np.isfinite(values[:, :, k])
    targets = np.argwhere(water & joint_missing)
    input_missing = int(len(targets))
    valid = water & np.all(np.isfinite(values), axis=-1) & ~joint_missing
    filled = 0

    for i, j in targets:
        weights: list[float] = []
        neighbours: list[np.ndarray] = []
        sectors: set[int] = set()
        for di in range(-RADIUS, RADIUS + 1):
            for dj in range(-RADIUS, RADIUS + 1):
                if di == 0 and dj == 0:
                    continue
                ni, nj = i + di, j + dj
                if ni < 0 or ni >= water.shape[0] or nj < 0 or nj >= water.shape[1]:
                    continue
                # The full bounding rectangle must be open water. This excludes
                # interpolation stencils that cross a physical land/obstacle cell.
                if not np.all(water[min(i, ni):max(i, ni) + 1, min(j, nj):max(j, nj) + 1]):
                    continue
                if not valid[ni, nj]:
                    continue
                weights.append(1.0 / float(di * di + dj * dj))
                neighbours.append(values[ni, nj, :])
                sectors.add(direction_sector(di, dj))
        if len(neighbours) < MIN_NEIGHBOURS or len(sectors) < MIN_SECTORS:
            continue
        w = np.asarray(weights, dtype=float)
        matrix = np.asarray(neighbours, dtype=float)
        if circular:
            angles = np.deg2rad(matrix[:, 0])
            sin_sum = float(np.sum(w * np.sin(angles)))
            cos_sum = float(np.sum(w * np.cos(angles)))
            if math.hypot(sin_sum, cos_sum) <= 1e-10 * float(w.sum()):
                continue
            estimate = np.asarray([math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0])
        else:
            estimate = np.sum(matrix * w[:, None], axis=0) / float(w.sum())
        values[i, j, :] = estimate
        filled += 1

    residual = input_missing - filled
    return [values[:, :, k] for k in range(values.shape[-1])], input_missing, filled, residual


def mask_from_indices(indices: list[int], shape: tuple[int, int]) -> np.ndarray:
    out = np.zeros(shape[0] * shape[1], dtype=bool)
    if indices:
        out[np.asarray(indices, dtype=int)] = True
    return out.reshape(shape)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--source-masks", type=Path, required=True)
    parser.add_argument("--out-root", type=Path, required=True)
    args = parser.parse_args()

    source_root, out_root = args.source_root.absolute(), args.out_root.absolute()
    data_root = out_root / "data"
    if out_root.exists():
        raise FileExistsError(f"Refusing to overwrite {out_root}")
    out_root.mkdir(parents=True)
    data_root.mkdir()
    sidecar = json.loads(args.source_masks.read_text(encoding="utf-8"))
    run_manifest: dict[str, object] = {
        "policy_id": "ocean_idw_v1",
        "policy": {
            "physical_obstacle_map": "preserved; never interpolated across",
            "observed_zero": "preserved as a measured value",
            "source_missing": "restored from per-field raw masks where historical JSON had finite zeros; otherwise read from null/nonfinite JSON values",
            "interpolator": "local inverse-distance weighted mean over at most the 8 adjacent target cells; weight=1/squared grid distance",
            "support": "at least 3 valid neighbors spanning at least 3 octants; every cell in the target-neighbor bounding rectangle must be navigable",
            "fields": "current and wind components jointly interpolated; wave height/period separately; wave direction uses circular mean",
            "residual": "remain JSON null and are rejected by the route scorer when required",
            "radius_cells": RADIUS,
            "minimum_neighbors": MIN_NEIGHBOURS,
            "minimum_direction_sectors": MIN_SECTORS,
        },
        "source_missing_masks_sha256": sha256(args.source_masks),
        "cases": {},
    }

    for relative, (copy_relative, source_case) in FOLDERS.items():
        source_dir = source_root / Path(*relative.split("/"))
        target_dir = data_root / Path(*copy_relative.split("/"))
        shutil.copytree(source_dir, target_dir)
        map_data = json.loads((source_dir / "obstacle_map.json").read_text(encoding="utf-8"))
        shape = (int(map_data["gridXY"]["ny"]), int(map_data["gridXY"]["nx"]))
        obstacle = np.asarray(map_data["obstacleMap"], dtype=bool)
        if obstacle.shape != shape:
            raise ValueError(f"{relative} obstacle shape {obstacle.shape} != {shape}")
        water = ~obstacle
        case_manifest: dict[str, object] = {
            "source_folder": relative,
            "grid": {"ny": shape[0], "nx": shape[1]},
            "input_hashes": {},
            "output_hashes": {},
            "per_file": {},
        }
        cur_paths = sorted(source_dir.glob("????????_cur.json"))
        for cur_path in cur_paths:
            date = cur_path.name[:8]
            wave_path = source_dir / f"{date}_wave.json"
            wind_path = source_dir / f"{date}_wind.json"
            if not wave_path.exists() or not wind_path.exists():
                continue
            payloads = {
                "cur": json.loads(cur_path.read_text(encoding="utf-8")),
                "wave": json.loads(wave_path.read_text(encoding="utf-8")),
                "wind": json.loads(wind_path.read_text(encoding="utf-8")),
            }
            source_payloads = {kind: dict(value) for kind, value in payloads.items()}
            date_masks = None
            if source_case:
                date_masks = sidecar["cases"][source_case]["per_date_flat_indices"].get(date)
                if date_masks is None:
                    raise KeyError(f"no raw source mask for {source_case} {date}")

            day_metrics: dict[str, object] = {}

            # Current U/V are treated as one vector: if either raw component is
            # missing, both are reconstructed from a shared complete stencil.
            cur_u = numeric_array(payloads["cur"]["currentDataU"], shape)
            cur_v = numeric_array(payloads["cur"]["currentDataV"], shape)
            cur_mask = ~np.isfinite(cur_u) | ~np.isfinite(cur_v)
            if date_masks is not None:
                cur_mask |= mask_from_indices(date_masks["current_vector_union"], shape)
            cur_u[cur_mask] = np.nan
            cur_v[cur_mask] = np.nan
            (cur_u, cur_v), n, f, r = interpolate_group([cur_u, cur_v], water, cur_mask, False)
            payloads["cur"]["currentDataU"] = serialized(cur_u)
            payloads["cur"]["currentDataV"] = serialized(cur_v)
            day_metrics["current_vector"] = {"navigable_missing": n, "filled": f, "residual": r}

            # Wind vectors use the same vector-support rule; a finite zero is
            # retained unless the stored JSON itself marks it missing.
            wind_u = numeric_array(payloads["wind"]["windDataU"], shape)
            wind_v = numeric_array(payloads["wind"]["windDataV"], shape)
            wind_mask = ~np.isfinite(wind_u) | ~np.isfinite(wind_v)
            if date_masks is not None:
                wind_mask |= mask_from_indices(date_masks.get("wind_vector_union", []), shape)
            wind_u[wind_mask] = np.nan
            wind_v[wind_mask] = np.nan
            (wind_u, wind_v), n, f, r = interpolate_group([wind_u, wind_v], water, wind_mask, False)
            payloads["wind"]["windDataU"] = serialized(wind_u)
            payloads["wind"]["windDataV"] = serialized(wind_v)
            day_metrics["wind_vector"] = {"navigable_missing": n, "filled": f, "residual": r}

            wave_metrics: dict[str, object] = {}
            for field, source_mask_key, kind in (
                ("waveDataH", "wave:VHM0", "scalar_height"),
                ("waveDataP", "wave:VTPK", "scalar_period"),
                ("waveDataD", "wave:VMDR", "circular_direction"),
            ):
                wave_value = numeric_array(payloads["wave"][field], shape)
                wave_mask = ~np.isfinite(wave_value)
                if date_masks is not None:
                    wave_mask |= mask_from_indices(date_masks.get(source_mask_key, []), shape)
                wave_value[wave_mask] = np.nan
                (wave_value,), n, f, r = interpolate_group(
                    [wave_value], water, wave_mask, kind == "circular_direction"
                )
                payloads["wave"][field] = serialized(wave_value)
                wave_metrics[field] = {"navigable_missing": n, "filled": f, "residual": r}
            day_metrics["wave_fields"] = wave_metrics

            for kind in ("cur", "wave", "wind"):
                src = source_dir / f"{date}_{kind}.json"
                dst = target_dir / src.name
                dst.write_text(json.dumps(payloads[kind], ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
                case_manifest["input_hashes"][src.name] = sha256(src)
                case_manifest["output_hashes"][src.name] = sha256(dst)
            case_manifest["per_file"][date] = day_metrics

        run_manifest["cases"][relative] = case_manifest

    (out_root / "unified_interpolated_data_manifest_v1.json").write_text(
        json.dumps(run_manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps({"policy": run_manifest["policy_id"], "output": str(data_root),
                      "cases": list(run_manifest["cases"])}, ensure_ascii=False))


if __name__ == "__main__":
    main()
