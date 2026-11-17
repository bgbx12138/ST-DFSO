# Environmental inputs included for the paper

The repository includes the 116 processed input files used by all six scenarios (five folders, approximately 16.19 MB). They match the recorded checksums in `config/environment_inventory.json`. After cloning, run `python tools/check_environment.py --historical-hashes`. No data-service registration or download is needed to run these scenarios. Raw NetCDF archives and credentials are excluded. Provider attribution is in [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md).

## Sources and variables

| Input | Source and product recorded in the preprocessing code | Variables |
|---|---|---|
| Current | [Copernicus Marine global physical forecast](https://documentation.marine.copernicus.eu/PUM/CMEMS-GLO-PUM-001-024.pdf), `cmems_mod_glo_phy-cur_anfc_0.083deg_P1D-m` | `uo`, `vo`; surface layer |
| Waves | [Copernicus Marine global wave forecast](https://data.marine.copernicus.eu/product/GLOBAL_ANALYSISFORECAST_WAV_001_027/description), `cmems_mod_glo_wav_anfc_0.083deg_PT3H-i` | `VHM0`, `VTPK`, `VMDR` |
| Wind | [Copernicus Climate Data Store seasonal original single levels](https://cds.climate.copernicus.eu/datasets/seasonal-original-single-levels?tab=overview), ECMWF system 51 | 10 m eastward and northward wind |
| Bathymetry/land | [NOAA ETOPO](https://www.ncei.noaa.gov/products/etopo-global-relief-model) | depth and derived navigability mask |

The wind utility uses the seasonal forecast product, not ERA5. Forecast initialisation, lead time and ensemble selection must be recorded when downloading. The exact ETOPO edition and source download receipt are not established by the generic product URL above; a new download must record its edition and checksum rather than claiming to be the recorded input file.

The six scenarios use 11–17 March 2026. The full file inventory, geographical bounds, grid sizes and recorded checksums are in `config/environment_inventory.json`. S1 and S5 share `S1_S5_taiwan`; other scenarios have separate folders. Daily files are named `YYYYMMDD_cur.json`, `YYYYMMDD_wave.json` and `YYYYMMDD_wind.json`, with an `obstacle_map.json` and the auxiliary files listed in the inventory. Wave/current inputs use the recorded daily sampling; retain units and wave-direction conventions. The Java environmental reader selects the field at the vessel's passage time.

## Optional preparation for different data

1. Create your own provider accounts and configure their clients outside this repository. Never commit tokens or credentials.
2. Read the inventory before downloading. The download utilities retain their original configurable regions and dates; set these explicitly for the required scenario rather than using their example defaults. Set `ST_DFSO_DOWNLOAD_DIR` to a local output directory.
3. Convert NetCDF fields with `tools/environment/nc_to_json.py`, supplying `--nc-dir`, `--date`, `--out-dir`, all geographical bounds, `--nx`, `--ny`, and the verified latitude order. `--out-flat` puts daily JSON files in one scenario directory. Do not infer grid orientation from a plot.
4. Prepare source missing-value masks with `build_2026_source_missing_masks_v1.py`. `build_unified_interpolated_inputs_v1.py` applies the recorded ocean-constrained missing-data policy to a separate output tree. It requires a source root and masks; it does not download or invent the original inputs. The published copy processes the five environmental folders shared by the six scenarios; the former application folder is excluded.
5. Supply bathymetry and the navigability mask at the same grid resolution. The inventory identifies the original files, but the release includes the processed paper masks, but does not claim a newly generated mask is byte-identical.
6. Place the prepared inputs under `data/environment/` and run `python tools/check_environment.py --scenario S1` (or select another scenario). `--historical-hashes` additionally requires every listed file to match the archived checksum. A difference denotes a new input version, not permission to overwrite the checksum list.

Forecast archive availability changes. The original file checksums identify the paper experiment; they do not guarantee that current downloads reproduce that issue. Neither the supplied paper experiments nor stored-result analysis requires downloading these inputs. A new environmental version must be reported as such.
