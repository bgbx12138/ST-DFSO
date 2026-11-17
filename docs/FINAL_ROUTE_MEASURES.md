# Final-route measures

`data/results/tables/final_route_metrics.csv` contains the 360 final routes for ST-DFSO, ACO, IAFSA and window ILS across six scenarios, five objectives and three paired runs.

| Field | Meaning and unit |
|---|---|
| voyage_time_h | Evaluated voyage time, h |
| distance_n_mile | Evaluated route distance, nautical miles |
| energy_J | Energy, J; printed tables divide this value by 10¹¹ |
| mean_ship_motion_cost | Mean ship-motion cost per unit route length, dimensionless |
| cumulative_exposure | Cumulative exposure, cost-weighted nautical miles |
| composite_cost | Composite route cost, weighted nautical miles |
| paired_run | Run 1, 2 or 3; matches the route figure column |
| original_run_id / selected_run_id | Planned identity and selected accepted outcome |
| route_file | Relative path to the shared `data/results/routes.json` catalogue |
| route_key | Key in the catalogue's `routes` object; matches `original_run_id` |
| route_sha256 | SHA-256 of the canonical coordinate array, as defined in `RESULTS.md` |

The CSV retains full precision. These final-route measures are separate from the fixed-CPU comparisons. Distance is a reported measure, not an additional optimisation objective. No model evaluations or statistical tests were repeated to prepare this file.

To export stored measures to a new file, use `python analysis/export_route_metrics.py --output outputs/final_route_metrics.csv`. The exporter checks the compact outcome records and route coordinates and refuses an existing output. It does not need the detailed local `runs/` directory. Initial environmental inputs are not needed to read or export these results. New planning runs require the inputs described in `ENVIRONMENT_DATA.md`.
