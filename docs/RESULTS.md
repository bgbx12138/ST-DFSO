# Recorded experiment results

`data/results/accepted_runs.json` contains all 810 distinct runs. Every planned configuration occurs exactly once. The main comparison uses 360 runs; the ablation analysis has 540 rows and shares 90 complete-ST-DFSO runs with the main comparison.

Each record includes its configuration, initial and final five-objective values, route length, complete-route observations, CPU-budget endpoints and measured process CPU. `route_key` selects its final coordinates from `data/results/routes.json`; `seed_route_file` identifies its common initial route. No local run-report directory is needed to analyse these records.

Coordinates are integer grid indices `i` and `j`. The environmental grids map them to geographical positions. `route_sha256` is SHA-256 of canonical UTF-8 JSON generated with `sort_keys=True, separators=(',', ':'), ensure_ascii=True, allow_nan=False`. Whole-file hashes are in `data/checksums.json`.

`experiment_record.json` identifies the execution order, source-run signature and physical-model definition. All observations and negative differences are retained. Machine-local trace paths are omitted from the public records.

## Statistics

```sh
python analysis/reproduce_results.py --output outputs/paper-statistics
```

The command reproduces the manuscript summaries from the recorded data. Positive percentage gains mean better ST-DFSO performance: `100 * (comparator - ST-DFSO) / comparator`. A symmetric tolerance, `abs(B-S) <= 1e-12 * max(abs(B),abs(S),1)`, identifies numerical ties for win/tie/loss counts and signed-rank tests. Percentage differences, means, medians and bootstrap intervals use unmodified costs. No extreme pair is excluded.

CPU comparisons use 18 matched pairs and two-sided Wilcoxon tests. Holm correction covers 15 main comparisons or 25 ablation comparisons separately at each budget. The mean's 95% interval uses 5,000 bootstrap resamples within scenarios with the recorded random seed. Final-route and cross-objective summaries are descriptive, separate from CPU endpoints.

The six output files under `data/` correspond to the publication summaries in `data/results/tables/`. `analysis/complete_set_analysis.py --output outputs/complete-analysis` exports the underlying 160 CPU comparisons, 2,880 paired CPU rows, 1,800 final-measure rows, 100 cross-objective summaries and 75 paired final comparisons. Use the publication reproduction command for the paper's common numerical-tie convention.

[Table and figure correspondence](DATA_GUIDE.md) · [Final-route units](FINAL_ROUTE_MEASURES.md)
