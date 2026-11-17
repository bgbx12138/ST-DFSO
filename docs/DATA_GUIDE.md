# Paper data guide

| Paper content | Repository data |
|---|---|
| Table 1: main comparisons at 60 CPU s | `data/results/tables/MAIN60_PUBLICATION15.csv` |
| Table 2: selected-objective final routes | `data/results/tables/FINAL_SELECTED_PUBLICATION15.csv` |
| Table 3: S6 length/motion/exposure decomposition | `data/results/tables/S6_DECOMPOSITION9.csv` |
| Figure 3: four CPU budgets | `data/results/tables/CPU_COMPARISONS160_PUBLICATION.csv`, rows with `family=main` |
| Figure 4: final ST-DFSO routes versus initial routes | `data/results/tables/STDFSO_FINAL_VS_INITIAL25_PUBLICATION.csv` |
| Figure 5 and complete route plots | `data/results/routes.json`, `config/seed_routes/`, `data/environment/` |
| Figure 6 and complete ablation comparisons | `data/results/tables/CPU_COMPARISONS160_PUBLICATION.csv`, rows with `family=ablation` |
| Complete cross-objective comparisons | `data/results/tables/FINAL_PAIRED75_PUBLICATION.csv` |
| All final-route measures for the four main algorithms | `data/results/tables/final_route_metrics.csv` |
| All 270 main-comparison length/motion/exposure pairs | `data/results/Supplementary_Data_S1.csv` |
| All 810 configurations, observations and outcomes | `data/results/accepted_runs.json` |

CSV versions of the three main-text tables, at displayed precision, are in `data/results/paper_tables/`. The full-precision files above also supply the complete supplementary results. Supplementary section and table numbers are not embedded in the exported records.

The objective identifiers are `TIME` (T), `FUEL` (E), `SAFETY_DENSITY` (D), `SAFETY_EXPOSURE` (R) and `COMPOSITE` (C). `FUEL` is an internal identifier for water-resistance energy in joules, not fuel mass. `FSA_SELECTED` identifies ST-DFSO; `ASV_IAFSA` identifies IAFSA; `WINDOW_ILS` identifies window ILS. `OFF_SF` is the complete ST-DFSO configuration; `MINUS_LL`, `MINUS_AV`, `MINUS_FG`, `MINUS_ER` and `CORE` are the ablation variants.

Figures 1–2 are algorithm schematics. Their drawn routes are illustrative rather than experimental observations. All recorded result-route coordinates are available in the route catalogue.
