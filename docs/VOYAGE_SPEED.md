# Wind-dependent voyage speed

## Sources and approximation

The longitudinal aerodynamic load follows Blendermann (1993), *Schiffsform und
Windlast*, Report 533, equation (61), equation (68), and Table 4 (destroyer):
transverse coefficient 0.85, frontal-area longitudinal coefficient 0.60 for
headwind and 0.65 for following wind, and cross-force parameter 0.65.
[Original report](https://tore.tuhh.de/entities/publication/a23d3a01-c309-43e1-9849-4d39de41392b).
This approach was also published as Blendermann (1994), *Parameter identification
of wind loads on ships*, doi:10.1016/0167-6105(94)90067-1.

Frontal and lateral projected areas (368.36 and 1143.43 m²) are borrowed from
the DTMB 5415 in Patil et al. (2021), Table 1,
[doi:10.21278/brod72102](https://doi.org/10.21278/brod72102).
The two 6.15 m propellers use the published 5415M open-water thrust curve from
Sadat-Hosseini et al. (2015), Table 3, *CFD and Potential Flow Simulations of
Fully Appended Free Running 5415M in Irregular Waves*:

`KT(J) = 0.41249 - 0.31135 J - 0.13264 J²`.

These are approximate borrowed wind-load coefficients, not a newly calibrated
5415M wind-tunnel model. Air density is 1.225 kg/m³. The speed model assumes
quasi-steady longitudinal equilibrium, constant shaft revolutions, zero wake
fraction and zero thrust deduction. The nominal revolutions are obtained from
the reference-speed equilibrium in still air and still water; they are not
claimed to be the experimental MARIN shaft setting. `config/voyage_speed.json`
contains the parameters. No coefficient is fitted to algorithm-comparison results.

## Coupled calculation

Let `U` be speed through water, `Vg` speed along the ground track, and `h` the
water-relative unit heading. The prescribed track and current determine
`Vg = sqrt(U² - current_cross²) + current_along` and
`h = (Vg * track_unit - current_vector) / U`.
The apparent airflow is formed from the ground velocity and the true wind
vector. Blendermann's signed axial load is evaluated against `h`.

The implementation solves `T(U,n0) = Rwater(U) + Rair(U)` in the positive-thrust
quadrant, with `T = 2 rho_water n0² D⁴ KT(U/(n0 D))`.
The water-resistance proxy used for voyage energy supplies `Rwater`:
the friction coefficient is `0.075/(log10(Re)-2)² + 0.0004`, with the
pressure-drag term and estimated wetted area. The Reynolds number uses
speed through water directly; it is not multiplied by the cosine of the current
direction. A failed equilibrium or an impossible ground track is unavailable.
No lower speed is silently substituted.

Time uses `distance/Vg`. Energy remains the water-resistance-work measure
`Rwater(U) U dt`, not shaft energy, engine fuel mass or a complete energy balance.
At midnight the integration stops at the actual time boundary, takes the next
environmental field and solves again. The remaining distance is calculated
using the speed of the completed part. The response request receives the same
`U` and the current-compensated heading. Time and energy scales for the composite
objective use directional 1st/99th percentiles from this same model.

## Response coefficients and limits

The ship-response calculation uses actual `U` in encounter frequencies,
explicit speed-dependent damping, foil lift and response equations. Its
precomputed hydrodynamic coefficients remain those at the nominal route speed.
This is a local coefficient approximation, admitted only within ±5% of that
speed; the bound is a declared computational domain, not an experimentally
established accuracy range. The code rejects requests outside it. Source-anchor
validation retains the measured conditioning speed. Numerical residual gates
remain unchanged. Cache keys contain the exact physical inputs and model identity; responses from a different physical model are not reused.

This model describes direct longitudinal wind effects. It does not add a
wave-induced mean resistance model, wind leeway, acceleration transients or
prove accurate full-scale voyage-speed prediction. Waves still enter the
ship-motion objectives. No accident probability is inferred.

## Computational comparison

Every algorithm and ablation uses these identical calculations. The six common
input routes may be evaluated before the experiment to prepare one common exact
response pool; its preparation CPU is recorded separately. Candidate searches
cannot contribute to that initial pool. During the experiment all new response
calculations and final route checks are included in the measured process-tree
CPU. A route-cost observation is available at a budget only after its CPU upper endpoint falls
within that budget. Missing budget observations are not interpolated.
