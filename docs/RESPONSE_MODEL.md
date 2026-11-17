# Ship-response model

All route algorithms use `navigation_safety.unified_response.SafetyFunction`, exposed to Java by `navigation_safety/provider.py`. It receives wave height, period and direction, speed through water and the current-compensated ship heading for each subsegment.

The four components are roll-angle standard deviation, first-order wave-induced yaw-angle standard deviation, sway-velocity standard deviation and surge-velocity standard deviation. Roll uses Gaussian equivalent-linear damping with calibrated coefficients. Yaw variance integrates the first-order wave response over the wave spectrum. Sway and surge use fixed linear horizontal response equations with sampled wave and prescribed roll forcing. The time step is 0.125 s; the record lasts 8192 s and discards its first 512 s.

The weighted local motion cost is the sum of normalised component standard deviations, with weights `(6,5,4,4)/19`. The reference scales are 4 degrees, 1.59796433001491 degrees, 0.44687470279710395 m/s and 0.1859704277566732 m/s, respectively. R accumulates local cost over distance; D equals R divided by total route length. These costs do not estimate accident probability.

Roll damping follows the equivalent-linear method in [ITTC (2021)](https://www.ittc.info/media/9759/75-02-07-045.pdf). Roll calibration uses the published irregular-wave measurements in [Sadat-Hosseini et al. (2015)](https://doi.org/10.5957/WMTC-2015-152). Hull derivatives follow [Franceschi et al. (2021)](https://doi.org/10.3390/jmse9060665). The fixed controller gains P = -0.8 and D = -24.9 s approximate the published 5415M case 206 CFD/PF setting. Normalised motion priorities refer to [Zou et al. (2023)](https://doi.org/10.3390/app13169420).

Yaw here represents wave-induced fluctuations and excludes slow course-keeping excursions. Roll comparisons and two published yaw peak RAOs do not establish accuracy for every sway/surge response or routing condition. Hydrodynamic coefficients are nominal-speed approximations within the declared ±5% speed domain. The parameters and supported domain are specified in `config/unified_response.json` and `config/linear_response.json`.

Cache entries require exact physical-input and model-identity matches. An unsupported or failed response has no objective cost; it is not assigned zero cost or an accident penalty. The common response pool is bound to the current model and the six initial routes. The [voyage speed calculation](VOYAGE_SPEED.md) supplies consistent speed and timing to time, energy and response calculations.

To calculate the response at the four published irregular-wave conditions, run `python tools/run_validation.py --output outputs/response-validation`. It reports all roll, sway and surge differences. Measured total heading variability has a different definition from the wave-induced yaw metric and is not used as a matching validation target. Reference scales and their basis are in `config/response_scales.json`.
