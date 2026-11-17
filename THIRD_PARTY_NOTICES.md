# Third-party materials and attribution

The project [LICENSE](LICENSE) covers only material its contributors have the right to license. It does not replace upstream licences, claim ownership of published measurements, or restrict rights granted directly by other providers. Existing author and source notices in the code are retained.

## Software dependencies

Dependencies are installed by pip or Maven; their binaries are not bundled in Git. Their distributions supply their licences and notices. Transitive dependencies retain their own terms.

| Component | Licence / source |
| --- | --- |
| Capytaine 3.0.0 | [Apache License 2.0](https://github.com/capytaine/capytaine/blob/v3.0.0/LICENSE), specifically version 3.0.0 |
| NumPy | [BSD 3-Clause](https://github.com/numpy/numpy/blob/v2.0.2/LICENSE.txt) |
| SciPy | [BSD 3-Clause and bundled notices](https://github.com/scipy/scipy/blob/v1.13.1/LICENSE.txt) |
| pandas | [BSD 3-Clause](https://github.com/pandas-dev/pandas/blob/v2.3.2/LICENSE) |
| threadpoolctl | [BSD 3-Clause](https://github.com/joblib/threadpoolctl/blob/3.6.0/LICENSE) |
| psutil | [BSD 3-Clause](https://github.com/giampaolo/psutil/blob/master/LICENSE) |
| Jackson | [Apache License 2.0](https://github.com/FasterXML/jackson-databind/blob/2.18/LICENSE) |

## Environmental inputs

Processed files in `data/environment/` derive from the products in [ENVIRONMENT_DATA.md](docs/ENVIRONMENT_DATA.md). They retain provider terms; the project's non-commercial restriction is not the providers' restriction.

- Current and wave fields: Source: European Union, Copernicus Marine Service Information or Data (2026), © Mercator Ocean. Fields have been processed for this study. See the [Marine licence](https://marine.copernicus.eu/user-corner/service-commitments-and-licence) and [citation guidance](https://help.marine.copernicus.eu/en/articles/4444611-citing-copernicus-marine-products-and-services).
- Wind fields: Contains modified Copernicus Climate Change Service information (2026), from the ECMWF seasonal forecast product. Neither the European Commission nor ECMWF is responsible for any use that may be made of this information. See the [Copernicus Products licence](https://cds.climate.copernicus.eu/licences/licence-to-use-copernicus-products).
- Relief and derived land/navigability inputs: NOAA National Centers for Environmental Information, ETOPO Global Relief Model. The manuscript cites ETOPO 2022, [DOI:10.25921/fd45-gt74](https://doi.org/10.25921/fd45-gt74). Processed-file hashes identify the experiment inputs; they do not independently establish the edition of an absent original download receipt. See [NOAA ETOPO](https://www.ncei.noaa.gov/products/etopo-global-relief-model) and [data-use information](https://www.ngdc.noaa.gov/mgg/global/dem_faq.html).

## Ship-model measurements and coefficients

Numerical source values, units, conditions and extraction details are recorded in `config/validation/` and the model's parameter/source files. Primary records include the 5415M irregular-wave experiments reported by Sadat-Hosseini et al. (2015), the regular-wave and prescribed-control records of Stern and co-authors (including RTO-MP-AVT-189-8), and hydrodynamic data reported by Franceschi et al. (2021). Full bibliographic entries are in the manuscript and supplementary bibliography. No journal PDF or scanned page is included.

The small bichromatic arrays are numerical readings of published curves; receipts identify the source figure, calibration and precision. They are not original instrument data or exclusively owned ST-DFSO measurements. Cite the original records when reusing these values.

Gaussian Splatting, StyleGAN3 and DUSt3R are licence-policy examples in [LICENSING.md](docs/LICENSING.md); their code is not included here.

## Wind and propulsion parameters in the voyage model

- Blendermann (1993), *Schiffsform und Windlast*, Report 533: Table 4 destroyer coefficients and equations (61), (68). [Source report](https://tore.tuhh.de/entities/publication/a23d3a01-c309-43e1-9849-4d39de41392b).
- Patil et al. (2021), *Numerical simulation of ship navigation in rough seas based on ECMWF data*: Table 1 DTMB 5415 projected areas and propeller diameter. [doi:10.21278/brod72102](https://doi.org/10.21278/brod72102).
- Sadat-Hosseini et al. (2015), *CFD and Potential Flow Simulations of Fully Appended Free Running 5415M in Irregular Waves*: Table 3 propeller thrust coefficients.

The code uses these published numerical parameters with the assumptions in [VOYAGE_SPEED.md](docs/VOYAGE_SPEED.md); it does not claim to reproduce each source's complete model. Source PDFs are not bundled in this repository.
