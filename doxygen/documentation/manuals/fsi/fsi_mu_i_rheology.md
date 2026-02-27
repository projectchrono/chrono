Chrono::FSI-SPH mu(I) Rheology Parameter Guide {#manual_fsi_mu_i_rheology}
=======================================================================

\tableofcontents

Scope
-----

This page is a practical guide for configuring the CRM `MU_OF_I` rheology in Chrono::FSI-SPH.

It focuses on:

- physical intuition for each mu(I) material parameter
- exact API calls used in Chrono
- code-level behavior of the return-mapping implementation
- recommended starting values based on validation and demo usage

Primary references:

- Original mu(I) paper: Jop, Forterre, Pouliquen (2006), "A constitutive law for dense granular flows"  
  https://arxiv.org/abs/cond-mat/0612110
- Chrono validation/examples paper: https://arxiv.org/abs/2507.05643
- Supplemental explainer: https://huzaifg.github.io/blog/2025/GranularFlow/
- Chrono implementation: `src/chrono_fsi/sph/physics/SphFluidDynamics.cu`, `src/chrono_fsi/sph/physics/SphForceWCSPH.cu`

Quick Start
-----------

1. Enable CRM and choose mu(I) rheology through `ElasticMaterialProperties`.
2. Set the core mu(I) material parameters:
   - `mu_fric_s`, `mu_fric_2`, `mu_I0`, `average_diam`, `cohesion_coeff`
3. Set SPH parameters following `manual_fsi_sph_parameter_selection`
   - `viscosity_method`, `artificial_viscosity`, shifting, `free_surface_threshold`

API Map (How To Set mu(I) Parameters)
-------------------------------------

Core API pattern:

~~~{.cpp}
ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
mat_props.rheology_model = RheologyCRM::MU_OF_I;

mat_props.density = 1700.0;
mat_props.Young_modulus = 1.0e6;
mat_props.Poisson_ratio = 0.3;

mat_props.mu_fric_s = 0.8;
mat_props.mu_fric_2 = 1.0;
mat_props.mu_I0 = 0.08;
mat_props.average_diam = 0.003;
mat_props.cohesion_coeff = 0.0;

sysSPH.SetElasticSPH(mat_props);
~~~

Solver/stabilization parameters (same SPH parameter block used in CFD/CRM following `manual_fsi_sph_parameter_selection`):

~~~{.cpp}
ChFsiFluidSystemSPH::SPHParameters sph_params;
sph_params.integration_scheme = IntegrationScheme::RK2;
sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
sph_params.artificial_viscosity = 0.2;
sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
sph_params.shifting_xsph_eps = 0.25;
sph_params.shifting_ppst_push = 3.0;
sph_params.shifting_ppst_pull = 1.0;
sph_params.free_surface_threshold = 2.0;
sysSPH.SetSPHParameters(sph_params);
~~~


mu(I) Math-to-Code Map
----------------------

Chrono's CRM mu(I) branch is implemented in `TauEulerStep(...)` (`SphFluidDynamics.cu`) and uses stress-rate terms from
`CrmRHS(...)` (`SphForceWCSPH.cu`).

| Concept | Implementation detail |
|---|---|
| Elastic trial stress | `new_tau = tau + dT * deriv_tau` in `TauEulerStep(...)` |
| Mean pressure | `p = -(1/3) tr(sigma)` |
| Deviatoric stress norm | `tau_bar = sqrt(0.5 * tau:tau)` |
| Shear-rate proxy | `Chi = abs(tau_tr - tau_n) * INV_G_shear / dT` |
| Inertial number | `I = Chi * d * sqrt(rho0 / (p_tr + 1e-9))` |
| Friction law | `mu = mu_s + (mu_2 - mu_s) * I / (I0 + I)` |
| Yield limit (with cohesion) | `tau_max = mu * p_tr + c` |
| Plastic correction | radial return when `tau_tr > tau_max` |
| Cohesive tension cutoff | if `p_tr < -c/mu_s`, stress is zeroed |
| Free-surface treatment | if flagged (`nabla_r < free_surface_threshold`), stress and pressure are zeroed |

Important model notes (current Chrono code behavior):

- EOS is not used in CRM; pressure comes from stress trace.

Core mu(I) Material Parameters
------------------------------

These are set through `ElasticMaterialProperties`.

| Parameter | Physical meaning | Code default | Practical recommendation | API |
|---|---|---|---|---|
| `density` | Bulk density scale for inertia, particle mass, and `I ~ d * sqrt(rho/p)`. | `1000` | Use measured bulk density for the soil state you want to reproduce (e.g., 1500-1800 kg/m^3 in many validations). | `mat_props.density` -> `SetElasticSPH` |
| `Young_modulus` | Elastic stiffness in the trial stress update (through `K` and `G`). | `1e6` | Start near `1e6` Pa for stable terramechanics baselines; increase if response is too soft, but expect smaller stable step sizes when stiffness rises. | `mat_props.Young_modulus` -> `SetElasticSPH` |
| `Poisson_ratio` | Sets volumetric vs shear elasticity split (`K/G`). | `0.3` | Keep around `0.25-0.35` for granular CRM unless you have calibrated test data. Avoid values too close to `0.5`. | `mat_props.Poisson_ratio` -> `SetElasticSPH` |
| `mu_fric_s` | Low-rate/static friction coefficient (`I -> 0`). Governs yield at slow shear. | `0.7` | Set from friction angle (`mu ~= tan(phi)`) when available. | `mat_props.mu_fric_s` -> `SetElasticSPH` |
| `mu_fric_2` | High-rate friction limit (`I` large). Controls rate strengthening. | `0.7` | Use `mu_fric_2 >= mu_fric_s`. If equal, rate dependence is effectively removed. Increase gap (`mu_2 - mu_s`) for stronger inertial hardening. | `mat_props.mu_fric_2` -> `SetElasticSPH` |
| `mu_I0` | Inertial-number transition scale between `mu_s` and `mu_2`. | `0.03` | Smaller `I0`: faster transition to high-rate friction. Larger `I0`: slower transition. Common values in demos/validation: `0.03-0.08`. | `mat_props.mu_I0` -> `SetElasticSPH` |
| `average_diam` | Grain-size scale in `I`. Larger `d` increases inertial effects for same shear/pressure state. | `0.005` | Use representative particle size from material data; this is one of the most sensitive calibration knobs. | `mat_props.average_diam` -> `SetElasticSPH` |
| `cohesion_coeff` | Cohesive intercept `c` in `tau_max = mu p + c`; adds shear strength at low pressure. | `0` | Start with `0` for dry cohesionless media; use positive values for weakly cemented/wet regolith-like behavior. | `mat_props.cohesion_coeff` -> `SetElasticSPH` or `SetCohesionForce` |
| `rheology_model` | Selects CRM constitutive law (`MU_OF_I` or `MCC`). | `MU_OF_I` | Set explicitly to `MU_OF_I` for clarity. | `mat_props.rheology_model` -> `SetElasticSPH` |

Derived elastic quantities (computed internally in `SetElasticSPH`):

- `G = E / (2(1+nu))`
- `K = E / (3(1-2nu))`


Parameter Selection Workflow
----------------------------

1. Set `density`, `Young_modulus`, `Poisson_ratio` from known material data (or best available estimates).
2. Set `mu_fric_s` from friction angle (`tan(phi)`), then choose `mu_fric_2`:
   - quasi-static dominated: start `mu_fric_2 ~= mu_fric_s`
   - rate-sensitive flow/impact: set `mu_fric_2 > mu_fric_s`
3. Tune `mu_I0` to control how quickly rate strengthening appears.
4. Set `average_diam` from representative grains/aggregate size used in calibration experiments.
5. Add `cohesion_coeff` only if low-pressure strength is under-predicted.
6. Stabilize numerics with `ARTIFICIAL_BILATERAL`, PPST/XSPH shifting, and `free_surface_threshold = 2.0`.


Common Failure Modes and First Fixes
------------------------------------

- Too much slip / under-predicted resistance:
  increase `mu_fric_s`, `mu_fric_2`, or `cohesion_coeff`; verify `average_diam`.
- Overly rate-sensitive response:
  reduce `mu_fric_2 - mu_fric_s` and/or increase `mu_I0`.
- Particle disorder or noisy stresses:
  use `PPST_XSPH`, increase `artificial_viscosity`, keep bilateral viscosity for CRM.
- Unstable runs:
  reduce time step seed (`SetStepSizeCFD`), enable `use_variable_time_step`, and keep `RK2`.
