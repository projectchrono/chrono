Chrono::FSI-SPH Parameter Selection Guide {#manual_fsi_sph_parameter_selection}
===========================================================================

\tableofcontents

Scope
-----

This page is a practical guide for selecting SPH parameters in Chrono::FSI-SPH, with emphasis on:

- physical intuition behind each parameter
- exact API calls used to set parameters
- variable time-step mode and its interaction with MBS time stepping

This guide is based on:

- `src/chrono_fsi/sph/ChFsiFluidSystemSPH.h` and `ChFsiFluidSystemSPH.cpp`
- variable-step implementation in `src/chrono_fsi/sph/physics/SphFluidDynamics.cu` and `SphForceWCSPH.cu`
- usage patterns in `src/demos/fsi/sph/`

Quick Start (Recommended Workflow)
----------------------------------

Use this sequence first, then refine:

1. Choose `initial_spacing` (`d0`) from geometry scale.
   Smaller `d0` resolves fine geometry features better but increases computational cost significantly because the number of particles increases.
2. Choose `d0_multiplier` (kernel length ratio `h/d0`).
   Start with `1.2` to `1.3`.
3. Choose boundary support.
   Keep `num_bce_layers >= 3` for common kernels; increase if you increase `d0_multiplier`. See `manual_fsi_rigid_bce_markers` for more details.
4. Choose viscosity/boundary/kernel model.
   Start from demo values for your scenario.
5. Set time stepping.
   - Fixed step: choose conservative `SetStepSizeCFD` on your `ChFsiSystemSPH`/`ChFsiProblemSPH` object.
   - Variable step: enable `use_variable_time_step` and keep the fixed step as first-step seed.

API Map (How To Set Parameters)
-------------------------------

Preferred pattern: build one parameter object and apply once.

~~~{.cpp}
ChFsiFluidSystemSPH::SPHParameters sph_params;
sph_params.integration_scheme = IntegrationScheme::RK2;
sph_params.initial_spacing = 0.005;
sph_params.d0_multiplier = 1.2;
sph_params.kernel_type = KernelType::CUBIC_SPLINE;
sph_params.use_variable_time_step = true;
sysSPH.SetSPHParameters(sph_params);
~~~

Equivalent direct setters exist for many fields:

- `SetInitialSpacing`, `SetKernelMultiplier`, `SetNumBCELayers`
- `SetIntegrationScheme`, `SetKernelType`, `SetBoundaryType`, `SetViscosityType`
- `SetArtificialViscosityCoefficient`
- `SetShiftingMethod`, `SetShiftingXSPHParameters`, `SetShiftingPPSTParameters`, `SetShiftingDiffusionParameters`
- `SetConsistentDerivativeDiscretization`
- `SetNumProximitySearchSteps`, `SetUseVariableTimeStep`

For material/model context:

- CFD: `SetCfdSPH(const FluidProperties&)`
- CRM: `SetElasticSPH(const ElasticMaterialProperties&)`
- ISPH linear solve: `SetLinSolverParameters(const LinSolverParameters&)` (or `SetSPHLinearSolver`)

For high-level problem builders (`ChFsiProblemSPH` and derived classes), use:

- `fsi.SetSPHParameters(sph_params)` (forwarded to `ChFsiFluidSystemSPH`)

Parameter Cheat Sheet
---------------------

### A) Resolution and support

| Parameter | Physical intuition | Practical tuning direction | API |
|---|---|---|---|
| `initial_spacing` | Particle spacing `d0`; sets spatial resolution and particle count. | Decrease for sharper interfaces/contact details; increase for faster runs. | `SPHParameters::initial_spacing`, `SetInitialSpacing` |
| `d0_multiplier` | Kernel length ratio `h/d0`; controls neighbor support width. | Increase for smoother, more robust kernels; decrease for sharper features and less diffusion. | `SPHParameters::d0_multiplier`, `SetKernelMultiplier` |
| `kernel_type` | Kernel shape/support quality (pairing resistance, smoothness, support radius multiplier). | Wendland is often more pairing-robust for larger support; cubic spline is common baseline. | `SPHParameters::kernel_type`, `SetKernelType` |
| `num_bce_layers` | Boundary marker thickness for kernel support near walls/solids. | See `manual_fsi_rigid_bce_markers`.| `SPHParameters::num_bce_layers`, `SetNumBCELayers` |
| `num_proximity_search_steps` | Frequency of neighbor-list rebuild. | Lower value (e.g., `1`) for rapidly changing flow (if you are doing CFD); higher values reduce computational cost but risk stale neighbors. Tested upto 10 for CRM with no loss in accuracy. | `SPHParameters::num_proximity_search_steps`, `SetNumProximitySearchSteps` |
### B) Time integration and compressibility control

| Parameter | Physical intuition | Practical tuning direction | API |
|---|---|---|---|
| `integration_scheme` | Time integrator and solver family (explicit WCSPH vs ISPH). | Start with `RK2` for explicit runs ; switch to `SYMPLECTIC` if you desire energy conservation. | `SPHParameters::integration_scheme`, `SetIntegrationScheme` |
| `use_variable_time_step` | Lets fluid substep size follow instantaneous CFL and acceleration limits. | Enable if you want to automatically adjust time step based on SPH resolution and changing flow speeds; keep a conservative fixed seed step for first step. | `SPHParameters::use_variable_time_step`, `SetUseVariableTimeStep` |
| `max_velocity` | Velocity scale used for sound speed in CFD (`Cs = 10 * v_Max`). | Increase if peak flow speeds are underpredicted; too low can destabilize EOS pressure update. | `SPHParameters::max_velocity` |
| `eos_type` | Density-pressure law used in explicit WCSPH CFD (`TAIT` or `ISOTHERMAL`). | See dedicated EOS section below for model-specific recommendations. | `SPHParameters::eos_type` |
| `use_delta_sph` | Adds a diffusion term in the continuity equation to smooth density oscillations. | Enable for noisy WCSPH free-surface runs; disable for stricter sharpness. | `SPHParameters::use_delta_sph` |
| `delta_sph_coefficient` | Strength of delta-SPH density diffusion. | Increase to damp density noise; decrease to preserve sharper gradients. Typical starting point is `0.1`. | `SPHParameters::delta_sph_coefficient` |
| `density_reinit_steps` | Periodic density re-initialization interval in WCSPH. | Smaller values reinitialize more often (less drift, more damping); larger values preserve dynamics but may drift. | `SPHParameters::density_reinit_steps` |

### C) Dissipation, boundaries, and derivative operators

| Parameter | Physical intuition | Practical tuning direction | API |
|---|---|---|---|
| `viscosity_method` | Momentum diffusion model (`LAMINAR`, artificial unilateral/bilateral). | Laminar for physical viscosity-driven CFD; artificial terms for shock/contact stabilization and CRM practice. | `SPHParameters::viscosity_method`, `SetViscosityType` |
| `artificial_viscosity` | Strength of artificial dissipation. | Increase for stability/noise suppression; decrease to reduce overdamping. | `SPHParameters::artificial_viscosity`, `SetArtificialViscosityCoefficient` |
| `boundary_method` | Wall/BCE boundary treatment model (`ADAMI`, `HOLMES`). | Choose based on benchmark behavior for your case; keep consistent when comparing runs. | `SPHParameters::boundary_method`, `SetBoundaryType` |
| `use_consistent_gradient_discretization` | Enables consistent gradient operator correction. | Helps some laminar/accuracy-focused setups; may add cost/complexity. | `SPHParameters::use_consistent_gradient_discretization`, `SetConsistentDerivativeDiscretization` |
| `use_consistent_laplacian_discretization` | Enables consistent Laplacian operator correction. | Same tradeoff as above; typically set together with consistent gradient. | `SPHParameters::use_consistent_laplacian_discretization`, `SetConsistentDerivativeDiscretization` |

### D) Particle shifting controls

| Parameter | Physical intuition | Practical tuning direction | API |
|---|---|---|---|
| `shifting_method` | Re-distributes particles to reduce disorder (`NONE`, `XSPH`, `PPST`, `PPST_XSPH`, `DIFFUSION`, `DIFFUSION_XSPH`) which improves SPH function approximation. | Enable when there is high particle disorder or noise. See dedicated shifting section below. | `SPHParameters::shifting_method`, `SetShiftingMethod` |
| `shifting_xsph_eps` | XSPH blending factor (neighbor velocity smoothing). | Increase for smoother motion and less noise; too large can overdamp. | `SPHParameters::shifting_xsph_eps`, `SetShiftingXSPHParameters` |
| `shifting_ppst_push` | PPST repulsive component on over-compressed neighborhoods. | Increase to prevent clustering/penetration; too high can cause artificial expansion. | `SPHParameters::shifting_ppst_push`, `SetShiftingPPSTParameters` |
| `shifting_ppst_pull` | PPST attractive component in sparse neighborhoods. | Increase to fill voids and improve uniformity; too high can over-cluster. | `SPHParameters::shifting_ppst_pull`, `SetShiftingPPSTParameters` |
| `shifting_beta_implicit` | ISPH-specific shifting scale. | Increase to regularize implicit particle distribution; decrease to reduce artificial drift. | `SPHParameters::shifting_beta_implicit` |
| `shifting_diffusion_A` | Overall diffusion-shifting magnitude. | Increase for stronger particle regularization; decrease for less artificial transport. | `SPHParameters::shifting_diffusion_A`, `SetShiftingDiffusionParameters` |
| `shifting_diffusion_AFSM` | Diffusion-shifting limiter threshold parameter. | Tune with `AFST` to gate behavior near free-surface-like regions. | `SPHParameters::shifting_diffusion_AFSM`, `SetShiftingDiffusionParameters` |
| `shifting_diffusion_AFST` | Diffusion-shifting limiter threshold parameter. | Tune with `AFSM` to avoid aggressive shifting near interfaces. | `SPHParameters::shifting_diffusion_AFST`, `SetShiftingDiffusionParameters` |

### E) CRM-specific free-surface and ISPH solver controls

| Parameter | Physical intuition | Practical tuning direction | API |
|---|---|---|---|
| `free_surface_threshold` | Threshold for free-surface identification in CRM using position-field divergence. | Increase/decrease based on desired sensitivity of free-surface tagging. See dedicated free-surface section below. | `SPHParameters::free_surface_threshold` |
| `LinSolverParameters::{type, atol, rtol, max_num_iters}` | Pressure/linear solve behavior for ISPH. | Tighter tolerances improve accuracy but increase cost. | `SetLinSolverParameters`, `SetSPHLinearSolver` |

Equation of State (CFD Only) and Pressure Closure
-------------------------------------------------

In Chrono SPH, EOS is used only in explicit WCSPH CFD mode.

- The density update calls `rho_p.y = Eos(rho_p.x, eos)` in `DensityEulerStep(...)`.
- This path is used for CFD updates in `SphFluidDynamics.cu`.

In CRM mode, EOS is not used to compute pressure.

- Pressure is obtained from the stress state (mean stress), i.e. from the trace of the stress tensor:
  `p = -(1/3) tr(sigma)`.
- In the CRM stress update (`TauEulerStep(...)`), pressure is updated from stress integration and written to
  `rho_p.y`.

Implemented EOS forms in Chrono (`SphGeneral.cuh`):

- Tait EOS:
  - `p = B[(rho/rho0)^gamma - 1] + p_base`
  - `B = rho0 * Cs^2 / gamma`, with `gamma = 7` in the current implementation.
- Isothermal EOS (linearized form):
  - `p = Cs^2 * (rho - rho0)`

Practical recommendation for `TAIT` vs `ISOTHERMAL`:

- Use `TAIT` for strongly free-surface, impact, and sloshing-like CFD cases.
  - This is the default choice in many Chrono free-surface demos (`DamBreak`, `ObjectDrop`, `WaveTank`, etc.).
  - It is the standard WCSPH closure for weakly compressible liquid flows in SPH literature.
- Use `ISOTHERMAL` for low-speed, near-incompressible internal-flow CFD where a linear pressure-density response is
  sufficient and easier to tune.
  - In Chrono demos, `Poiseuille_flow` uses `ISOTHERMAL`.

delta-SPH: Density Diffusion (and Why It Also Reduces Pressure Noise)
---------------------------------------------------------------------

`use_delta_sph` and `delta_sph_coefficient` control a diffusion term in the continuity equation (see
`SphForceWCSPH.cu`, both CRM and CFD RHS paths).

- Direct effect: smooths density oscillations.
- Indirect CFD effect: because pressure is computed from EOS(`rho`), reducing density oscillations also reduces pressure
  oscillations.

So the primary target is density noise; pressure-noise reduction is a consequence of the EOS closure in WCSPH CFD.

In code, the diffusion contribution uses:

- `Psi ~ delta_sph_coefficient * h * Cs * (...) * (rho_i - rho_j)/(r^2 + eps h^2)`
- and is added to `drho/dt`.

This follows standard delta-SPH practice (e.g., Marrone et al., 2011).

Practical tuning:

- Start with `delta_sph_coefficient = 0.1`.
- Increase toward `0.15-0.2` if density/pressure spectra are noisy.
- Decrease toward `0.05` or disable if the solution is over-diffused.

Shifting Methods: What Each One Does
------------------------------------

Shifting modifies the particle transport velocity to keep particle distribution regular and maintain SPH accuracy.
Chrono computes a shifting velocity and adds it in the position update.

`NONE`

- No shifting. Maximum sharpness, lowest extra damping, highest risk of disorder in long or violent runs.

`XSPH`

- Velocity blending with neighbors:
  `v_shift ~ eps * m * sum[(v_j - v_i) W_ij / rho_bar_ij]`.
- Smooth, low-complexity regularization.
- Main control: `shifting_xsph_eps`.

`PPST` (pushing/pulling)

- Uses a fictitious-sphere penetration measure and applies:
  - push when local spacing is too small,
  - pull when local spacing is too large.
- Main controls: `shifting_ppst_push`, `shifting_ppst_pull`.

- `shifting_ppst_push`:
  repulsive gain for crowded neighborhoods.
  Increasing this reduces clustering/overlap faster, but too high can over-expand local particle spacing.
- `shifting_ppst_pull`:
  attractive gain for depleted neighborhoods.
  Increasing this fills voids faster, but too high can over-contract and add artificial transport.

- In PPST mode, Chrono uses fluid neighbors in the support domain for the shift accumulation.
- Typical CRM defaults follow the PPST paper values: `shifting_ppst_push = 3`, `shifting_ppst_pull = 1`.

`PPST_XSPH`

- Sum of PPST and XSPH contributions.
- Common robust option for CRM-type granular runs where strong distortion occurs.

`DIFFUSION`

- Uses a concentration-gradient-like correction:
  `inner_sum = sum[(m_j/rho_j) gradW_ij]`
  then
  `v_shift ~ -A h |v_i| inner_sum`.
- Uses limiter parameters `AFSM`, `AFST` to reduce over-shifting near under-supported regions.
- Main controls: `shifting_diffusion_A`, `shifting_diffusion_AFSM`, `shifting_diffusion_AFST`.

`DIFFUSION_XSPH`

- Sum of diffusion-based shifting and XSPH.

Practical selection:

- CFD free-surface baseline: `XSPH`.
- CFD with severe disorder/clustering: `DIFFUSION` or `DIFFUSION_XSPH`.
- CRM excavation/terramechanics baseline: `PPST_XSPH`.

Free-Surface Threshold in CRM
-----------------------------

`free_surface_threshold` is used in CRM to classify particles as near free surface based on support deficiency.

In `CrmRHS` (`SphForceWCSPH.cu`), Chrono computes:

- `nabla_r = sum_j (V_j * (-r_ij · gradW_ij))`, with `V_j = m_j / rho_j`.

Then:

- if `nabla_r < free_surface_threshold`, particle is flagged as free-surface-adjacent.

Physical interpretation:

- Interior particles with full 3D support have larger `nabla_r`.
- Particles missing neighbors (top boundary, excavated face, splash/front) have smaller `nabla_r`.

How this flag is used:

- In CRM stress update (`TauEulerStep`), near-surface particles have stress reset and pressure set to zero.
- This prevents unphysical tensile stress retention at the exposed surface.

Practical tuning:

- Default: `2.0`.
- Increase threshold:
  more particles treated as free surface (softer/less load-bearing surface, more damping of near-surface stress).
- Decrease threshold:
  fewer particles treated as free surface (stiffer surface response, but higher risk of spurious near-surface stresses).

Variable Time Step: What It Does and How It Is Computed
-------------------------------------------------------

CFL Link Between Initial Spacing and Time Step
----------------------------------------------

The SPH stability limit is tied to the smoothing length `h`, and `h` is tied to the initial spacing `d0`:

- `h = d0_multiplier * d0`
- `dt_CFL ~ C * h / (Cs + u_char)`

where `Cs` is the numerical sound speed and `u_char` is a characteristic relative particle speed.

In fixed-step mode (`use_variable_time_step = false`), you must enforce this manually through `SetStepSizeCFD(...)`:

- if you reduce `d0`, you should reduce `dt` proportionally (first estimate).
- example: if `d0` is halved and flow speeds are similar, start by halving `dt`.

In variable-step mode (`use_variable_time_step = true`), Chrono applies this relationship automatically at runtime because the computed step uses `h` directly (see formulas below). So when `d0` is reduced, the internal CFD substep decreases accordingly, except for the first seed step which is still provided by `SetStepSizeCFD(...)`.

Enable via:

~~~{.cpp}
sph_params.use_variable_time_step = true;
sysSPH.SetSPHParameters(sph_params);
// or:
sysSPH.SetUseVariableTimeStep(true);
~~~

Important behavior:

1. The first fluid step is still the fixed step (`SetStepSizeCFD(...)` on the owning FSI system/problem object).
   Reason: variable stepping uses state-derived quantities from previous force evaluation.
2. From the second frame onward, Chrono computes a new fluid step each CFD substep.

Current internal formula (per step):

- Per fluid particle:
  - `dt_cv_i = h / (Cs + max_vel_diff_i)`
  - `dt_acc_i = sqrt(h / |a_i|)`
- Global candidate:
  - `dt = 0.3 * min( min_i(dt_cv_i), min_i(dt_acc_i) )`

Where:

- `h = d0_multiplier * d0`
- `Cs = 10 * v_Max` is speed of sound set from `max_velocity` for CFD mode
- `Cs = sqrt(K_bulk / rho0)` is speed of sound set from `K_bulk` and `rho0` for CRM mode

How it enters FSI stepping:

- `ChFsiSystem::AdvanceCFD` repeatedly queries `GetCurrentStepSize()`.
- It enforces a small floor (`1e-6`) and clamps each CFD substep to the remaining communication interval.

Using Variable CFD Step with MBS Step
-------------------------------------

Chrono keeps CFD and MBS stepping separate inside each communication interval:

- CFD: variable internal substeps (if enabled).
- MBS: fixed substeps of `SetStepsizeMBD(...)` (unless you provide a callback).
- Coupling exchange: at each `fsi.DoStepDynamics(meta_time_step)` boundary.

Recommended pattern (used in demos):

~~~{.cpp}
double step_size_CFD = 1e-4;      // first-step seed for CFD
double step_size_MBD = 1e-5;      // often smaller for flexible solids
double meta_time_step = 5 * std::max(step_size_CFD, step_size_MBD);

fsi.SetStepSizeCFD(step_size_CFD);
fsi.SetStepsizeMBD(step_size_MBD);

ChFsiFluidSystemSPH::SPHParameters sph_params;
sph_params.use_variable_time_step = true;
fsi.SetSPHParameters(sph_params);

while (time < t_end) {
    fsi.DoStepDynamics(meta_time_step);   // CFD subcycles with variable h internally
    time += meta_time_step;
}
~~~

This is the same integration structure used in:

- `demo_FSI-SPH_WaveTank.cpp`
- `demo_FSI-SPH_ObjectDrop.cpp`
- `demo_FSI-SPH_RassorDrum.cpp`

When variable stepping is enabled, you can log step statistics with:

~~~{.cpp}
fsi.PrintStats();
fsi.PrintTimeSteps(out_dir + "/time_steps.txt");
~~~

Practical Starting Values
-------------------------

The following values are good baseline anchors (then tune one knob at a time):

- `d0_multiplier`: `1.2` to `1.3`
- `num_bce_layers`: `3` minimum for common kernels/support choices
- `integration_scheme`: `RK2` for explicit WCSPH baseline
- `kernel_type`: `CUBIC_SPLINE` baseline, `WENDLAND` if pairing robustness is needed
- `use_delta_sph = true`, `delta_sph_coefficient = 0.1` for many free-surface CFD runs
- `num_proximity_search_steps = 1` for strongly dynamic contact/impact runs
- `shifting_method = PPST_XSPH` for higher stability and more damping, `XSPH` for less damping, commonly used in CFD free-surface tank/drop setups.
- `viscosity_method = ARTIFICIAL_BILATERAL` if using CRM, `ARTIFICIAL_UNILATERAL` for CFD.
- `eos_type = TAIT`

Common Failure Modes and First Fix
----------------------------------

- Leakage near boundaries:
  increase `num_bce_layers`, check `d0_multiplier`, and avoid too-large neighbor-search interval.
- Pressure/velocity noise:
  increase `artificial_viscosity` slightly, enable/use `delta_sph`, or switch shifting method.
- Overdamped motion:
  reduce `artificial_viscosity` or weaker shifting.
- Instability after enabling variable step:
  reduce initial fixed `SetStepSizeCFD` seed and verify `max_velocity` is realistic.
- Pairing/clumping:
  reduce support (`d0_multiplier`) or switch kernel to `WENDLAND`.
