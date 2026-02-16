Rigid BCE Markers in Chrono::FSI SPH {#manual_fsi_rigid_bce_markers}
=====================================================================

\tableofcontents

Scope
-----

This page covers **rigid-body BCE markers only** (no deformable 1D/2D FEA BCE markers).

Why BCE quality matters
-----------------------

In SPH, field equations are evaluated through neighbor summations over a kernel support. Boundary (BCE) markers are
part of those same local sums near walls and rigid bodies. If the BCE cloud is poor, boundary particles see an
incomplete or biased neighbor set, which typically shows up as pressure noise, spurious penetration, or leakage.

### 1) Match BCE spacing to fluid spacing

- In Chrono SPH, the reference spacing is `d0` (`SPHParameters::initial_spacing`).
- Built-in BCE generators use this same spacing.
- If you provide custom BCE markers, keep nearest-neighbor spacing close to `d0` and avoid large holes.

Reason (SPH discretization): continuum integrals are approximated by particle sums weighted by particle volume. Large
spacing mismatch between fluid and boundary particles causes incorrect integration results.

### 2) Use enough BCE layers

Chrono uses:

- `h = d0_multiplier * d0`
- kernel support radius `r_support = h_multiplier * h`
- for default kernels (Cubic Spline, Wendland), `h_multiplier = 2`

So, for default kernels:

- `r_support = 2 * d0_multiplier * d0`

For boundary completeness, a practical requirement is that BCE thickness along the inward normal covers this support.
With layer spacing about `d0`, this gives:

- `num_bce_layers * d0 >= r_support`
- `num_bce_layers >= 2 * d0_multiplier`

Therefore, if `d0_multiplier < 1.5`, **at least 3 BCE layers** are needed.

This matches Chrono's built-in warning that fewer than 3 layers can cause insufficient kernel support and leakage
(`ChFsiFluidSystemSPH::CheckSPHParameters`). However, it is important to note that if `d0_multiplier` is greater than 1.5, there need to be more than 3 layers to provide full kernel support.

Built-in rigid BCE generation in Chrono
---------------------------------------

Chrono exposes two API levels over the same BCE generation machinery.

### A) High-level: `ChFsiProblemSPH::AddRigidBody(...)`

Recommended when using `ChFsiProblemSPH` / `ChFsiProblemCartesian`.

- `AddRigidBody(body, geometry, check_embedded, use_grid_bce=false)`
- convenience wrappers:
  - `AddRigidBodySphere(...)`
  - `AddRigidBodyBox(...)`
  - `AddRigidBodyCylinderX(...)`
  - `AddRigidBodyMesh(...)`

What happens internally:

Call chain:

1. `ChFsiProblemSPH::AddRigidBody(...)`
2. `ChFsiSystem::AddFsiBody(body, geometry, check_embedded)`
3. `ChFsiInterface::AddFsiBody(...)`
4. `ChFsiFluidSystemSPH::OnAddFsiBody(...)`
5. `ChFsiFluidSystemSPH::CreateBCEFsiBody(...)`

At step 5, Chrono generates BCE points from geometry with the same low-level helpers (`CreatePointsSphereInterior`,
`CreatePointsBoxInterior`, `CreatePointsCylinderInterior`, `CreatePointsMesh`), then transforms/attaches them to the
FSI rigid body.

During `Initialize()`, if `check_embedded=true`, SPH particles overlapping rigid volumes are pruned.

### B) Low-level: `ChFsiFluidSystemSPH::CreatePoints*` + `ChFsiSystemSPH::AddFsiBody(...)`

Useful when you want direct control over point generation and attachment.

Primitive helpers include:

- `CreatePointsBoxInterior/Exterior(...)`
- `CreatePointsSphereInterior/Exterior(radius, polar)`
- `CreatePointsCylinderInterior/Exterior(rad, height, polar)`
- `CreatePointsConeInterior/Exterior(...)`
- `CreatePointsTruncatedConeInterior/Exterior(...)`
- `CreatePointsMesh(...)`
- `CreatePointsBoxContainer(...)` (for boundary walls)

What happens internally:

- You generate points explicitly (typically by calling the same `CreatePoints*` helpers yourself).
- Then you attach those points with `ChFsiSystemSPH::AddFsiBody(body, bce, rel_frame, check_embedded)`.
- In this overload, Chrono does not regenerate points from geometry; it uses exactly the points you provided.
- The point-generation algorithm is therefore the same as high-level generation when you use the same helper
  functions:
  - layered shells from `num_bce_layers` and `d0`
  - polar sampling for round geometry (or Cartesian accept/reject)
  - mesh fill from Cartesian grid sampling with ray-parity inside tests

Custom rigid BCE marker workflows
---------------------------------

### 1) Provide your own point cloud file (CSV): Rassor drum demo

See `src/demos/fsi/sph/demo_FSI-SPH_RassorDrum.cpp`.

Pattern:

1. Read BCE coordinates from CSV into `std::vector<ChVector3d>`.
2. Attach to rigid body:

~~~{.cpp}
sysFSI.AddFsiBody(drum, BCE_drum, ChFrame<>(), true);
~~~

Use this when CAD or preprocessing already produced a validated BCE cloud.

### 2) Generate custom BCE points in code: Viper wheel demo

See:

- `src/demos/robot/viper/demo_ROBOT_ViperWheel_CRM_slope.cpp`
- `src/demos/robot/viper/viper_wheel.h`

Pattern:

1. Write a custom generator (e.g., `CreateWheelBCE(...)`) for wheel body + grousers.
2. Attach generated markers:

~~~{.cpp}
auto bce = CreateWheelBCE(..., spacing, false);
sysFSI.AddFsiBody(wheel, bce, ChFrame<>(ChVector3d(0,0,0), Q_ROTATE_Z_TO_Y), false);
~~~

For tire test rigs, you can wrap this in a callback (`ViperTireBCE`) and register with
`ChTireTestRig::RegisterWheelBCECreationCallback(...)`.

Minimal user checklist
----------------------

1. Set `sph_params.initial_spacing = d0`.
2. Build BCE markers with the same spacing (`d0`).
3. Set `sph_params.num_bce_layers` to satisfy support (3 is the practical minimum for common settings).
4. If a body starts inside fluid, use `check_embedded=true` so overlapping SPH particles are removed.
5. For mesh-based BCE, ensure watertight geometry (Chrono will throw on non-watertight interior flood-fill cases).

Notes:

- The spacing/layer rules above combine SPH literature guidance with Chrono's kernel/support implementation.
- Inference: the `num_bce_layers >= 2 * d0_multiplier` rule is a practical support-coverage estimate for default
  kernels in Chrono (`h_multiplier=2`).
