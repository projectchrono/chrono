Added Mass in Chrono::FSI-TDPF {#manual_fsi_tdpf_added_mass}
=====================================================================

\tableofcontents

## Scope

This page describes how infinite-frequency added mass is applied to the solid phase of a Chrono::FSI-TDPF
problem, how the added mass of several hydrodynamic bodies is coupled, and what the association between an
FSI body and its hydrodynamic coefficients depends on. The last point is the one most easily got wrong in a
model with more than one hydrodynamic body, because it is resolved by index and never by name.


## Why added mass is not applied as a force

The time-domain equation of motion for a floating body has the Cummins form

\f[
  \left(M + A_\infty\right)\ddot{x} + \int_0^t K(t-\tau)\,\dot{x}(\tau)\,d\tau + C x = f_{exc}
\f]

Only the retardation convolution and the excitation term are functions of the state history; the
infinite-frequency added mass \f$A_\infty\f$ multiplies the *current* acceleration. It therefore belongs on
the left-hand side, as a contribution to the system mass matrix, and cannot be evaluated as an external
force without already knowing the acceleration it is meant to produce.

Accordingly, the two are handled by different mechanisms:

- The retardation and excitation terms are ordinary forces. They are produced by the fluid solver and
  delivered to the solid phase over the FSI coupling interface, like any other fluid force.
- The infinite-frequency added mass is applied by [ChLoadHydrodynamics](@ref chrono::ChLoadHydrodynamics),
  a `ChPhysicsItem` that injects a KRM block contributing \f$A_\infty\f$ to the system mass matrix and
  that implements the corresponding `IntLoadResidual_Mv` product.

`ChLoadHydrodynamics` is created automatically: by
[ChFsiSystemTDPF::Initialize](@ref chrono::fsi::tdpf::ChFsiSystemTDPF::Initialize) for a monolithic FSI
problem, and by the Chrono preCICE MBS adapter for a co-simulation, where the `added_mass` entry of the
participant specification either names the hydrodynamic data file to read them from or gives the blocks
explicitly.


## Coupling between hydrodynamic bodies

Added mass couples the hydrodynamic bodies to one another. Accelerating one body accelerates the fluid
around it, which exerts forces on every other body in the same fluid domain. The added mass block of a
model with `N` hydrodynamic bodies is therefore not block-diagonal: each body contributes a `6 x 6N` row
block, made up of its own `6 x 6` self added mass together with the `6 x 6` cross-coupling to each of the
other bodies.

In the HDF5 hydrodynamic data file these appear as `/bodyN/hydro_coeffs/added_mass/inf_freq`,
non-dimensionalized by the density in `/simulation_parameters/rho`. The RM3 two-body point absorber shipped
in `data/fsi-tdpf/rm3/rm3.h5` is an example: `/body1` is the float and `/body2` the reaction plate, and each
carries a `6 x 12` block whose second half is the coupling to the other hull.

A single-body model is the degenerate case of this, with one `6 x 6` block.


## Frame convention and reference point {#manual_fsi_tdpf_added_mass_frame}

The 6 x 6 block coupling two bodies relates the force and moment on one body to the acceleration of the
other. Which accelerations and which axes the coefficients refer to is a convention of the source of the
coefficients, and must be stated explicitly, because Chrono's own velocity coordinates for a body are the
velocity of its center of mass (COM) in absolute axes and its angular velocity in the axes of the COM frame.

[ChLoadHydrodynamics](@ref chrono::ChLoadHydrodynamics) interprets the coefficients of each body as
referring to:

- the velocity of the origin of the **body reference frame**, and
- the body angular velocity,

both expressed in the axes selected with
[SetAddedMassFrame](@ref chrono::ChLoadHydrodynamics::SetAddedMassFrame):

- `INERTIAL` (default): inertial (absolute) axes. This is the convention of coefficients computed by a
  boundary element (BEM) code, which are obtained for the body at its reference position and orientation.
- `BODY`: the axes of the body reference frame, for coefficients that rotate with the body.

The body reference frame is the one in which the body is defined: the COM frame for a
[ChBody](@ref chrono::ChBody), and the auxiliary reference frame for a
[ChBodyAuxRef](@ref chrono::ChBodyAuxRef), relative to which the COM frame is specified.

`ChLoadHydrodynamics` transforms the coefficients to Chrono's velocity coordinates using the current
orientation of each body. For body \f$i\f$, let \f$R_i\f$ and \f$Q_i\f$ be the orientations of its COM and
reference frames, and \f$d_i\f$ the position of the COM relative to the origin of the reference frame, in
absolute axes. The velocities the coefficients refer to are \f$T_i\,[v_i;\,\omega_i]\f$, where \f$v_i\f$
is the COM velocity in absolute axes, \f$\omega_i\f$ the angular velocity in the axes of the COM frame, and

\f[
  T_i^{\mathrm{inertial}} = \begin{bmatrix} I & \tilde{d}_i R_i \ 0 & R_i \end{bmatrix}, \qquad
  T_i^{\mathrm{body}} = \begin{bmatrix} Q_i^T & 0 \ 0 & Q_i^T \end{bmatrix} T_i^{\mathrm{inertial}}.
\f]

For a `ChBody`, \f$d_i = 0\f$ and \f$Q_i = R_i\f$, so that \f$T_i^{\mathrm{inertial}} = \mathrm{diag}(I, R_i)\f$
and \f$T_i^{\mathrm{body}} = \mathrm{diag}(R_i^T, I)\f$.

The block coupling bodies \f$i\f$ and \f$j\f$ in the system mass matrix is then
\f$T_i^T A_{ij} T_j\f$. The added mass is constant in the selected axes; in Chrono's velocity coordinates,
it varies with the body orientation. The velocity-dependent terms that arise from this variation (for
example the Munk moment) are not included. This is consistent with the linear potential-flow theory from
which the coefficients are typically obtained, which assumes small motions about the reference position.

### Coefficients about other points

Coefficients computed about a point other than the origin of the body reference frame must be shifted to
that origin before they are passed to `ChLoadHydrodynamics`.
[ComputeShiftTransformation](@ref chrono::ChLoadHydrodynamics::ComputeShiftTransformation) returns, for a
point \f$p_i\f$ given in the body reference frame, the transformation

\f[
  S_i = \begin{bmatrix} I & -\tilde{e}_i \ 0 & I \end{bmatrix},
\f]

where \f$e_i\f$ is \f$p_i\f$ expressed in the axes of the frame convention (\f$e_i = Q_i p_i\f$ for inertial
axes, \f$e_i = p_i\f$ for body axes), and
[ShiftBlocks](@ref chrono::ChLoadHydrodynamics::ShiftBlocks) replaces each block \f$A_{ij}\f$ with
\f$S_i^T A_{ij} S_j\f$. For body axes the shift is exact. For inertial axes it depends on the body
orientation and is computed once, at the initial orientation; the coefficients are then constant about the
origin of the body reference frame rather than about the original point, which differs only at second order
in the motion amplitude, beyond the reach of linear potential-flow theory.

A hydrodynamic data file does not record the point about which its coefficients were computed (the BEM
rotation center), and this point need not be the center of gravity stored in the file. It is given:

- for a monolithic FSI problem, with the `h5_reference_points` list of the TDPF model specification (or
  [ChFsiSystemTDPF::SetHydroReferencePoints](@ref chrono::fsi::tdpf::ChFsiSystemTDPF::SetHydroReferencePoints)),
  one point per body in the order of the bodies in the hydrodynamic file;
- for a preCICE co-simulation, with the `reference_points` list of the `added_mass` entry of the solid
  participant specification (or `ChPreciceAdapterMbs::SetAddedMassReferencePoints`), one point per coupling
  body.

In both cases the points are expressed in the body reference frame, and default to its origin (no shift).
The simplest setup avoids the shift altogether: model each hydrodynamic body as a `ChBodyAuxRef` whose
reference frame is placed at the point about which the coefficients were computed, with the COM specified
relative to it, as in the BEM model itself.

Coefficients read from a hydrodynamic data file are always taken to be in inertial axes. Coefficients given
explicitly in a preCICE participant specification may be declared in body axes instead, with
`frame: BODY`.


## Ordering of the FSI bodies

**The association between an FSI body and its hydrodynamic coefficients is by index.** The `i`-th
hydrodynamic body registered corresponds to `/body{i+1}` in the HDF5 file, and the `i`-th group of six
columns of every added mass block refers to that same body. Nothing is matched by name, so an ordering
mistake is silent: the simulation runs and produces a plausible-looking but wrong answer.

The order is fixed by:

- the sequence of [ChFsiSystem::AddRigidBody](@ref chrono::fsi::ChFsiSystem::AddRigidBody) calls, for a
  monolithic FSI problem;
- the order of the `bodies` sequence in the participant specification files, for a preCICE co-simulation.
  Both participants exchange data by coupling-mesh vertex index, so the solid and fluid participants must
  list their bodies in the same order, and that order must match the body order in the hydrodynamic file.

See `data/precice/rm3_reg_waves` for a two-body example in which this constraint is stated in all three
specification files.


## Position of the hydrodynamic bodies within the multibody system

A model may contain bodies with no hydrodynamics, and those bodies may be created before, after or in
between the hydrodynamic ones. None of this affects the result: the hydrodynamic bodies need not be the
first bodies of the multibody system, need not occupy contiguous system variable offsets, and their
relative order among the system variables need not match the order of the added mass blocks.

This is worth stating explicitly because the order in which bodies are created is often not the order in
which they were written down. [ChParserMbsYAML](@ref chrono::parsers::ChParserMbsYAML) holds the parsed
bodies in an unordered map, so for a YAML-specified model the creation order, and with it the system
variable offsets, follows hash order rather than the order of the `bodies` sequence in the model file, and
can change simply because a body was renamed.

`ChLoadHydrodynamics` consequently works in two distinct index spaces, and the distinction must be
respected by anyone modifying it:

- The KRM block it injects into the system descriptor covers only the hydrodynamic bodies. It is
  `6N x 6N` and is indexed in the order in which those variables were declared, that is, the order of the
  body blocks.
- The total mass matrix assembled for Schur complement-based solvers is system-wide, and is indexed by the
  system offsets reported by `ChBody::GetOffset_w()`.

Using system offsets for the KRM block gives the right answer only when the hydrodynamic bodies happen to
be the leading system variables, contiguous, and in block order. That is the case for a single-body model,
and for a model whose only non-hydrodynamic bodies are fixed (a fixed body contributes no variables) or are
created last, which is why the distinction is easy to overlook.


## Solver considerations

Added mass makes the mass matrix dense over the variables of the hydrodynamic bodies, and couples them to
each other. Two consequences are worth noting:

- Schur complement-based solvers (`APGD`, `BARZILAIBORWEIN`) require the inverse of the total mass matrix.
  `ChLoadHydrodynamics` assembles the system mass matrix including the added mass blocks, inverts it, and
  provides the result to the system descriptor. Since the added mass depends on the body orientations (see
  [Frame convention and reference point](@ref manual_fsi_tdpf_added_mass_frame)), this dense inversion is
  repeated whenever a hydrodynamic body rotates, as well as when the added mass coefficients or the size of
  the system problem change. Other solvers need only the KRM block and skip this work entirely. `PSOR` does
  not support the KRM block and cannot be used with added mass.
- \f$M + A_\infty\f$ must be positive definite. Added mass coefficients from a boundary element solution
  are symmetric only up to the accuracy of that solution, and heavily truncated or poorly converged
  coefficients can violate this. A singular or indefinite total mass matrix shows up as a diverging or
  immediately non-finite solution rather than as a solver error message.
