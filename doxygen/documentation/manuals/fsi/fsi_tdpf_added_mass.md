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

- Schur complement-based solvers (`APGD`, `BARZILAIBORWEIN`, `PSOR`) require the inverse of the total mass
  matrix. `ChLoadHydrodynamics` assembles the system mass matrix including the added mass blocks, inverts
  it, and provides the result to the system descriptor. This is done once, and repeated only if the size of
  the system problem changes. Other solvers need only the KRM block and skip this work entirely.
- \f$M + A_\infty\f$ must be positive definite. Added mass coefficients from a boundary element solution
  are symmetric only up to the accuracy of that solution, and heavily truncated or poorly converged
  coefficients can violate this. A singular or indefinite total mass matrix shows up as a diverging or
  immediately non-finite solution rather than as a solver error message.
