Coupling Schemes and Time Stepping {#precice_coupling}
==========================================

\tableofcontents

## Scope

This page covers the parts of a coupled setup that live in the preCICE configuration rather than in any one
participant: how the coupling window relates to solver steps, how the ordering of participants interacts
with the instant at which data is sampled, and what the module does and does not support. The one-window
force lag described below is the most common cause of a coupled simulation that is stable in isolation but
grows without bound once coupled, so it is worth reading before setting up a new case.

## Time windows and solver steps

preCICE advances the coupled simulation in *time windows* of the size declared in its configuration. Within
a window each participant asks preCICE for the maximum step it may take, and may take a smaller one; the
adapter computes this in `GetSolverTimeStep`. If the step taken is smaller than the window, preCICE
sub-cycles: the loop runs several iterations before the window completes, and data is read and written on
each of them.

That is worth understanding, because sub-cycling does not mean the data changes at each sub-step. Within a
window the partner has not produced anything new, so what a participant reads on the second and later
sub-steps is what it read on the first. Sub-cycling refines the solver's own integration, not the coupling:
the exchange rate is the window size, and improving the accuracy of the coupling means shrinking the window,
not the solver step.

Of the three adapters, the MBS and SPH participants cap their step at their configured value and so will
sub-cycle whenever that value is smaller than the window. The TDPF participant always takes the whole
window in one call, because a time-domain potential flow solver has no internal step to refine.

## Explicit coupling and the one-window lag

The shipped examples all use `coupling-scheme:serial-explicit`, in which the two participants run in a fixed
order within each window: the one listed `first` runs, then the one listed `second`.

The subtlety is when data written by a partner becomes readable. preCICE associates the data a participant
writes during a window with the **end** of that window. A participant that samples at the start of the
window therefore receives what its partner produced during the *previous* window. For a fluid force applied
to a solid body, that means the force acting over a window was evaluated at a solid state one full window
old.

That lag is not a harmless loss of accuracy. For a restoring force, a force that lags the displacement it
responds to behaves like *negative* damping, injecting energy into the system at a rate proportional to the
window size. A case that should decay will instead grow, and refining the window only slows the growth.

The adapters expose the choice through
[CouplingReadTime](@ref chrono::ch_precice::ChPreciceAdapter::CouplingReadTime), set from the
`read_data_time` key of the participant YAML:

| Value | Data sampled at | When it is valid |
|-------|-----------------|------------------|
| `WINDOW_START` | beginning of the window (default) | always; carries the one-window lag |
| `WINDOW_END` | end of the window | only for the participant listed `second` in a serial scheme |

`WINDOW_END` works only because the `second` participant runs after the `first` within the same window, so
the data it wants already exists. Combining `participants first="Fluid" second="Solid"` with
`read_data_time: WINDOW_END` on the solid participant makes the force applied over a window derive from the
solid state at the beginning of *that* window, which removes the lag. This is the arrangement used by all
the shipped configurations.

The setting is checked during initialization against the coupling scheme found in the preCICE XML, and an
inconsistent combination is rejected with an explanation rather than producing a quietly wrong answer.
Requesting `WINDOW_END` without a serial scheme, or for the participant listed `first`, is an error.

A parallel explicit scheme, in which both participants run concurrently, is faster in wall-clock terms but
cannot avoid the lag: neither participant has produced the current window's data when the other needs it.
That trade is worth making only when the coupling is weak enough for the lag not to matter.

## Implicit coupling

Implicit schemes iterate within a window until the exchanged data converges, which requires participants to
save their state at the start of a window and restore it on each iteration. preCICE requests this through
the checkpoint mechanism, which the adapters surface as `OnWriteCheckpoint` and `OnReadCheckpoint`.

Only the MBS adapter implements those. Both fluid adapters throw if preCICE asks for a checkpoint, so an
implicit scheme cannot currently be used with a Chrono fluid participant. Coupling a Chrono multibody
participant implicitly to an external solver that supports checkpointing is not subject to that limitation.

## Added mass over the interface

When a fluid participant contributes added mass, the coefficients cannot be sent as a force: they multiply
acceleration and belong in the mass matrix. The module therefore carries them on dedicated meshes, declared
in the preCICE XML as `SolidAddedMass` and `FluidAddedMass` with an `am_coeffs` data block.

Two behaviors follow from those declarations, and the distinction is easy to miss:

- Declaring the two **meshes** is what enables added mass on the solid participant at all. It is a request
  for added mass, and the solid participant then requires an `added_mass` entry in its YAML naming the
  source of the coefficients. Nothing needs to be exchanged for this to take effect.
- Added mass becomes *dynamic* — updated every window over the interface — only if a participant also
  **provides** one of those meshes and the scheme declares an `am_coeffs` exchange. Each update carries one
  6x6 block per body and replaces that body's own block, leaving any body-to-body coupling terms as they
  were.

The coefficients are sampled at the same point in the window as the forces, so that a force and the added
mass it was evaluated with refer to the same instant.

## Controlling the simulated duration

An adapter terminates on the coupling status reported by preCICE, and nothing else. The duration is the
product of `time-window-size` and `max-time-windows` in the preCICE XML. The `end_time` in a participant
YAML is never read by an adapter, which is why the shipped participant files set it to `-1` and say so in a
comment. A configuration that declares no `max-time-windows` runs until preCICE's own default limit.

## Practical notes

**Gravity and other shared physics.** Each participant reads its own settings and nothing reconciles them.
Gravity in particular must be set identically in both participant files; a mismatch produces a plausible
but wrong result rather than an error.

**Matching the solver step to the window.** For the MBS participant, the integrator step from its solver
specification is honored, and the adapter advances by `min(step, remaining window)`. Setting it equal to the
preCICE window size makes each window exactly one integration step, which is the easiest arrangement to
reason about.

**Starting and cleaning up.** Participants rendezvous through the mechanism declared in the XML; with
`<m2n:sockets>` and an exchange directory they must be started from the same directory, and the first to
start waits. If one participant fails during startup the other waits indefinitely. Before re-running after a
failure, make sure no participant processes survive and remove the `precice-run` directory, or the next
attempt may hang during the handshake.
