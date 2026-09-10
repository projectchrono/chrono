YAML schema for Chrono::FSI-TDPF solver specification {#YAML_schema_fsitdpf_solver}
=======================================

A Chrono YAML TDPF solver file defines the parameters needed to run a Chrono::FSI-TDPF simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- The `radiation` object specifying how the radiation force is evaluated.
- The `excitation` object specifying how the wave excitation force is evaluated.
- The `diagnostics` object specifying solver diagnostics output.

All objects listed below are optional; any object or property that is omitted keeps its default value.
A TDPF solver file containing only `chrono-version` is therefore valid and selects all defaults.
Enumeration values are read case-insensitively.

Note that a TDPF solver file specifies neither an integrator nor a linear solver. The fluid phase only contributes
applied forces on the FSI bodies, and the multibody system is advanced by the MBS solver, so those settings belong in
the [MBS solver file](@ref YAML_schema_mbs_solver). Likewise, output and run-time visualization settings belong in the
[FSI-TDPF simulation file](@ref YAML_schema_fsitdpf_simulation).

## Radiation force specification

The `radiation` object controls the evaluation of the radiation force, that is, the Cummins-equation memory term. The
infinite-frequency added mass from the HDF5 file is always applied through a Chrono `ChLoadHydrodynamics` and is not
configurable here.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `method` | Radiation force evaluation method | enum | `RIRF_CONVOLUTION`,`STATE_SPACE` | No | `RIRF_CONVOLUTION` |
| `truncation_time` | Time in seconds at which the retardation kernel is truncated; 0 uses the full kernel | double | -- | No | 0 |
| `smoothing` | Smoothing of the retardation kernel | object | -- | No | no smoothing |
| `taper` | Half-cosine taper applied near the end of the retardation kernel | object | -- | No | no taper |
| `state_space` | Settings for the state-space kernel fit | object | -- | No | see below |
| `diagnostics` | Radiation kernel diagnostics | object | -- | No | no export |

`RIRF_CONVOLUTION` convolves the retardation (impulse response) kernel from the HDF5 file with the body velocity
history. `STATE_SPACE` instead fits a state-space model to the kernel, trading a one-time fit for a per-step cost that
is independent of the kernel length.

The body velocity history is retained over the kernel length, so both the per-step cost and the memory footprint of
`RIRF_CONVOLUTION` scale with it, and the cost grows as the integration step is reduced. Setting `truncation_time`
shortens that history. This is only safe once the kernel has decayed, and how soon that happens is a property of the
body: the 0.12 m sphere of the `sphere_decay` example is spent after about 1 s and is truncated at 3 s, whereas the
5 m sphere of the `sphere_regular_waves` example still holds 16% of its kernel energy at 2 s and needs most of its
15 s kernel.

#### Kernel smoothing

The `smoothing` object is only valid with `method` `RIRF_CONVOLUTION`. It is useful when a kernel computed by a BEM
code carries numerical noise.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Smoothing filter applied to the kernel | enum | `NONE`,`SG`,`MOVING_AVERAGE` | No | `NONE` |
| `window_length` | Filter window length, in kernel samples | integer | -- | No | 5 |

`SG` selects a Savitzky-Golay filter. `window_length` is forced to be odd and at least 3; a warning is issued if the
requested value is adjusted.

#### Kernel tapering

The `taper` object is only valid with `method` `RIRF_CONVOLUTION`. It is useful when a kernel does not decay to zero at
its end, where the residual tail would otherwise be convolved with the entire velocity history.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `enabled` | Whether the taper is applied | boolean | -- | No | `true` |
| `start_fraction` | Taper start, as a fraction of the kernel length | double | -- | No | 0.8 |
| `end_fraction` | Taper end, as a fraction of the kernel length | double | -- | No | 1.0 |
| `final_amplitude` | Kernel scale at the end of the taper (0 forces the kernel to zero, 1 leaves it unchanged) | double | -- | No | 0.0 |

The presence of the `taper` object turns the taper on, so that only the fractions need be given. An explicit `enabled`
key takes precedence, which allows the settings to be kept on record while the taper is switched off.

Because smoothing and tapering operate on the retardation kernel, they have no meaning for the state-space
approximation. Combining either with `method` `STATE_SPACE` is reported as an error when the solver file is read.

#### State-space fit

The `state_space` object is used only with `method` `STATE_SPACE` and is ignored otherwise.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `max_order` | Maximum number of state-space modes per DOF pair | integer | -- | No | 10 |
| `r2_threshold` | Minimum coefficient of determination for accepting a fit | double | -- | No | 0.95 |
| `max_hankel_size` | Maximum Hankel matrix size used in the singular value decomposition | integer | -- | No | 200 |
| `r2_num_samples` | Number of subsamples used for the fit quality check | integer | -- | No | 50 |

#### Radiation diagnostics

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `export_csv` | Write a summary of the retardation kernel before and after smoothing/tapering | boolean | -- | No | `false` |

This has an effect only when kernel processing is actually active, that is, when `smoothing.type` is not `NONE` or the
taper is enabled; there is otherwise no processing pass to report on. One file per body, named
`rirf_body<N>_summary.csv`, is written to the directory given by `diagnostics.output_dir`, or to the current working
directory if that key is empty or absent. Each file tabulates the kernel before and after processing for the first
DOF pair only, as a check on the filter settings rather than a full dump of the kernels.

## Wave excitation force specification

The `excitation` object controls the evaluation of the wave excitation force. These settings have no effect in still
water, where the excitation force is identically zero and the excitation component is not created at all.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `method` | Wave excitation force evaluation method | enum | `AUTO`,`IRF_CONVOLUTION`,<br>`FREQUENCY_DOMAIN` | No | `AUTO` |
| `interpolation` | Interpolation of the excitation transfer function read from the HDF5 file | enum | `CARTESIAN`,`POLAR` | No | `CARTESIAN` |
| `truncation_time` | Time in seconds at which the excitation impulse response function is truncated; 0 uses the full kernel | double | -- | No | 0 |

`AUTO` selects `FREQUENCY_DOMAIN` for regular seas and for irregular seas carrying more than one wave heading, and
`IRF_CONVOLUTION` otherwise. Excitation IRF convolution is only valid for long-crested seas, since the IRF data in the
HDF5 file is tabulated per heading and cannot be convolved with a single kernel against the total wave elevation.

`CARTESIAN` interpolates the real and imaginary parts of the transfer function, which avoids phase-wrap artefacts.
`POLAR` interpolates magnitude and phase, matching legacy HydroChrono behavior.

Note that `truncation_time` applies to excitation IRF convolution only, and that the excitation IRF is two-sided
(the kernels distributed with Chrono are tabulated over a symmetric interval about zero), since the wave elevation
ahead of the body influences the force acting on it.

## Solver diagnostics specification

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `output_dir` | Destination directory for the radiation kernel CSV export | string | -- | No | empty, the current working directory |

This is currently consumed only by `radiation.diagnostics.export_csv`. It selects where those files are written, not
whether anything is written at all: left empty or absent, they go to the current working directory.

## Fluid phase stepping

Evaluating the TDPF hydrodynamic forces is an algebraic operation on the current solid state and the retained velocity
history, so the fluid phase has no time step of its own and none is specified in this file. The entire co-simulation
step (`time_step` in the [FSI simulation file](@ref YAML_schema_fsi_simulation)) is covered by a single force
evaluation, which is required rather than merely efficient: the solid state is refreshed once per co-simulation step,
so a sub-cycled fluid advance would record the same body velocity at several distinct times and corrupt the radiation
convolution history.

## Example

Below is an example of an FSI-TDPF solver configuration, spelling out every available setting at its default value:

\include data/yaml/fsi/sphere_regular_waves/tdpf_solver.yaml


## YAML schema

The YAML TDPF solver specification file must follow the ``data/yaml/schema/fsitdpf_solver.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsitdpf_solver.schema.yaml
