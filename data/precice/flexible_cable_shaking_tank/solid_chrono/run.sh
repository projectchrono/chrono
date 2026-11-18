#!/usr/bin/env bash
set -e

# Solid participant for the coupling with the OpenFOAM participant in ../fluid_openfoam.
# The Chrono::FSI-SPH coupling does not use this script (see ../README.md).
mbs_solver=../../../../Release/demo_PRECICE_flexible_cable_shaking_tank

${mbs_solver} -p Solid -f OpenFOAM "$@"
