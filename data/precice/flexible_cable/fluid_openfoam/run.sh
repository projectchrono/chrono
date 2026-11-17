#!/usr/bin/env bash
set -e

# OpenFOAM (interFoam) fluid participant, coupled with the Chrono solid participant in ../solid_chrono.
# Pass '-parallel' to run in parallel (see system/decomposeParDict).

blockMesh
checkMesh -constant
cp 0/alpha.water.orig 0/alpha.water
setFields

. "${WM_PROJECT_DIR}/bin/tools/RunFunctions"
foam_solver=$(getApplication)
if [ "${1:-}" = "-parallel" ]; then
    num_procs=$(getNumberOfProcessors)
    decomposePar -force
    mpirun -np "${num_procs}" "${foam_solver}" -parallel
    reconstructPar
else
    ${foam_solver}
fi
