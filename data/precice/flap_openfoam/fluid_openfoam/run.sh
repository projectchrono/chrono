#!/usr/bin/env bash
set -e

blockMesh

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
