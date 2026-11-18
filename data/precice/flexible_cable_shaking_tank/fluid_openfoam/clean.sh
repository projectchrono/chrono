#!/usr/bin/env sh
set -e -u

echo "---- Cleaning up OpenFOAM preCICE participant ----"

. ../cleaning-tools.sh

# Time directories (except 0), mesh, decomposition, and generated fields
find . -maxdepth 1 -type d -regex '\./[0-9.e+-]+' ! -name 0 -exec rm -rfv {} +
rm -rfv ./constant/polyMesh ./processor* ./postProcessing
rm -rfv ./0/uniform/functionObjects/functionObjectProperties history

clean_precice_logs .
clean_case_logs .
