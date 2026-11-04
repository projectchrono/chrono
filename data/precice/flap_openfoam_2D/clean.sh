#!/usr/bin/env sh
set -e -u

echo "---- Cleaning up Chrono-OpenFOAM preCICE ----"

. cleaning-tools.sh

rm -rfv ./precice-run/
rm -fv ./*.log
rm -fv ./*.vtu

clean_precice_logs .

(cd "fluid_openfoam" && bash ./clean.sh)
(cd "solid_chrono" && bash ./clean.sh)
