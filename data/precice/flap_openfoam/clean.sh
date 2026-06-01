#!/usr/bin/env sh
set -e -u

echo "\n---- Cleaning up Chrono-OpenFOAM preCICE ----\n"

. cleaning-tools.sh

rm -rfv ./precice-run/
rm -fv ./*.log
rm -fv ./*.vtu

clean_precice_logs .

(cd "fluid_openfoam" && bash ./clean.sh)
(cd "solid_chrono" && bash ./clean.sh)
