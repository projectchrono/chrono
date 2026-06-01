#!/usr/bin/env sh
set -e -u

. cleaning-tools.sh

rm -rfv ./precice-run/
rm -fv ./*.log
rm -fv ./*.vtu

clean_precice_logs .

(cd "fluid_openfoam" && ./clean.sh)
(cd "solid_chrono" && ./clean.sh)

