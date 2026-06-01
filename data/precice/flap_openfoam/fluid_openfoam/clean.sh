#!/usr/bin/env sh
set -e -u

echo "\n---- Cleaning up OpenFOAM preCICE participant ----\n"

. ../cleaning-tools.sh

rm -rfv 0/uniform/functionObjects/functionObjectProperties history

clean_precice_logs .
clean_case_logs .
