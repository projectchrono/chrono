#!/usr/bin/env sh
set -e -u

echo "---- Cleaning up Chrono preCICE participant ----"

. ../cleaning-tools.sh

rm -rfv ./results/

clean_precice_logs .
clean_case_logs .
