#!/usr/bin/env sh
set -e -u

echo "\n---- Cleaning up Chrono preCICE participant ----\n"

. ../cleaning-tools.sh

clean_precice_logs .
clean_case_logs .
