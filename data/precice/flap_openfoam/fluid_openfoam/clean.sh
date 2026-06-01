#!/usr/bin/env sh
set -e -u

. ../cleaning-tools.sh

rm -rfv 0/uniform/functionObjects/functionObjectProperties history

clean_precice_logs .
clean_case_logs .
