#!/usr/bin/env sh

error() {
    echo "Error: $1" >&2
    exit 1
}

clean_precice_logs() {
    (
        set -e -u
        cd "$1"
        echo "- Cleaning up preCICE logs in $(pwd)"
        rm -fv ./precice-*-iterations.log \
            ./precice-*-convergence.log \
            ./precice-*-watchpoint-*.log \
            ./precice-*-watchintegral-*.log \
            ./core
        rm -rfv ./precice-profiling/ profiling.json trace.json
        rm -rfv ./precice-exports/
    )
}

clean_case_logs() {
    (
        set -e -u
        cd "$1"
        echo "- Cleaning up general case logs in $(pwd)"
        CASENAME="$(readlink -f "$0" | xargs dirname | xargs basename)"
        rm -fv "./$CASENAME.log"
    )
}
