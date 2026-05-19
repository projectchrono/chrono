#!/usr/bin/env bash
set -e -u

. ../../tools/log.sh
exec > >(tee --append "$LOGFILE") 2>&1

blockMesh

../../tools/run-openfoam.sh "$@"

. ../../tools/openfoam-remove-empty-dirs.sh && openfoam_remove_empty_dirs

close_log
