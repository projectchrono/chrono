#!/bin/bash
# If NO_RECURSE is not defined, keep going
if ! [ -z "${NO_RECURSE}" ]; then exit; fi
# Get the directory of this script
SCRIPTS_DIR=$(dirname "$0")
# dump to this file
LOG_FILE=$SCRIPTS_DIR/granular_metrics_log.txt
# Where are the tests
CHRONO_BUILD=/home/raid/chrono-dev/build
# Handy info to have
BREAK_STRING="<================ Running post-commit scripts, information is below: ================>"
LAST_COMMIT=$(git log -1 HEAD --pretty=medium --decorate=no)
CURR_DATE=$(date -R)

gitdir="$(git rev-parse --git-dir)"
hook="$gitdir/hooks/post-commit"

# disable post-commit hook temporarily
[ -x $hook ] && chmod -x $hook
export NO_RECURSE=1

echo $BREAK_STRING >> $LOG_FILE
echo "Current date is " $CURR_DATE >> $LOG_FILE
echo $LAST_COMMIT >> $LOG_FILE
echo "Results:" >> $LOG_FILE

$CHRONO_BUILD/bin/test_gran_milwave $LOG_FILE 
$CHRONO_BUILD/bin/test_gran_milsettle $LOG_FILE 

# amend last commit to include new results
git add $LOG_FILE
git commit --amend --no-edit

# Re-enable the hooks script now
chmod +x $hook
