#!/usr/bin/env bash

if [ $# -ne 2 ]; then
    echo "usage: $0 <data_dir> <job_id>"
    exit 1
fi

cd $1
if [[ $? -ne 0 ]]; then
    echo "Invalid data_dir"
    exit 1
fi

id=$2
dir_prefix="cylinder_lift_${id}_"
raw2csv="${HOME}/chrono_granular/scripts/raw2csv_friction"
repose_angle="${HOME}/chrono_granular/scripts/repose_angle.py"

echo "cohesion,mu_s,mu_r,angle"
for d in ${dir_prefix}*; do
    cd $d
    # Get last data file
    last_file=$(ls | sort | tail -n3 | sed -E -e "/(meshframes|\.raw\.csv)/d" | tail -n1)
    $raw2csv $last_file ${last_file}.csv
    $repose_angle ${last_file}.csv $d
    cd ..
done
