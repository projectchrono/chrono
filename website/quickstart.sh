#!/bin/bash

# Launches api backend and jekyll site. API is placed in background but when jekyll is killed, APi also recieves kill signal 
# Written by Conlain Kelly for SBEl on 6/16/2016

api_run_path="metrics-database/backend/run.py"
api_python_path="metrics-database/backend/metricsAPI/bin/python"

trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
$api_python_path $api_run_path & jekyll serve