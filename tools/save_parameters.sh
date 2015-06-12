#!/usr/bin/env bash

output_file=${1:?'Must specify output file'}
wait_time=${2:-0}

sleep ${wait_time}
rosparam dump ${output_file}
