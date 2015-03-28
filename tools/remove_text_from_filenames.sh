#!/bin/bash

delimiter=${1:?'Must specify the marker until text will be removed'}
directory=${2:-'~/'}

cd ${directory}

for filename in *
do
	new_filename=${filename#*${delimiter}}
	mv ${filename} ${delimiter}${new_filename}
done
