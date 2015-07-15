#!/usr/bin/env bash

##################################################
################### parameters ###################
##################################################

input_file=${1:?'Must specify bag file to convert'}
topics_to_export=${2:-''}


echo "===================================================================================================="
echo "===== Extracting data from: ${input_file}.bag"
echo "===== Exporting topics: ${topics_to_export}"
echo "===================================================================================================="
echo -e "\n"


for topic in `rostopic list -b ${input_file}.bag` ; do
	output_file="${input_file}_`echo ${topic} | tr '/' '_'`.csv"

	if echo ${topics_to_export} | grep -q ${topic} ; then
		echo "Saving data from topic ${topic} into file ${output_file}"
		rostopic echo -p -b ${input_file}.bag ${topic} > ${output_file} &
	else
		echo "Ignoring topic ${topic}"
	fi
done

wait

echo -e "\n"
echo "===================================================================================================="
echo "===== Finished extraction of data from ${input_file}.bag"
echo "===================================================================================================="
echo -e "\n"
