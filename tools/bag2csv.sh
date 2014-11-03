#!/bin/sh

##################################################
################### parameters ###################
##################################################

input_file=${1:?'Must specify bag file to convert'}
ignore_topics=${2:-''}


echo "####################################################################################################"
echo "##### Extracting data from: ${input_file}.bag"
if [ -n "${ignore_topics}" ] ; then echo "##### Ignoring topics: ${ignore_topics}" ; fi
echo "####################################################################################################\n"


for topic in `rostopic list -b ${input_file}.bag` ; do
	output_file="${input_file}_`echo ${topic} | tr '/' '_'`.csv"

	if echo ${ignore_topics} | grep -q ${topic} ; then
		echo "Ignoring topic ${topic}"
	else
		echo "Saving data from topic ${topic} into file ${output_file}"
		rostopic echo -p -b ${input_file}.bag ${topic} > ${output_file} &
	fi
done

wait

echo "\n####################################################################################################"
echo "##### Finished extraction of data from ${input_file}.bag"
echo "####################################################################################################\n"
