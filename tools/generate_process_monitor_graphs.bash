#!/usr/bin/env sh

results_directory=${1:?'Must specify directory where the hw_resources_usage_log.prc file is and in which the results will be saved'}
file_name=${2:-'results_hw_resources_usage_log.prc'}
file_path=${results_directory}/${file_name}

echo "############################################################################################################################################################"
echo "##### Generating hardware resources usage graphs for file ${file_path}"
echo "############################################################################################################################################################\n"

mkdir -p "${results_directory}/pdf"
mkdir -p "${results_directory}/svg"
mkdir -p "${results_directory}/eps"

graphs_common_configs="-z sp -k 0.75 -r 1 -g 1 -s 1 -q 1 -d 0"
rosrun robot_localization_tools graph_plotter.py -i ${file_path} -o ${results_directory}/memory-usage -x 0 -y '9' -w 0.25 -m 1 -n 0.0009765625 -b 'Time of memory usage measurement' -v 'Resident memory (MB)' -l 'Memory usage' -c 'b' -t 'Memory usage' ${graphs_common_configs} &
rosrun robot_localization_tools graph_plotter.py -i ${file_path} -o ${results_directory}/processor-usage -x 0 -y '17' -w 0.25 -m 1 -n 1 -b 'Time of processor usage measurement' -v 'Processor usage [0..100 * number of cores]' -l 'Processor usage' -c 'b' -t 'Processor usage' ${graphs_common_configs} &


probability_distributions_csv="${results_directory}/probability_distributions_hw_usage_temp.csv"
echo -n "" > ${probability_distributions_csv}
probability_distributions_common_configs="-z sp -b -1 -n 100 -m -1 -l 11 -a 10 -w 0.25 -g 4 -r 1 -s 1 -q 1 -d 0 -f 0"
rosrun robot_localization_tools probability_distribution_plotter.py -i ${file_path} -o ${results_directory}/memory-usage-distributions -e 0.0009765625 -c 9 -t 'Probability distributions for memory usage' -x 'Memory usage bins (MB)' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &
rosrun robot_localization_tools probability_distribution_plotter.py -i ${file_path} -o ${results_directory}/processor-usage-distributions -c 17 -t 'Probability distributions for processor usage' -x 'Processor usage bins [0..100 * number of cores]' ${probability_distributions_common_configs} >> ${probability_distributions_csv} &

wait


probability_distributions_csv_final="${results_directory}/probability_distributions_hw_usage.csv"
echo -n "" > ${probability_distributions_csv_final}
echo "%file,norm_location,norm_scale,lognorm_location,lognorm_scale,lognorm_shape,genextreme_location,genextreme_scale,genextreme_shape" > ${probability_distributions_csv_final}
sort ${probability_distributions_csv} >> ${probability_distributions_csv_final}

rm -f ${probability_distributions_csv}


echo -e "\n"
echo "############################################################################################################################################################"
echo "##### Finished generating results for ${results_directory}"
echo "############################################################################################################################################################\n"
