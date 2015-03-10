#!/bin/sh

process_name=${1:?'Must specify process to monitor'}
output_folder_path=${2:-'~/'}
sampling_interval=${3:-'0.1'}


process_not_running=1
while [ ${process_not_running} -eq 1 ]
do
	sleep 0.1
	case `pidof ${process_name}` in
		''|*[!0-9]*) ;;
		*)
		process_not_running=0
		;;
	esac
done


echo "\n===================================================================================================="
echo "===== Saving system resources usage for process ${process_name} into ${output_folder_path}"
echo "====================================================================================================\n"

collectl -F1 --interval ${sampling_interval}:${sampling_interval}:${sampling_interval} -o2cmUz --plot --subsys Z --procfilt p`pidof ${process_name}` --filename ${output_folder_path}
