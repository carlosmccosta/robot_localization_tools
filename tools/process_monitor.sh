#!/bin/sh

process_name=${1:?'Must specify process to monitor'}
output_filename_perf=${2:-'~/'}
output_filename_collectl=${3:-'~/'}
sampling_interval=${4:-'0.1'}


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


echo "\n=========================================================================================================================================================="
echo "===== Saving system resources usage for process ${process_name} into ${output_filename_perf} and ${output_filename_collectl}"
echo "============================================================================================================================================================\n"

perf stat --pid=`pidof drl_localization_node` --scale --verbose --event=cpu-cycles,instructions,cache-references,cache-misses,branch-instructions,branch-misses,bus-cycles,stalled-cycles-frontend,stalled-cycles-backend,ref-cycles,cpu-clock,task-clock,page-faults,context-switches,cpu-migrations,minor-faults,major-faults,alignment-faults,emulation-faults,L1-dcache-loads,L1-dcache-load-misses,L1-dcache-stores,L1-dcache-store-misses,L1-dcache-prefetches,L1-dcache-prefetch-misses,L1-icache-loads,L1-icache-load-misses,L1-icache-prefetches,L1-icache-prefetch-misses,LLC-loads,LLC-load-misses,LLC-stores,LLC-store-misses,LLC-prefetches,LLC-prefetch-misses,dTLB-loads,dTLB-load-misses,dTLB-stores,dTLB-store-misses,dTLB-prefetches,dTLB-prefetch-misses,iTLB-loads,iTLB-load-misses,branch-loads,branch-load-misses,node-loads,node-load-misses,node-stores,node-store-misses,node-prefetches,node-prefetch-misses 2> ${output_filename_perf} &

collectl -F1 --interval ${sampling_interval}:${sampling_interval}:${sampling_interval} -o2cmUz --plot --subsys Z --procfilt p`pidof ${process_name}` --filename ${output_filename_collectl}
