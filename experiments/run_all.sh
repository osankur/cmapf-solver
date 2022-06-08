#!/usr/bin/bash

# Usage examples:
#   run_all.sh CMARRT IGNORE_COLLISIONS
#   run_all.sh DFS IGNORE_COLLISIONS
#   run_all.sh CMARRT CHECK_COLLISIONS
#   ...
#

# This is the set of prefix base names to be run
benchmarks=()
for i in {2..2}; do
	benchmarks=("${benchmarks[@]}" "obstacle_field_range5_window2_agents${i}_")
done
algo=$1 # CMARRT, COORD, DFS
if [[ $algo = "" ]]; then
	echo "Give algo"
	exit -1
fi
col=$2 # IGNORE_COLLISIONS or CHECK_COLLISIONS

# success_rate_log="logs/success_rate_${algo}_${colshort}.csv"
# rm -f ${success_rate_log}
for bench_prefix in ${benchmarks[*]}; do
		cmd="./run.sh $algo $bench_prefix $col"
		echo $cmd
		$cmd
    	# file="logs/${bench}_${algo}_${col_short}"
		# python parse_log.py $file > $file.csv
		# echo "$bench;" `wc -l < $file.csv` >> ${success_rate_log}
done
