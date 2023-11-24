#!/usr/bin/bash

# This is the set of prefix base names to be run
benchmarks=()
for i in {79..80}; do
	#benchmarks=("${benchmarks[@]}" "small_obstacles_3d_agents${i}_")
	benchmarks=("${benchmarks[@]}" "offices_range1_agents${i}_")
	#benchmarks=("${benchmarks[@]}" "parallel_range1_agents${i}_")
	#benchmarks=("${benchmarks[@]}" "pyramid_range3_3d_agents${i}_")
done
algo=$1 # CMARRT, COORD, DFS
if [[ $algo = "" ]]; then
	echo "Give algo"
	exit -1
fi
col=$2 # IGNORE_COLLISIONS or CHECK_COLLISIONS

# success_rate_log="logs/success_rate_${algo}_${col}.csv"
# rm -f ${success_rate_log}
for bench_prefix in ${benchmarks[*]}; do
		#cmd="./run.sh $algo $bench_prefix $col"
		cmd="./run.sh $algo $bench_prefix CHECK_COLLISIONS"
		echo $cmd
		$cmd
    	# file="logs/${bench}_${algo}_${col_short}"
		# python parse_log.py $file > $file.csv
		# echo "$bench;" `wc -l < $file.csv` >> ${success_rate_log}
done
