#
# Usage example: run_all.sh COORD IGNORE_COLLISIONS (for self algorithm with collisions)
#
benchmarks=()
for i in {30..30}; do
#benchmarks=("${benchmarks[@]}" "open13_range150_${i}_")
#benchmarks=("${benchmarks[@]}" "offices11_range25_${i}_")
	benchmarks=("${benchmarks[@]}" "open13_range50_${i}__")
done
algo=$1 # COORD, MAS, CA, DFS
if [[ $algo = "" ]]; then
	echo "Give algo"
	exit -1
fi
collisions=nocol
col=IGNORE_COLLISIONS # or CHECK_COLLISIONS
if [ $col = "IGNORE_COLLISIONS" ]; then
	col_short=nocol
else
	col_short=col
fi
success_rate_log="logs/success_rate_${algo}_${colshort}.csv"
rm -f ${success_rate_log}
for bench in ${benchmarks[*]}; do
		cmd="./run.sh $algo $bench $col_short"
		echo $cmd
		$cmd
    file="logs/${bench}_${algo}_${col_short}"
		python parse_log.py $file > $file.csv
		echo "$bench;" `wc -l < $file.csv` >> ${success_rate_log}
done
