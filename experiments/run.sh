#!/usr/bin/bash
#
algo=$1
bench_prefix=$2
col=$3 # CHECK_COLLISIONS or IGNORE_COLLISIONS
echo $3
MEM_LIMIT=4000000
TIME_LIMIT=300
ulimit -t ${TIME_LIMIT} -v ${MEM_LIMIT} -m ${MEM_LIMIT}
GFOLDER="cmarrt-data/"
logfile=logs/${bench_prefix}_${algo}_$3
rm -f $logfile
echo "$GFOLDER$bench_prefix"
for SEL in `find $GFOLDER$bench_prefix*.exp`
do
    for rseed in 0 1 2 3 4 5 6 7 8 9
    do
        echo "\n>>>>> $SEL" >> $logfile
        cmd="time cmapf-solver -a $algo -e $SEL -G $GFOLDER --collisions $col -p 20 --heuristics SHORTEST_PATH --verbose false --rs $rseed"
        echo $cmd >> $logfile
        echo $cmd
        $cmd >> $logfile 2>&1
    done
done
echo "Results written to $logfile"
python parse_log.py $logfile > $logfile.csv
# echo "$bench;" `wc -l < $logfile.csv` >> ${success_rate_log}