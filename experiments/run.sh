#!/usr/bin/sh
#
algo=$1
bench_prefix=$2
col_bool="true"
col=$3 # CHECK_COLLISIONS or IGNORE_COLLISIONS
MEM_LIMIT=4000000
TIME_LIMIT=300
# ulimit -t ${TIME_LIMIT} -v ${MEM_LIMIT} -m ${MEM_LIMIT}
GFOLDER="cmarrt-data/"
logfile=logs/${bench_prefix}_${algo}_$3
rm -f $logfile
echo "$GFOLDER$bench_prefix"
for SEL in `find $GFOLDER$bench_prefix*.exp`
do
    echo ">>>>> $SEL" >> $logfile
    cmd="time cmapf-solver -a $algo -e $SEL -G $GFOLDER --collisions $col -p 20 --heuristics SHORTEST_PATH --verbose false"
    echo $cmd
    $cmd >> $logfile 2>&1
done
echo "Results written to $logfile"
