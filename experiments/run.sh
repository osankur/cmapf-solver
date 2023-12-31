#!/usr/bin/bash
#
algo=$1
bench_prefix=$2
col=$3 # CHECK_COLLISIONS or IGNORE_COLLISIONS
MEM_LIMIT=4000000
TIME_LIMIT=300
ulimit -t ${TIME_LIMIT} -v ${MEM_LIMIT} -m ${MEM_LIMIT}
GFOLDER="graphs/"
#EFOLDER="tmp-cmarrt-data-wm1/"
EFOLDER="cmarrt-data/"
logfile=logs200/${bench_prefix}_${algo}_$3
rm -f $logfile
echo "$EFOLDER$bench_prefix"
for SEL in `find $EFOLDER$bench_prefix*.exp`
do
    for i in {0..0}
    do
        echo '
        ' >> $logfile
        echo ">>>>> $SEL" >> $logfile
        #cmd="time cmapf-solver -a $algo -e $SEL -G $GFOLDER --collisions $col -p 33 --step_size 50 --heuristics SHORTEST_PATH --verbose false --rs $RANDOM"
        cmd="time cmapf-solver -a $algo -e $SEL -G $GFOLDER --collisions $col --nbst 200 --verbose false --subsolver CASTAR --rs $RANDOM"
        echo $cmd >> $logfile
        echo $cmd
        $cmd >> $logfile 2>&1
    done
done
echo "Results written to $logfile"
python parse_log.py $logfile > $logfile.csv
