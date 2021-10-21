#!/usr/bin/sh
#
algo=$1
bench=$2
col_bool="true"
col=$3 # "col" or "nocol"
if [[ $col != "col" ]]; then
    col_bool="false"
    if [[ $col != "nocol" ]]; then
				echo "<$col>"
        exit -1
    fi
fi
MEM_LIMIT=4000000
TIME_LIMIT=2
DATA=data/
ulimit -t ${TIME_LIMIT} -v ${MEM_LIMIT} -m ${MEM_LIMIT}
logfile=logs/${bench}_${algo}_$3
rm -f $logfile
for SEL in `find $DATA$bench*.exp`
do
    echo ">>>>> $SEL" >> $logfile
		cmd="time ./console_main -a $algo -e $SEL -G $DATA -w 3"
		echo $cmd
		$cmd
    $cmd >> $logfile 2>&1
done
