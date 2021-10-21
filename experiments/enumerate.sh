N=30
for i in {2..30}
do
	FILE=logs_open_ccbs/open13_range150_${i}__irisa
	python parse_log.py $FILE > $FILE.csv
	echo "$FILE;col;"  `python success_count.py < $FILE.csv`
done
for i in {2..30}
do
	FILE=logs_open_ccbs/open13_range150_${i}__nocol_irisa
	python parse_log.py $FILE > $FILE.csv
	echo "$FILE;nocol;" `python success_count.py < $FILE.csv`
done


