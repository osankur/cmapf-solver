set term png size 350, 200 crop

set datafile separator ";"
set pointsize 0.7

set xtics font ", 8"
set ytics font ", 8"
set key font ",8"
set yrange [0:100]
set format y "%.0f%%"
#set format y "%.0f%%"
set xrange [2:80]

files="offices parallel pyramid obstacles"
### Rand vs no rand
     set key right top
do for [i=1:words(files)] {
     set output word(files, i)."_norand.png"
     castar_filename="logs100/".word(files, i)."_success.csv"
     dfs_filename="logs100-no-shake/".word(files, i)."_success.csv"
plot castar_filename using (100*$3) title 'w/ rand.' with linespoints ls 1,\
    dfs_filename using (100*$3) title 'w/out rand.' with linespoints ls 5
}
