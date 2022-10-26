set term png size 240, 140 crop

set datafile separator ";"
set pointsize 0.7

set xtics font ", 8"
set ytics font ", 8"
set key font ",8"
set yrange [0:100]
set format y "%.0f%%"
#set format y "%.0f%%"
set xrange [2:80]


     set key right top
files="offices parallel pyramid obstacles"
do for [i=1:words(files)] {
     if (i>1){
          set key off
     }
     # if (i==2 || i == 4) {
     #      set key right top;
     # }
     set output word(files, i)."_success.png"
     castar_filename="logs100/".word(files, i)."_success.csv"
     dfs_filename="logs_dfs/".word(files, i)."_success.csv"
plot castar_filename using (100*$3) title 'WHCA*' with linespoints ls 1,\
    dfs_filename using (100*$3) title 'DFS' with linespoints ls 2
}


set key right top
do for [i=1:words(files)] {
     if (i>1){
          set key off
     }
     set output word(files, i)."_100_300_success.png"
     castar100_filename="logs100/".word(files, i)."_success.csv"
     castar200_filename="logs200/".word(files, i)."_success.csv"
     castar300_filename="logs300/".word(files, i)."_success.csv"
plot castar100_filename using (100*$3) title '100' with linespoints ls 1,\
     castar200_filename using (100*$3) title '200' with linespoints ls 3,\
     castar300_filename using (100*$3) title '300' with linespoints ls 4
}


set key center top
set xtics 400
do for [i=1:words(files)] {
     if (i>1){
          set key off
     }
     set autoscale
     set format y "%.0fs"
     set output word(files, i)."_time.png"
     plot "< sort -n logs100/".word(files, i)."_length.csv -k4 -t ';'" using ($4) title 'WHCA*' with linespoints ls 1  ,\
          "< sort -n logs_dfs/".word(files, i)."_length.csv -k4 -t ';'" using ($4) title 'DFS' with linespoints ls 2  

     set autoscale
     set format y "%.0f"
     set output word(files, i)."_length.png"
     plot "< sort -n logs100/".word(files, i)."_length.csv -k3 -t ';'" using ($3) title 'WHCA*' with linespoints ls 1  ,\
          "< sort -n logs_dfs/".word(files, i)."_length.csv -k3 -t ';'" using ($3) title 'DFS' with linespoints ls 2  
}
