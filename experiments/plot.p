set term png size 600, 400 crop
set datafile separator ";"
set pointsize 2

cmarrt_filename="logs/obstacles_CMARRT_IGNORE_COLLISIONS.csv"
dfs_filename="logs/obstacles_dfs_IGNORE_COLLISIONS.csv"
set output "obstacles_IGNORE_COLLISIONS.png"
plot "< sort -n logs/obstacles_CMARRT_IGNORE_COLLISIONS.csv -k2 -t ';'" using ($3) title 'CMARRT' with lines,\
     "< sort -n ".dfs_filename." -k2 -t ';'" using ($3) title 'DFS' with lines
# plot "< sort -n ".cmarrt_filename." -k2 -t ';'" using ($2 > timeout ? 1/0 : $2) title 'CMARRT' with lines,\
#      "< sort -n ".dfs_filename." -k2 -t ';'" using ($2 > timeout ? 1/0 : $2) title 'DFS' with lines
