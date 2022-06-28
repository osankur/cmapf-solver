set term png size 600, 400 crop
set datafile separator ";"
set pointsize 2
set yrange [0:1.1]

cmarrt_filename="logs/obstacles_window2_CMARRT_IGNORE_COLLISIONS.csv"
dfs_filename="logs/obstacles_window2_dfs_IGNORE_COLLISIONS.csv"

set title "Obstacle Field: 2-window-connected, no collision constraints"
set output "obstacles_window2_IGNORE_COLLISIONS.png"
plot "< sort -n logs/obstacles_CMARRT_IGNORE_COLLISIONS.csv -k2 -t ';'" using ($3) title 'CMARRT' with lines,\
     "< sort -n ".dfs_filename." -k2 -t ';'" using ($3) title 'DFS' with lines
# plot "< sort -n ".cmarrt_filename." -k2 -t ';'" using ($2 > timeout ? 1/0 : $2) title 'CMARRT' with lines,\
#      "< sort -n ".dfs_filename." -k2 -t ';'" using ($2 > timeout ? 1/0 : $2) title 'DFS' with lines

set title "Obstacle Field: 2-window-connected, collision constraints"
set output "obstacles_window2_CHECK_COLLISIONS.png"
cmarrt_filename="logs/obstacles_window2_CMARRT_CHECK_COLLISIONS.csv"
dfs_filename="logs/obstacles_window2_dfs_CHECK_COLLISIONS.csv"
plot "< sort -n ".cmarrt_filename." -k2 -t ';'" using ($3) title 'CMARRT' with lines,\
     "< sort -n ".dfs_filename." -k2 -t ';'" using ($3) title 'DFS' with lines

set title "Pyramid: 2-window-connected, no collision constraints"
cmarrt_filename="logs/pyramid_window2_CMARRT_IGNORE_COLLISIONS.csv"
dfs_filename="logs/pyramid_window2_dfs_IGNORE_COLLISIONS.csv"
set output "pyramid_window2_IGNORE_COLLISIONS.png"
plot cmarrt_filename using ($3) title 'CMARRT' with lines,\
     dfs_filename using ($3) title 'DFS' with lines

set title "Pyramid: 2-window-connected, collision constraints"
cmarrt_filename="logs/pyramid_window2_CMARRT_CHECK_COLLISIONS.csv"
dfs_filename="logs/pyramid_window2_dfs_CHECK_COLLISIONS.csv"
set output "pyramid_window2_CHECK_COLLISIONS.png"
plot cmarrt_filename using ($3) title 'CMARRT' with lines,\
     dfs_filename using ($3) title 'DFS' with lines



set title "Pyramid: wm1-connected, no collision constraints"
cmarrt_filename="logs-wm1/pyramid_wm1_CMARRT_IGNORE_COLLISIONS.csv"
dfs_filename="logs-wm1/pyramid_wm1_dfs_IGNORE_COLLISIONS.csv"
set output "pyramid_wm1_IGNORE_COLLISIONS.png"
plot cmarrt_filename using ($3) title 'CMARRT' with lines,\
     dfs_filename using ($3) title 'DFS' with lines

set title "Obstacles: wm1-connected, no collision constraints"
cmarrt_filename="logs-wm1/obstacles_wm1_CMARRT_IGNORE_COLLISIONS.csv"
dfs_filename="logs-wm1/obstacles_wm1_dfs_IGNORE_COLLISIONS.csv"
set output "obstacles_wm1_IGNORE_COLLISIONS.png"
plot cmarrt_filename using ($3) title 'CMARRT' with lines,\
     dfs_filename using ($3) title 'DFS' with lines

set title "Obstacles: wm1-connected, collision constraints"
cmarrt_filename="logs-wm1/obstacles_wm1_CMARRT_CHECK_COLLISIONS.csv"
dfs_filename="logs-wm1/obstacles_wm1_dfs_CHECK_COLLISIONS.csv"
set output "obstacles_wm1_CHECK_COLLISIONS.png"
plot cmarrt_filename using ($3) title 'CMARRT' with lines,\
     dfs_filename using ($3) title 'DFS' with lines


