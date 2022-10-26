set term png size 300, 200 crop
set datafile separator ";"
set pointsize 2
set yrange [0:1.1]
set xrange [0:40]

set key right bottom

set title "Obstacle Range 1 - CA*"
castar_filename="logs/obstacle_field_range1_CASTAR_CHECK_COLLISIONS.csv"
castar2_filename="logs2/obstacle_field_range1_CASTAR_CHECK_COLLISIONS.csv"
#castar_dfs_filename="logs/obstacle_field_range1_CASTAR_DFS_CHECK_COLLISIONS.csv"
set output "obstacle_field_range1_castar.png"
plot castar_filename using ($3) title 'CA* 1000 - 1000' with lines,\
    castar2_filename using ($3) title 'CA* 500 - 100' with lines
#    castar_dfs_filename using ($3) title 'CASTAR+DFS' with lines

set title "Offices Range 1 - CA* alone"   
castar_filename="logs/offices_range1_CASTAR_CHECK_COLLISIONS.csv"
castar2_filename="logs2/offices_range1_CASTAR_CHECK_COLLISIONS.csv"
set output "offices_range1_castar.png"
plot castar_filename using ($3) title 'CA* 1000 - 1000' with lines,\
     castar2_filename using ($3) title 'CA* 500 - 100' with lines#,\

set key left top
set autoscale
set output "offices_time.png"
set title "Cactus Plot of Execution Times"
plot "< sort -n logs/offices_range1_lengths.csv -k4 -t ';'" using ($4) title 'WHCA* w/ rand.' with linespoints ls 1


set autoscale
set output "offices_length.png"
set title "Cactus Plot of Total Execution Lengths"
plot "< sort -n logs/offices_range1_lengths.csv -k3 -t ';'" using ($3) title 'WHCA* w/ rand.' with linespoints ls 1

set key left top
set autoscale
set output "parallel_time.png"
#set title "Cactus Plot of Execution Times"
plot "< sort -n logs/parallel_range1_nbst100_length.csv -k4 -t ';'" using ($4) title 'WHCA* nbst100  w/ rand.' with linespoints ls 1

set key left top
set autoscale
set output "parallel_length.png"
#set title "Cactus Plot of Execution Times"
plot "< sort -n logs/parallel_range1_nbst100_length.csv -k3 -t ';'" using ($3) title 'WHCA* nbst100 w/ rand.' with linespoints ls 1


# cmarrt_filename="logs/obstacles_window2_CMARRT_IGNORE_COLLISIONS.csv"
# dfs_filename="logs/obstacles_window2_dfs_IGNORE_COLLISIONS.csv"

# set title "Obstacle Field: 2-window-connected, no collision constraints"
# set output "obstacles_window2_IGNORE_COLLISIONS.png"
# plot "< sort -n logs/obstacles_CMARRT_IGNORE_COLLISIONS.csv -k2 -t ';'" using ($3) title 'CMARRT' with lines,\
#      "< sort -n ".dfs_filename." -k2 -t ';'" using ($3) title 'DFS' with lines
# # plot "< sort -n ".cmarrt_filename." -k2 -t ';'" using ($2 > timeout ? 1/0 : $2) title 'CMARRT' with lines,\
# #      "< sort -n ".dfs_filename." -k2 -t ';'" using ($2 > timeout ? 1/0 : $2) title 'DFS' with lines

# set title "Obstacle Field: 2-window-connected, collision constraints"
# set output "obstacles_window2_CHECK_COLLISIONS.png"
# cmarrt_filename="logs/obstacles_window2_CMARRT_CHECK_COLLISIONS.csv"
# dfs_filename="logs/obstacles_window2_dfs_CHECK_COLLISIONS.csv"
# plot "< sort -n ".cmarrt_filename." -k2 -t ';'" using ($3) title 'CMARRT' with lines,\
#      "< sort -n ".dfs_filename." -k2 -t ';'" using ($3) title 'DFS' with lines

# set title "Pyramid: 2-window-connected, no collision constraints"
# cmarrt_filename="logs/pyramid_window2_CMARRT_IGNORE_COLLISIONS.csv"
# dfs_filename="logs/pyramid_window2_dfs_IGNORE_COLLISIONS.csv"
# set output "pyramid_window2_IGNORE_COLLISIONS.png"
# plot cmarrt_filename using ($3) title 'CMARRT' with lines,\
#      dfs_filename using ($3) title 'DFS' with lines

# set title "Pyramid: 2-window-connected, collision constraints"
# cmarrt_filename="logs/pyramid_window2_CMARRT_CHECK_COLLISIONS.csv"
# dfs_filename="logs/pyramid_window2_dfs_CHECK_COLLISIONS.csv"
# set output "pyramid_window2_CHECK_COLLISIONS.png"
# plot cmarrt_filename using ($3) title 'CMARRT' with lines,\
#      dfs_filename using ($3) title 'DFS' with lines



# set title "Pyramid: wm1-connected, no collision constraints"
# cmarrt_filename="logs-wm1/pyramid_wm1_CMARRT_IGNORE_COLLISIONS.csv"
# dfs_filename="logs-wm1/pyramid_wm1_dfs_IGNORE_COLLISIONS.csv"
# set output "pyramid_wm1_IGNORE_COLLISIONS.png"
# plot cmarrt_filename using ($3) title 'CMARRT' with lines,\
#      dfs_filename using ($3) title 'DFS' with lines

# set title "Obstacles: wm1-connected, no collision constraints"
# cmarrt_filename="logs-wm1/obstacles_wm1_CMARRT_IGNORE_COLLISIONS.csv"
# dfs_filename="logs-wm1/obstacles_wm1_dfs_IGNORE_COLLISIONS.csv"
# set output "obstacles_wm1_IGNORE_COLLISIONS.png"
# plot cmarrt_filename using ($3) title 'CMARRT' with lines,\
#      dfs_filename using ($3) title 'DFS' with lines

# set title "Obstacles: wm1-connected, collision constraints"
# cmarrt_filename="logs-wm1/obstacles_wm1_CMARRT_CHECK_COLLISIONS.csv"
# dfs_filename="logs-wm1/obstacles_wm1_dfs_CHECK_COLLISIONS.csv"
# set output "obstacles_wm1_CHECK_COLLISIONS.png"
# plot cmarrt_filename using ($3) title 'CMARRT' with lines,\
#      dfs_filename using ($3) title 'DFS' with lines


