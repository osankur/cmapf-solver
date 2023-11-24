cd ..
set -e
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p offices_range1_agents -l aamas/logs100/ -t length > aamas/logs100/offices_length.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p offices_range1_agents -l aamas/logs300/ -t length > aamas/logs300/offices_length.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p parallel_range1_agents -l aamas/logs100/ -t length > aamas/logs100/parallel_length.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p parallel_range1_agents -l aamas/logs300/ -t length > aamas/logs300/parallel_length.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p pyramid_range3_3d_agents -l aamas/logs300/ -t length > aamas/logs300/pyramid_length.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p pyramid_range3_3d_agents -l aamas/logs100/ -t length > aamas/logs100/pyramid_length.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p small_obstacles_3d_agents -l aamas/logs100/ -t length > aamas/logs100/obstacles_length.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p small_obstacles_3d_agents -l aamas/logs300/ -t length > aamas/logs300/obstacles_length.csv

# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p offices_range1_agents -l aamas/logs100/ -t success > aamas/logs100/offices_success.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p offices_range1_agents -l aamas/logs300/ -t success > aamas/logs300/offices_success.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p parallel_range1_agents -l aamas/logs100/ -t success > aamas/logs100/parallel_success.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p parallel_range1_agents -l aamas/logs300/ -t success > aamas/logs300/parallel_success.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p pyramid_range3_3d_agents -l aamas/logs300/ -t success > aamas/logs300/pyramid_success.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p pyramid_range3_3d_agents -l aamas/logs100/ -t success > aamas/logs100/pyramid_success.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p small_obstacles_3d_agents -l aamas/logs100/ -t success > aamas/logs100/obstacles_success.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p small_obstacles_3d_agents -l aamas/logs300/ -t success > aamas/logs300/obstacles_success.csv
python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p offices_range1_agents -l aamas/logs200/ -t success > aamas/logs200/offices_success.csv
python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p parallel_range1_agents -l aamas/logs200/ -t success > aamas/logs200/parallel_success.csv
python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p pyramid_range3_3d_agents -l aamas/logs200/ -t success > aamas/logs200/pyramid_success.csv
python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p small_obstacles_3d_agents -l aamas/logs200/ -t success > aamas/logs200/obstacles_success.csv

# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p offices_range1_agents -l aamas/logs100-no-shake/ -t success > aamas/logs100-no-shake/offices_success.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p parallel_range1_agents -l aamas/logs100-no-shake/ -t success > aamas/logs100-no-shake/parallel_success.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p pyramid_range3_3d_agents -l aamas/logs100-no-shake/ -t success > aamas/logs100-no-shake/pyramid_success.csv
# python make_plot.py -a CASTAR -c CHECK_COLLISIONS -p small_obstacles_3d_agents -l aamas/logs100-no-shake/ -t success > aamas/logs100-no-shake/obstacles_success.csv


# python make_plot.py -a dfs -c COL -p offices_range1_agents -l aamas/logs_dfs/ -t success -n 20  > aamas/logs_dfs/offices_success.csv
# python make_plot.py -a dfs -c COL -p parallel_range1_agents -l aamas/logs_dfs/ -t success -n 20  > aamas/logs_dfs/parallel_success.csv
# python make_plot.py -a dfs -c dfs -p pyramid_range3_3d_agents -l aamas/logs_dfs/ -t success -n 20  > aamas/logs_dfs/pyramid_success.csv
# python make_plot.py -a dfs -c dfs -p small_obstacles_3d_agents -l aamas/logs_dfs/ -t success -n 20  > aamas/logs_dfs/obstacles_success_dfs.csv
# python make_plot.py -a dfs -c COL -p offices_range1_agents -l aamas/logs_dfs/ -t length > aamas/logs_dfs/offices_length.csv
# python make_plot.py -a dfs -c COL -p parallel_range1_agents -l aamas/logs_dfs/ -t length > aamas/logs_dfs/parallel_length.csv
# python make_plot.py -a dfs -c dfs -p pyramid_range3_3d_agents -l aamas/logs_dfs/ -t length > aamas/logs_dfs/pyramid_length.csv
# python make_plot.py -a dfs -c dfs -p small_obstacles_3d_agents -l aamas/logs_dfs/ -t length > aamas/logs_dfs/obstacles_length_dfs.csv


