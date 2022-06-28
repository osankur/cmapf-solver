set -e
# Number of instances per case
for i in {2..40}
do
	# Only start is window-connected
	python3 generate_exp.py -d cmarrt-data-wm1 -c graphs/pyramid.png_comm_uniform_grid_1_range_5.graphml -p graphs/pyramid.png_phys_uniform_grid_1_range_5.graphml -a $i -n 20 -o pyramid_range5_only-start-window2_agents${i} -wm 1 -ws 2
	# python3 generate_exp.py -d cmarrt-data-wm1 -p cmarrt-data/obstacle_field.png_phys_uniform_grid_1_range_5.graphml -c cmarrt-data/obstacle_field.png_comm_uniform_grid_1_range_5.graphml -a $i -n 20 -o obstacle_field_range5_window2_agents${i} -ws 2 -wm 1
	# Both are window-connected
	# python3 generate_exp.py -d cmarrt-data -p cmarrt-data/pyramid.png_comm_uniform_grid_1_range_5.graphml -c cmarrt-data/pyramid.png_phys_uniform_grid_1_range_5.graphml -a $i -n 10 -o pyramid_range5_window2_agents${i} -w 2
	# python3 generate_exp.py -d cmarrt-data -p cmarrt-data/obstacle_field.png_phys_uniform_grid_1_range_5.graphml -c cmarrt-data/obstacle_field.png_comm_uniform_grid_1_range_5.graphml -a $i -n 10 -o obstacle_field_range5_window2_agents${i} -w 2
	# python3 generate_exp.py -d cmarrt-data -p cmarrt-data/offices.png_phys_uniform_grid_1_range_5.graphml -c cmarrt-data/offices.png_comm_uniform_grid_1_range_5.graphml -a $i -n 20 -o offices_range5_window2_agents${i} -w 2	
done
