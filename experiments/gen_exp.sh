set -e
# Number of instances per case
for i in {53..80}
do
	python3 generate_exp.py -d cmarrt-data -c graphs/pyramid.png_comm_uniform_grid_1_range_3_3d.graphml -p graphs/pyramid.png_phys_uniform_grid_1_range_3_3d.graphml -a $i -n 20 -o pyramid_range3_3d_agents${i}
	python3 generate_exp.py -d cmarrt-data -c graphs/small_obstacles.png_comm_range_2_dens15_3d.graphml -p graphs/small_obstacles.png_phys_range_2_dens15_3d.graphml -a $i -n 20 -o small_obstacles_3d_agents${i}	
	python3 generate_parallel_paths_exp.py -d cmarrt-data -p graphs/parallel_paths_sparse.png_phys_uniform_grid_1_range_4.graphml -c graphs/parallel_paths_sparse.png_comm_uniform_grid_1_range_4.graphml -a $i -n 20 -o parallel_range1_agents${i} -s 173 -g 203
	python3 generate_exp.py -d cmarrt-data -p graphs/offices.png_phys_uniform_grid_1_range_1.graphml -c graphs/offices.png_comm_uniform_grid_1_range_1.graphml -a $i -n 20 -o offices_range1_agents${i}
	# python3 generate_exp.py -d cmarrt-data-wm1 -p cmarrt-data/obstacle_field.png_phys_uniform_grid_1_range_5.graphml -c cmarrt-data/obstacle_field.png_comm_uniform_grid_1_range_5.graphml -a $i -n 20 -o obstacle_field_range5_window2_agents${i} -ws 2 -wm 1
	# Both are window-connected
	# python3 generate_exp.py -d cmarrt-data -p cmarrt-data/pyramid.png_comm_uniform_grid_1_range_5.graphml -c cmarrt-data/pyramid.png_phys_uniform_grid_1_range_5.graphml -a $i -n 10 -o pyramid_range5_window2_agents${i} -w 2
	# python3 generate_exp.py -d cmarrt-data -p cmarrt-data/obstacle_field.png_phys_uniform_grid_1_range_5.graphml -c cmarrt-data/obstacle_field.png_comm_uniform_grid_1_range_5.graphml -a $i -n 10 -o obstacle_field_range5_window2_agents${i} -w 2
	#python3 generate_exp.py -d cmarrt-data -p graphs/obstacle_field.png_phys_uniform_grid_1_range_1.graphml -c graphs/obstacle_field.png_comm_uniform_grid_1_range_1.graphml -a $i -n 20 -o obstacle_field_range1_agents${i} -wm 0
	
	
done

# parallel 2D
# offices 2D
#
# pyramid 3D
# obstacle_field 3D