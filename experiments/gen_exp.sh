set -e
# Number of instances per case
n=10
#for i in {2..50}
for i in {1..1}
do
	python3 generate_exp.py -d cmarrt-data -p cmarrt-data/pyramid.png_comm_uniform_grid_1_range_5.graphml -c cmarrt-data/pyramid.png_phys_uniform_grid_1_range_5.graphml -a $i -n 10 -o pyramid_range5_window2_agents${i} -w 2
	python3 generate_exp.py -d cmarrt-data -p cmarrt-data/obstacle_field.png_phys_uniform_grid_1_range_5.graphml -c cmarrt-data/obstacle_field.png_comm_uniform_grid_1_range_5.graphml -a $i -n 10 -o obstacle_field_range5_window2_agents${i} -w 2
done
