set -e
for i in {10..29}
do
#	python3 generate_exp.py -c data/offices_comm_uniform_grid_11_range_25.graphml -p data/offices_phys_uniform_grid_11_range_25.graphml -a $i -n 30 -o offices11_range25_$i
	python3 generate_exp.py -p data/b-w-open_phys_uniform_grid_13_range_50.graphml -c data/b-w-open_comm_uniform_grid_13_range_50.graphml -a $i -n 10 -o open13_range50_$i
done
 
