# cmapf-solver
This is a solver for the connected multi-agent path finding problem. The program has several solvers, including the one published in [the following paper](https://hal.science/hal-04075393).

    Improved Complexity Results and an Efficient Solution for Connected Multi-Agent Path Finding
    Isseïnie Calviac and Ocan Sankur and François Schwarzentruber
    22nd International Conference on Autonomous Agents and Multiagent Systems (AAMAS'23) 2023

## Usage
The following algorithms are available:
- CASTAR (with randomized conflict resolution as described in the paper)
- DFS (the greedy algorithm of Tateo et al. AAAI 2018)
- CASTARDFS: the same CASTAR which falls back to DFS when stuck
- CMARRT, CMARRTSTAR: an algorithm inspired by RRT (unpublished)
- COORD: greedy algorithm similar to DFS in which one uses coordinated action selection to pick successors. Assumes a constant connectivity topology.

Benchmarks can be found in `experiments/data/` which contain .exp files that specify the movement graph, the communication graph, and the start and target configurations.
The graphs referred to by these experiments are .graphml files whose directory is to be specified with the -G option. 

For instance, the following commands run three algorithms on a given experiment.

    cmapf-solver -a CASTAR -e experiments/data/open13_range50_10__3.exp -G experiments/data/
    cmapf-solver -a DFS -e experiments/data/open13_range50_10__3.exp -G experiments/data/
    cmapf-solver -a COORD -e experiments/data/open13_range50_10__3.exp -G experiments/data/

Add the `--exec 1` option to display the computed execution.

Benchmarks that are presented in the AAMAS paper can be reproduced by `experiments/run_all.sh`. The experiment files are in the `experiments/cmarrt-data` directory.
All logs that were used to produce graphs are in `experiments/aamas`.

## GUI and Producing Benchmarks
Francois Schwarzentruber's web-based [CMAPF GUI](https://github.com/francoisschwarzentruber/cmapf-gui) tool can be used to generate experiment files, and to visualize computed executions.

## Contributors
- Arthur Queffelec
- Ocan Sankur
- Isseïnie Calviac