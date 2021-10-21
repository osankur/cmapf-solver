import sys
import igraph
import os
from igraph import *
import argparse
import math
import random

output_folder = "data/"

"""
Generate connected configuration of size nb_agents
Argument from_base is the vertex of the first agent, which will serve as the basis.
If it is None, then we pick it randomly
"""
def generate_connected_configuration(comm, nb_agents, from_base=None):
    n = len(comm.vs)
    if from_base == None:
        prev_node = random.randint(0,n-1)
    else:
        prev_node = from_base
    start = [prev_node]
    cluster = set(map(lambda x: x.index, comm.vs[prev_node].neighbors()))
    for a in range(1,nb_agents):
        # resample if the chosen vertex is already occupied by another agent
        neighbors = list(cluster)
        if len(neighbors) <= a:
            raise Exception("Cannot make connected configuration collision-free")
        while prev_node in start:
            prev_node = neighbors[random.randint(0,len(neighbors)-1)]
        cluster = cluster | set(map(lambda x: x.index, comm.vs[prev_node].neighbors()))
        start.append(prev_node)
    return start



"""
@pre i>0
@pre conf defined for 0...i-1
"""
def generate_sequentially_connected_configuration_aux(comm, conf, support, i):
    if (i == len(conf)):
        return True
    cluster = list(set(map(lambda x: x.index, comm.vs[conf[i-1]].neighbors())) - support)
    random.shuffle(cluster)
    for v in cluster:
        support.add(v)
        conf[i] = v
        if(generate_sequentially_connected_configuration_aux(comm, conf, support, i+1)):
            return True
        support.remove(v)
    return False

"""
Generate a random configuration where each agent i is connected to i-1
"""
def generate_sequentially_connected_configuration(comm, nb_agents, from_base=None):
    n = len(comm.vs)
    if from_base == None:
        node = random.randint(0,n-1)
    else:
        node = from_base
    support = set([node])
    conf = [None] * nb_agents
    conf[0] = node
    if (generate_sequentially_connected_configuration_aux(comm, conf, support, 1)):
        return conf
    else:
        return None


def generate_connected_successor_rec(phy, comm, conf, partial_succ, support, i):
    if i == len(conf):
        return True
    move_neighbors = list(map(lambda x : x.index, phy.vs[conf[i]].neighbors()))
    move_neighbors.append(conf[i])
    conn_neighbors = set([])
    if i == 0:
        conn_neighbors =  set(move_neighbors)
    else:
        for j in range(0,i):
            conn_neighbors = conn_neighbors | set(map(lambda x: x.index, comm.vs[partial_succ[j]].neighbors()))
    # list of successors of conf[i] which are neighbors of partial_succ[0...i-1]
    # print("Move neighbors of node ", i, ": ", move_neighbors)
    # print("Comm neighbors of node ", i, ": ", conn_neighbors)
    candidate_neighbors = list((set(move_neighbors) & conn_neighbors) - support)
    # print("Viable neighbors of node ", i, ": ", len(candidate_neighbors))
    random.shuffle(candidate_neighbors)
    for v in candidate_neighbors:
        partial_succ[i] = v
        support.add(v)
        if (generate_connected_successor_rec(phy,comm,conf,partial_succ,support,i+1)):
            return True
        support.remove(v)
    return False
"""
Generate a random connected successor of conf where agent i is connected to {0,1,...,i-1}
"""
def generate_connected_successor(phy, comm, conf):
    partial_succ = [None] * len(conf)
    if (generate_connected_successor_rec(phy, comm, conf, partial_succ, set([]), 0)):
        return partial_succ
    else:
        return None

def check_connected(comm, conf):
    u = conf[0]
    cluster = set(map(lambda x: x.index, comm.vs[u].neighbors()))
    for v in conf[1:]:        
        #print("Neighborshood of u=" + str(u) + ": ", neighbors)
        #print("Is v=", v, " in?")
        if not(v in cluster):
            return False
        u = v
        cluster = cluster | set(map(lambda x: x.index, comm.vs[u].neighbors()))
    return True

def generate(phys_filename, comm_filename, nb_agents, filename):
    phy = Graph.Read_GraphML(phys_filename)
    comm = Graph.Read_GraphML(comm_filename)
    start = generate_sequentially_connected_configuration(comm, nb_agents, None)
    goal = generate_sequentially_connected_configuration(comm, nb_agents, start[0])
    assert(check_connected(comm,start))
    assert(check_connected(comm,goal))
    # We have start[0] == goal[0]. This is the base agent's position
    with open(output_folder + filename,"w") as f:
        print("Writing " + output_folder + filename)
        print("phys_graph " + os.path.basename(phys_filename), file=f)
        print("comm_graph " + os.path.basename(comm_filename), file=f)
        print("start ",end='',file=f)
        for v in start:
            print(str(v),"",end='',file=f)
        print("",file=f)
        print("goal ",end='',file=f)
        for v in goal:
            print(str(v),"",end='',file=f)
        print("",file=f)


def main():
    parser = argparse.ArgumentParser(description="Exp generator. Given physical and communication graphs this script randomly generates two connected components and writes this in an .exp file. Agent 0 has the same start and goal vertex since it will serve as a fixed base.")
    parser.add_argument("-p", "--physical", type=str, dest="phys",
                        help="Physical graph",required=True)
    parser.add_argument("-c", "--communication", type=str, dest="comm",
                        help="Communication graph",required=True)
    parser.add_argument("-a", "--agents", dest="nb_agents", type=int,                        
                        help="Number of agents",required=True)
    parser.add_argument("-n", "--number_of_exps", dest="nb_exps", type=int,
                        help="Number of agents",required=True)
    parser.add_argument("-o",dest="out", type=str,
                        help="Output file name base",required=True)
    args = parser.parse_args()

    nb_agents = args.nb_agents
    nb_exps = args.nb_exps
    
    random.seed()
    for i in range(nb_exps):
        filename = args.out + "__" + str(i) + ".exp"
        generate(args.phys, args.comm, nb_agents, filename)

main()
