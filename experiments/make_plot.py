import glob
import re

import os
import argparse


prefix = "obstacle_field_range5_window2_agents"
alg = "CMARRT"
col = "IGNORE_COLLISIONS"
logfolder = "logs/"
plottype = "success"
results={}

# hardcoded value: number of experiments per csv file``
# max_lines = 60

def plot_success():
    pat = re.compile(f"({prefix})([0-9]*)__(.*?)_(.*).csv")
    files = []
    for absfile in glob.glob(logfolder + "/" + prefix + "*.csv"):
        file = os.path.basename(absfile)
        m = pat.match(file)
        assert(m)
        (exp, nb_agents, ealg, ecol) = m.groups()
        if ealg == alg and ecol == col: 
            files.append((int(nb_agents), absfile))
    files.sort()
    if plottype == "success":
        print("Experiment ; nb_agents ; success_rate")
        for absfile in files:
            average_length = 0
            average_time = 0
            # print("Opening ", absfile[1])
            with open(absfile[1], mode='r') as csv_file:
                lines = list(csv_file.readlines())
                nb_lines = len(lines)
                #print("nb_lines: ", nb_lines)
                # success = len(lines)
                success = 0
                for line in lines:
                    parts = line.split(" ; ")
                    if int(parts[2]) == 1:
                        success += 1
                        average_length += int(parts[1])
                        average_time += float(parts[3])
                if success > 0:
                    average_length /= float(success)
                    average_time /= float(success)
            # print(f"{exp} ; {absfile[0]} ; {success / float(max_lines)}; {average_length / int(nb_agents)} ; {average_time}")
            print(f"{exp} ; {absfile[0]} ; {success / float(nb_lines)}")#; {average_length / int(nb_agents)} ; {average_time}")
    elif plottype == "performance":
        print("Experiment ; nb_agents ; exp_no ; avg_cost ; avg_time ")
        # nb_agents x exp_no -> nb of rows
        nbexps = {} 
        # nb_agents x exp_no -> sum of costs
        costs = {}
        # nb_agents x exp_no -> nb of exec times
        times = {}
        #
        key2nb_agents = {}
        #exppat = re.compile(f"({prefix})([0-9]*)__(.*?)_(.*).csv")

        for absfile in files:
            with open(absfile[1], mode='r') as csv_file:
                nb_agents = absfile[0]
                lines = list(csv_file.readlines())
                for line in lines:
                    parts = line.split(" ; ")
                    exp = parts[0]
                    cost = int(parts[1])
                    time = float(parts[3])
                    key = parts[0]
                    key2nb_agents[key] = absfile[0]
                    if key in nbexps:
                        nbexps[key] += 1
                    else:
                        nbexps[key] = 1
                    if key in costs:
                        costs[key] += cost
                    else:
                        costs[key] = cost
                    if key in times:
                        times[key] += time
                    else:
                        times[key] = time
        for key in nbexps.keys():
            print(f"{key} ; {key2nb_agents[key]} ; {costs[key] / nbexps[key]} ; {times[key] / nbexps[key]}")

def main():
    global alg, col, logfolder, prefix, plottype
    parser = argparse.ArgumentParser(description="Given prefix, alg and collision mode, read all csv files in given log directory, produce summary.")
    parser.add_argument("-a", "--algorithm", type=str, dest="alg",
                        help="Algorithm",required=True)
    parser.add_argument("-c", "--collision", type=str, dest="col",
                        help="collision mode",required=True)
    parser.add_argument("-p", "--prefix", type=str, dest="prefix",
                        help="Algorithm",required=True)
    parser.add_argument("-l",dest="logfolder", type=str,
                        help="log folder to read",required=False)
    parser.add_argument("-t",dest="plottype", type=str,
                        help="success or performance",required=True)
    args = parser.parse_args()

    alg = args.alg
    col = args.col
    if args.logfolder:
        logfolder = args.logfolder
    prefix = args.prefix    
    plottype = args.plottype
    plot_success()
main()
