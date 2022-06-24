import sys
import re
def parse(filename):
    with open(filename,'r') as f:
        bench=""
        time=0
        cost=0
        for line in f:
            m = re.search('>>>>> (.*)', line)
            if m:
                #print("Line: " + line)
                #print("\t"+m.group(1))
                bench=m.group(1)
                time=0
                cost=0
            m = re.search('(.*)user', line)
            if m:
                time = float(m.group(1))
                if cost > 0:
                    print(bench,";",cost,"; 1 ;",time)
                else:
                    print(bench,";",cost,"; 0 ;",time)
                    # pass # if printed cost is 0, this means the benchmark failed
            m = re.search('Execution cost:([0-9]+)', line)
            if m:
                cost = int(m.group(1))
            #m = re.search('Execution found', line)
            #if m:
            #    print(bench + ";" + str(length) +  ";1;" + str(time))
            #m = re.search('Execution not found', line)
            #if m:
            #    print(bench + ";" +  str(length) + ";0; " + str(time))
            

if len(sys.argv) < 2:
    raise Exception("Give log file name")
filename=sys.argv[1]
parse(filename)
