import time
import random
import math
import pandas as pd
import os
import copy
from os.path import isfile,join
import argparse
import re
import csv

parser = argparse.ArgumentParser()

def getMaxValInList(given_list):
    t = 0.0
    counter = 0
    for a in range(len(given_list)):
        counter += 1
        if len(given_list[a]) == 1:
            continue
        if float(given_list[a][1]) > t and given_list[a][1] != 'inf':
            t = float(given_list[a][1])
            
    return t

def main():
    global startX, startY, endX, endY
    global im_Size
    global sat_image
    global parser 
    global args

    parser.add_argument('--csv_file_name', help="Output from runs that are dumped into this csv file.", type=str, default='cav_runs.csv')
    parser.add_argument('--static_or_dynamic', help="STATIC for non traversal and DYNAMIC for traversal.", type=str, default="DYNAMIC")

    args = parser.parse_args()
    data = pd.read_csv(args.csv_file_name, delimiter=';')

    args.path_planners = list(data.columns[5:])
    print('The path planners that you ran are:', *args.path_planners)

    args.norm_path_length = [0.0] * len(args.path_planners)
    if args.static_or_dynamic == "STATIC":
        args.success_rate = [0] * len(args.path_planners)
        args.avg_path_acc = [0.0] * len(args.path_planners)
        args.nodes_expanded_rate = [0] * len(args.path_planners)
    else:
        args.replans = [0] * len(args.path_planners)
	
    constant = -1


    with open(args.csv_file_name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')

        len_iter = 0
        for row in csv_reader:
            arow = re.split(';+', ''.join(row))[4:]
            if len(row) == 0:
                continue
            elif constant == -1:
                constant += 1
                len_iter = len(arow[1:])
                continue
            constant += 1

            arow = [s.replace(' ', ',').strip('][').split(',') for s in arow]
            tmp_val = getMaxValInList(arow)
            for a in range(len_iter):
                if args.static_or_dynamic == "STATIC":
                    if arow[a][3] == 'True':
                        args.success_rate[a] += 1
                    args.avg_path_acc[a] += float(arow[a][2])
                    if arow[a][1] == 'inf' or (arow[a][1] == '0.0' and arow[a][3] == 'False'):
                        args.norm_path_length[a] += 2 * tmp_val
                    else:
                        args.norm_path_length[a] += float(arow[a][1])

                    args.nodes_expanded_rate[a] += int(arow[a][4])
                else:
                    if arow[a][1] == 'inf' or arow[a][1] == 0.0:
                        args.norm_path_length[a] += 2 * tmp_val
                    else:
                        args.norm_path_length[a] += float(arow[a][1])
                    args.replans[a] += int(arow[a][2])

        for z in range(len(args.path_planners)):
            args.norm_path_length[z] /= constant
            if args.static_or_dynamic == "STATIC":
                args.success_rate[z] /= constant
                args.avg_path_acc[z] /= constant
                args.nodes_expanded_rate[z] = int(args.nodes_expanded_rate[z] / constant)
                print(args.path_planners[z] + ':', [args.success_rate[z] * 100, args.norm_path_length[z], args.avg_path_acc[z], args.nodes_expanded_rate[z]])

            else:
                args.replans[z] /= constant
                print(args.path_planners[z] + ':', [args.norm_path_length[z], int(args.replans[z])])
if __name__ == "__main__":
    main()