import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import shlex
import json
import cProfile, pstats, sys
import cv2
import sys
import ast
import csv
import pickle
import re
from path_planning.astar import AStar
from path_planning.RRA import RRASEARCHTREE
from path_planning.URA import URA
from path_planning.DLITE import DLITESEARCH
from path_planning.RRT_star import RRT_Star
import time
import random
import math
import pandas as pd
import os
import copy
from os.path import isfile,join
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--static_mode', action='store_true', help="Static path planning instead of dynamic path planning")
parser.add_argument("--start_goal", type=str, default="data/massachusetts_start_goal.csv", help="Input CSV file specifying start and goal coordinates")
parser.add_argument("--segmented_dir", type=str, default="data/sample_predictions_ensemble_massachuests/predictions/", help="Input directory of segmented images")
parser.add_argument("--ground_truth_dir", type=str, default="data/sample_predictions_ensemble_massachuests/prediction_matrix/", help="Input directory of ground truth images")
parser.add_argument("--prediction_dir", type=str, default="data/sample_predictions_ensemble_massachuests/ground_truth/", help="Input directory of prediction matrices")
parser.add_argument("--output_dir", type=str, default="output", help="Output directory name")
parser.add_argument("--output_csv", type=str, default="massachusetts.csv", help="Output CSV file")
args = parser.parse_args()

if args.static_mode:
    pathPlannerNames = ['RRTSTAR', 'ASTAR', 'ASTARTHRESHOLD', 'URA']
#    pathPlannerNames = ['ASTAR']
    if not os.path.exists(os.path.join(args.output_dir, "STATIC")):
        for p in pathPlannerNames:
            os.makedirs(os.path.join(args.output_dir, "STATIC", p))
else:
    pathPlannerNames = ['RRA', 'DSTARLITE', 'URA']
    if not os.path.exists(os.path.join(args.output_dir, "DYNAMIC")):
        for p in pathPlannerNames:
            os.makedirs(os.path.join(args.output_dir, "DYNAMIC", p))
    
fieldnames = ['Image Name', 'Image Size', 'Start Position', 'Goal Position', 'Success?']
fieldnames.extend(pathPlannerNames)

pathMetrics = []
outputWriter = None
startX, startY, endX, endY = 0,0,0,0
selectStart = True
selectEnd = True
img = None
im_Size = 600
img1 = None
prediction_matrix = None
path = None
path_runner = None
img_copy = None
ground_truth_im_copy = None
filename = None


def runPathPlanning(static_or_dynamic):
    global startX, startY, endX, endY
    startPos = (startX,startY)
    endPos = (endX,endY)
    global path
    global alreadyGenerated
    global path_runner
    global img, img1
    global prediction_matrix
    global filename
    global pathPlannerNames
    algoPlanner_information = []
    prediction_matrix_copy = copy.deepcopy(prediction_matrix)
    img_copy = copy.deepcopy(img)
    ground_truth_im_copy = copy.deepcopy(img1)
    
    for algoName in pathPlannerNames:
        start = time.time()

        if algoName == "ASTAR":
            path_runner = AStar(startPos,endPos,copy.deepcopy(img),"image")
            path = path_runner.run()
        
        elif algoName == "ASTARTHRESHOLD":
            im_c = copy.deepcopy(img)
            for p in range(len(prediction_matrix)):
                for z in range(len(prediction_matrix)):
                    if prediction_matrix[p][z][1] >= 0.30:
                        im_c[p][z] = [255, 255, 255]
                    else:
                        im_c[p][z] = [0,0,0]
                        
            path_runner = AStar(startPos,endPos,im_c,"image")
            path = path_runner.run()
                    

        elif algoName == "URA":
            path_runner = URA(startPos, endPos, img, prediction_matrix, 0.75, img1)
            path = path_runner.static_path_main()

        elif algoName == "RRTSTAR":
            path_runner = RRT_Star(startPos, endPos, 5, 200, 50, copy.deepcopy(img))
            path = path_runner.run()

        elif algoName == "RRA":
            path_runner = RRASEARCHTREE(startPos, endPos, img, img1)
            path = path_runner.ComputePath()

        elif algoName == "DSTARLITE":
            path_runner = DLITESEARCH(startPos, endPos, img, img1, prediction_matrix)
            path = path_runner.DLITERUN()

        else:
            print("No valid algorithm found for the algorithm name given...exiting")
            exit()

        end = time.time()
        duration = end - start
        solution_quality = 0.0
        goal_found = False

        similarity_Score = GTsimilarity(path, img1)

         
        if path != None and endPos in path:
            map_path = []
            first_point = path[0]
            second_point = None
            for pos in path:
                map_path.append(pos)
                if pos != first_point:
                    second_point = pos
                    cv2.line(img1, first_point, second_point, color=(0, 255, 0), thickness=5)
                    cv2.line(img, first_point, second_point, color=(0, 255, 0), thickness=5)

                    first_point = pos
                if pos != path[0]:
                    solution_quality += math.hypot((map_path[-1][0] - map_path[-2][0]),(map_path[-1][1] - map_path[-2][1]))
            goal_found = True
        else: 
            if path == None or (endX, endY) not in path:
                solution_quality = np.inf
            else:
                map_path = []
                second_point = None
                first_point = path[0]
                for pos in path:
                    map_path.append(pos)
                    if pos != path[0]:
                        solution_quality += math.hypot((map_path[-1][0] - map_path[-2][0]),(map_path[-1][1] - map_path[-2][1]))
                    
                
                    
            
        csvCellHolder = None
        if static_or_dynamic:
            csvCellHolder = [duration, solution_quality / np.linalg.norm(np.array([startX, startY])-np.array([endX, endY])), similarity_Score, goal_found]
        else:
            csvCellHolder = [duration, solution_quality / np.linalg.norm(np.array([startX, startY])-np.array([endX, endY])), path_runner.replans]

        img1 = cv2.circle(img1, (startX, startY), radius=5, color=(255, 0, 0), thickness=-1)
        img1 = cv2.circle(img1, (endX, endY), radius=5, color=(0, 0, 255), thickness=-1)
        img = cv2.circle(img, (startX, startY), radius=5, color=(255, 0, 0), thickness=-1)
        img = cv2.circle(img, (endX, endY), radius=5, color=(0, 0, 255), thickness=-1)
        
        if static_or_dynamic:
            l = os.path.join(args.output_dir, "STATIC", algoName)
        else: 
            l = os.path.join(args.output_dir, "DYNAMIC", algoName)

        cv2.imwrite(os.path.join(l, 'generated_path_' + filename + '.jpg'), img1)            
        cv2.imwrite(os.path.join(l, 'generated_path_' + filename + '_segmentated.jpg'), img)            
        
        algoPlanner_information.append(csvCellHolder)

        img1 = copy.deepcopy(ground_truth_im_copy)
        img = copy.deepcopy(img_copy)
        prediction_matrix = copy.deepcopy(prediction_matrix_copy)
    return algoPlanner_information

def GTsimilarity(path, gt_img):
    if path == None:
        return 0.0
    ratio = 0
    for a in path:
        if np.array_equal(gt_img[a[1]][a[0]], [255, 255, 255]):
            ratio += 1

    return (ratio/(len(path) + 1e-6)) * 100

def main():
    os.environ["CUDA_VISIBLE_DEVICES"] = "1"

    global img 
    global img1
    global prediction_matrix
    global im_Size
    global filename
    global img_copy
    global ground_truth_im_copy
    global outputWriter
    global startX, startY, endX, endY
    global fieldnames
    calculate_ret_array = []

    with open(args.output_csv, 'w+') as csvfile:
        outputWriter = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter = ';')
        outputWriter.writeheader()

        imgFolder = args.segmented_dir
        groundTruthFolder = args.ground_truth_dir
        predictionMatrixFolder = args.prediction_dir
        points_input = args.start_goal
        appender = []
        
        with open(points_input, 'r') as file:
            csvreader = csv.reader(file, delimiter=" ")
            for row in csvreader:
                if len(row) == 0:
                    continue
                appender.append(row)

        for t in appender:
            filename = t[0]
            print('Processing file %s ...' % filename)
            with open(os.path.join(predictionMatrixFolder, filename[0:-3]+'pkl'), 'rb') as f:
                prediction_matrix = pickle.load(f)
            img = cv2.resize(cv2.imread(join(imgFolder, filename)), (im_Size, im_Size))
            img1 = cv2.resize(cv2.imread(join(groundTruthFolder, filename)), (im_Size, im_Size))
            (startX, startY) =  eval(t[1])
            (endX, endY) =  eval(t[2])
            prediction_matrix = cv2.resize(prediction_matrix, (im_Size, im_Size))
            return_array = runPathPlanning(args.static_mode)
            calculate_ret_array.append(return_array)
            current_row = {'Image Name': filename, 'Image Size': im_Size, 'Start Position': (startX, startY), 'Goal Position': (endX, endY)}
            for i in range(len(pathPlannerNames)):
                current_row[pathPlannerNames[i]] = return_array[i]
            outputWriter.writerow(current_row)
    
if __name__ == "__main__":
    main()
