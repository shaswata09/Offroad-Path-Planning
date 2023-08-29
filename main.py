#!/usr/bin/env python
# coding: utf-8

# In[1]:


import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import shlex
import json
import cProfile, pstats, sys
import cv2
import shutil
import sys
import ast
import csv
import pickle
import re
from path_planning.astar import AStar
from path_planning.BFS import BFS
from path_planning.bidirectDijkstras import BiDirectionalDijkstra
from path_planning.RRT_star import RRT_Star
from path_planning.INFORMED_RRT import Informed_RRT_star
from path_planning.RRT import RRTtree
from path_planning.RRA import RRA
from path_planning.BIT import BIT, ELLIPSE
from path_planning.Theta import Theta
from path_planning.URA import URA
from path_planning.DLITE import DLITESEARCH
from path_planning.URD import URD


import time
import random
import math
import pandas as pd
import os
import copy
from os.path import isfile,join
import argparse


startX, startY, endX, endY = 0,0,0,0
selectStart = True
selectEnd = True
img1 = None
img = None
prediction_matrix = None
im_Size = 0
sat_image = None
filename = None



def runPathPlanning(static_or_dynamic=None, path_planner_names = ['Theta']):
    start = time.time()
    global startX, startY, endX, endY

    startPos = (startX,startY)
    endPos = (endX,endY)
    global path
    global path_runner, filename, temp_file_name
    global img, img1, sat_image
    global prediction_matrix
    global args
    algoPlanner_information = []
    
    for algoName in args.path_planners:
        prediction_matrix_copy = copy.deepcopy(prediction_matrix)
        img_copy = copy.deepcopy(img)
        ground_truth_im_copy = copy.deepcopy(img1)
        sat_copy = copy.deepcopy(sat_image)
        if algoName == "A":

            #Run the pathfinding algorithm
            path_runner = AStar(startPos,endPos,copy.deepcopy(img),"image", prediction_matrix)
            path = path_runner.run()
        elif algoName == "ASTARTHRESHOLD":
            im_c = copy.deepcopy(img)
            for p in range(len(prediction_matrix)):
                for z in range(len(prediction_matrix)):
                    if prediction_matrix[p][z][1] >= 0.30:
                        im_c[p][z] = [255, 255, 255]
                    else:
                        im_c[p][z] = [0,0,0]
                        
            path_runner = AStar(startPos,endPos,im_c,"image", prediction_matrix)
            path = path_runner.run()


        elif algoName == 'Theta':
            path_runner = Theta(startPos,endPos, copy.deepcopy(img1), "image")
            path = path_runner.run()
        
        elif algoName == 'URD':
        	path_runner = URD(startPos, endPos, copy.deepcopy(img1), pred_matrix=copy.deepcopy(prediction_matrix))
        	path = path_runner.URD()
	
	
        elif algoName == "BIT":
            path_runner = BIT(startPos, endPos, 5000, copy.deepcopy(img1))
            path = path_runner.run()
        elif algoName == "BFS":
            path_runner = BFS(startPos,endPos,copy.deepcopy(img),"image")
            path = path_runner.run()

        elif algoName == "URA":
            path_runner = ARA(startPos, endPos, img, prediction_matrix, 0.75, img1)
            path = path_runner.static_path_main()

        elif algoName == "GreedyBestFirst":
            path_runner = greedyBestFirst(startPos,endPos,copy.deepcopy(img),"image")
            path = path_runner.run()
        elif algoName == "BidirectDijkstra":
            path_runner = BiDirectionalDijkstra(startPos,endPos,copy.deepcopy(img),"image")
            path = path_runner.run()
        elif algoName == "RRT":
            path_runner = RRTtree(startPos, endPos, 4, 300, copy.deepcopy(img), "image")
            path = path_runner.run()

        elif algoName == "RRTSTAR":
            path_runner = RRT_Star(startPos, endPos, 5, 10000, 50, copy.deepcopy(img))
            path = path_runner.run()


        elif algoName == "InformedRRT":
            path_runner = Informed_RRT_star(startPos, endPos, 8000, 30, 50, 150, copy.deepcopy(img1), prediction_matrix, copy.deepcopy(img1))
            path = path_runner.run()

        elif algoName == "RRA":
            path_runner = RRA(startPos, endPos, img_copy, img1)
            path = path_runner.ComputePath()

        elif algoName == "DLITE":
            path_runner = DLITESEARCH(startPos, endPos, img_copy, img1)
            path = path_runner.DLITERUN()


        else:
            print("No valid algorithm found for the algorithm name given...exiting")
            exit()


        end = time.time()
        duration = end - start
        similarity_Score = GTsimilarity(path, img1)
        goal_found = False
        
        solution_quality = 0.0
        map_path = []


        if path is not None and endPos in path:    
            first_point = path[0]
            second_point = None
            for pos in path:
                map_path.append(pos)
                if pos != first_point:
                    second_point = pos
                    cv2.line(sat_image, first_point, second_point, color=(0, 255, 0), thickness=7)
                    cv2.line(ground_truth_im_copy, first_point, second_point, color=(0, 255, 0), thickness=7)
                    cv2.line(img_copy, first_point, second_point, color=(0, 255, 0), thickness=7)


                    first_point = pos
                if pos != path[0]:
                    solution_quality += math.hypot((map_path[-1][0] - map_path[-2][0]),(map_path[-1][1] - map_path[-2][1]))
            goal_found = True
        else:
            if path is None or (endX, endY) not in path:
                solution_quality = np.inf
            else:
                map_path = []
                second_point = None
                first_point = path[0]
                for pos in path:
                    map_path.append(pos)
                    if pos != path[0]:
                        solution_quality += math.hypot((map_path[-1][0] - map_path[-2][0]),(map_path[-1][1] - map_path[-2][1]))
                    

        if static_or_dynamic is not None:
            csvCellholder = None
            if static_or_dynamic.upper() == "STATIC":
                csvCellHolder = [duration, solution_quality / np.linalg.norm(np.array([startX, startY])-np.array([endX, endY])), similarity_Score, goal_found, path_runner.nodes_expanded]
            else:
                csvCellHolder = [duration, solution_quality / np.linalg.norm(np.array([startX, startY])-np.array([endX, endY])), path_runner.replans]
            sat_image = cv2.circle(sat_image, (startX, startY), radius=5, color=(255, 0, 0), thickness=-1)
            sat_image = cv2.circle(sat_image, (endX, endY), radius=5, color=(0, 0, 255), thickness=-1)
            ground_truth_im_copy = cv2.circle(ground_truth_im_copy, (startX, startY), radius=5, color=(255, 0, 0), thickness=-1)
            ground_truth_im_copy = cv2.circle(ground_truth_im_copy, (endX, endY), radius=5, color=(0, 0, 255), thickness=-1) 
            l_points_file = copy.deepcopy(img1)
            l_points_file = cv2.circle(l_points_file, (startX, startY), radius=5, color=(255, 0, 0), thickness=-1)
            l_points_file = cv2.circle(l_points_file, (endX, endY), radius=5, color=(0, 0, 255), thickness=-1)
            img_copy = cv2.circle(img_copy, (startX, startY), radius=5, color=(255, 0, 0), thickness=-1)
            img_copy = cv2.circle(img_copy, (endX, endY), radius=5, color=(0, 0, 255), thickness=-1)
            
    
    
            cv2.imwrite(os.path.join(args.output_image_path, static_or_dynamic, algoName, 'sat_path', filename), sat_image)
            cv2.imwrite(os.path.join(args.output_image_path, static_or_dynamic, algoName, 'ground_path', "imcopy"+filename), ground_truth_im_copy)
            cv2.imwrite(os.path.join(args.output_image_path, static_or_dynamic, 'logged_points', filename), l_points_file)
            cv2.imwrite(os.path.join(args.output_image_path, static_or_dynamic, algoName, 'seg_path', filename), img_copy)
            
            
            algoPlanner_information.append(csvCellHolder)
            sat_image = copy.deepcopy(sat_copy)
            img_copy = copy.deepcopy(img)
            prediction_matrix = copy.deepcopy(prediction_matrix_copy)
        else:
            print("Time took: ", duration)
            print('Quality/Cost of solution: ', solution_quality)
    return algoPlanner_information



def GTsimilarity(path, gt_img):
    if path is None:
        path = []
        return 0.0
    ratio = 0
    for a in path:
        if np.array_equal(gt_img[a[1]][a[0]], [255, 255, 255]):
            ratio += 1

    return (ratio/len(path)) * 100



def select_point(event,x,y,flags,param):
    """
    Grabs points selected from the cv2 image shown on screen
    """
    global startX,startY,endX,endY,img,selectStart, img1, selectEnd, prediction_matrix, sat_image, args

        

    if event == cv2.EVENT_LBUTTONDBLCLK: # captures left button double-click
        ix,iy = x,y

        #Check to see if the user selected a point inside a road
        if not np.array_equal(img1[iy][ix],[255,255,255]):
            print("Please select a valid point inside the road!")
            return
            
        #Set the start coordinate if selectStart is still true
        if selectStart:
            img1 = cv2.circle(img1,(ix,iy),radius=5,color=(255,0,0),thickness=-1)
            img = cv2.circle(img,(ix,iy),radius=5,color=(255,0,0),thickness=-1)
            sat_image = cv2.circle(sat_image,(ix,iy),radius=5,color=(255,0,0),thickness=-1)


            selectStart = False
            startX,startY = ix,iy
            cv2.imshow('dimage',img1)
            cv2.waitKey(1)
        #Set the endPoint and begin path planning computation
        elif selectEnd:
            selectEnd = False
            img1 = cv2.circle(img1,(ix,iy),radius=5,color=(0,0,255),thickness=-1)
            img = cv2.circle(img,(ix,iy),radius=5,color=(255,0,0),thickness=-1)
            sat_image = cv2.circle(sat_image,(ix,iy),radius=5,color=(255,0,0),thickness=-1)


            endX,endY = ix,iy
            cv2.imshow('image',sat_image)
            cv2.imshow('dimage', img1)
            cv2.waitKey(1)

            runPathPlanning()
            cv2.imshow('image',sat_image)
            cv2.imshow('dimage', img1)




def getMaxValInList(given_list):
    t = 0.0
    counter = 0
    for a in range(len(given_list)):
        counter += 1
        print(counter)
        print(given_list[a])
        if len(given_list[a]) == 1:
            continue
        if float(given_list[a][1]) > t and given_list[a][1] != 'inf':
            t = float(given_list[a][1])
            
    return t





parser = argparse.ArgumentParser()

def main():
    global img # prediction
    global img1 # gt
    global prediction_matrix
    global startX, startY, endX, endY
    global im_Size
    global sat_image
    global filename
    global temp_file_name
    global parser 
    global args
    
    parser.add_argument('--im_folder', help="Output from neural network", type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\sample_predictions_ensemble_cavs_v2\predictions")
    parser.add_argument('--gt_folder', help="Ground truth image", type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\sample_predictions_ensemble_cavs_v2\ground_truth")
    parser.add_argument('--pred_matrix_folder', help="Prediction matrix from neural network", type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\sample_predictions_ensemble_cavs_v2\prediction_matrix\\")
    parser.add_argument('--sat_folder', help="Satellite image", type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\sample_predictions_ensemble_cavs_v2\original_image")
    parser.add_argument('--number_of_images', help="Number of images that you want to run. Set to arbitrarily high amount that is greater than the number of images in your folder to run entire folder.", type=int, default=2)
    parser.add_argument('--output_image_path', help="Output dump from runs. Use name of dataset for clarity.", type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\CAVSrun")
    parser.add_argument('--image_size', help="Image size for runs", type=int, default=600)
    parser.add_argument('--csv_file_name', help="Output from runs that are dumped into this csv file.", type=str, default='cav_runs.csv')
    parser.add_argument('--logged_points', help="User defined points for runs.", type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\cavs_points.csv")
    parser.add_argument('--static_or_dynamic', help="STATIC for non traversal and DYNAMIC for traversal.", type=str, default="DYNAMIC")
    parser.add_argument('--path_planners', nargs="*", type=str, help='Available path planners: URA, URD, Theta*, ASTARTHRESHOLD, BIT*, RRT STAR, Informed RRT*, RRT, BFS, A*, Greedy Best First, Bidirectional Dijkstra, C Search', default=['URD'])
    
    args = parser.parse_args()
    if len(os.listdir(args.im_folder)) == 0:
        raise Exception("Number of images in image folder is not sufficient.")
        
    elif args.number_of_images > len(os.listdir(args.im_folder)):
        args.number_of_images = len(os.listdir(args.im_folder))
        
    appender = None
    if args.number_of_images > 1:
        calculate_ret_array = []

        appender = []
        with open(args.logged_points, 'r') as file:
            csvreader = csv.reader(file, delimiter=";")
            for row in csvreader:
                    if len(row) == 0:
                        continue
                    appender.append(row)

        random.shuffle(appender)
        appender = appender[:args.number_of_images]
    

        args.static_or_dynamic = args.static_or_dynamic.upper()

        if os.path.exists(os.path.join(args.output_image_path, args.static_or_dynamic)):
            shutil.rmtree(os.path.join(args.output_image_path, args.static_or_dynamic), ignore_errors=True)
                
                
        os.makedirs(os.path.join(args.output_image_path, args.static_or_dynamic, 'logged_points'))            
        for pp_condition in args.path_planners:
                
            pp_condition = (pp_condition.replace(' ', '')).translate({ord('*'): None})

            os.makedirs(os.path.join(args.output_image_path, args.static_or_dynamic, pp_condition))
            os.makedirs(os.path.join(args.output_image_path, args.static_or_dynamic, pp_condition, 'sat_path'))
            os.makedirs(os.path.join(args.output_image_path, args.static_or_dynamic, pp_condition, 'seg_path'))
            os.makedirs(os.path.join(args.output_image_path, args.static_or_dynamic, pp_condition, 'ground_path'))
                
                
        with open(args.csv_file_name, 'w+') as csvfile:
            fieldnames = ['Image Name', 'Image Size', 'Start Position', 'Goal Position', 'Success?']

            fieldnames.extend(args.path_planners)
            thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter = ';')
            thewriter.writeheader()

            for point_arrays in appender:
                    t = shlex.split(point_arrays[0])
                    filename = t[0]
                    with open(args.pred_matrix_folder+filename[0:-3]+'pkl', 'rb') as f:
                        prediction_matrix = pickle.load(f)
                    img = cv2.resize(cv2.imread(join(args.im_folder, filename)), (args.image_size, args.image_size))
                    img1 = cv2.resize(cv2.imread(join(args.gt_folder, filename)), (args.image_size, args.image_size))
                    sat_image = cv2.resize(cv2.imread(join(args.sat_folder, filename)), (args.image_size, args.image_size))
                    (startX, startY) =  eval(t[1])
                    (endX, endY) =  eval(t[2])
                    prediction_matrix = cv2.resize(prediction_matrix, (args.image_size, args.image_size))
                    return_array = runPathPlanning(args.static_or_dynamic, args.path_planners)
                    calculate_ret_array.append(return_array)

                    dict_to_write = {'Image Name': filename, 'Image Size': args.image_size, 'Start Position': (startX, startY), 'Goal Position': (endX, endY)}
                    for p in range(len(args.path_planners)):
                        dict_to_write[args.path_planners[p]] = return_array[p]
                    thewriter.writerow(dict_to_write)


                               
        data = pd.read_csv(args.csv_file_name, delimiter=';')
        head_data = []
        success_rate = [0] * len(args.path_planners)
        avg_path_acc = [0.0] * len(args.path_planners)
        norm_path_length = [0.0] * len(args.path_planners)
        nodes_expanded_rate = [0] * len(args.path_planners)
        constant = -1


        with open(args.csv_file_name) as csv_file:

            csv_reader = csv.reader(csv_file, delimiter = ',')

            len_iter = 0
            for row in csv_reader:
                print(row)
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
                    if arow[a][3] == 'True':
                        success_rate[a] += 1
                    avg_path_acc[a] += float(arow[a][2])
                    if arow[a][1] == 'inf' or (arow[a][1] == '0.0' and arow[a][3] == 'False'):
                        norm_path_length[a] += 2 * tmp_val
                    else:
                        norm_path_length[a] += float(arow[a][1])
                    nodes_expanded_rate[a] += int(arow[a][4])


            for z in range(len(args.path_planners)):
                success_rate[z] /= constant
                norm_path_length[z] /= constant
                avg_path_acc[z] /= constant
                nodes_expanded_rate[z] = int(nodes_expanded_rate[z] / constant)
                print(args.path_planners[z] + ':', [success_rate[z] * 100, norm_path_length[z], avg_path_acc[z], nodes_expanded_rate[z]])



    else:
        cv2.namedWindow('image')
        cv2.namedWindow('dimage')
        imgPath = random.choice(os.listdir(args.im_folder))
        img = cv2.resize(cv2.imread(os.path.join(args.im_folder, imgPath)), (args.image_size, args.image_size))
        img1 = cv2.resize(cv2.imread(os.path.join(args.gt_folder, imgPath)), (args.image_size, args.image_size))
        sat_image = cv2.resize(cv2.imread(os.path.join(args.sat_folder , imgPath)) , (args.image_size, args.image_size), interpolation=cv2.INTER_LINEAR)
        with open(os.path.join(args.pred_matrix_folder,imgPath[0:-3]+'pkl'), 'rb') as f:
            prediction_matrix = pickle.load(f)
        prediction_matrix = cv2.resize(prediction_matrix, (args.image_size, args.image_size))     
        imcopy = copy.deepcopy(img)
        cv2.setMouseCallback('dimage',select_point)
        cv2.imshow('dimage', img1)
        cv2.imshow('image', sat_image)
        k = cv2.waitKey(0) 
        cv2.destroyAllWindows()

        
if __name__ == "__main__":
    main()
        







