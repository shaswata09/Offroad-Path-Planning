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
from modules.path_planning.astar import AStar
from modules.path_planning.BFS import BFS
from modules.path_planning.greedyBestFirst import greedyBestFirst
from modules.path_planning.bidirectDijkstras import BiDirectionalDijkstra
from modules.path_planning.RRT_star import RRT_Star
from modules.path_planning.INFORMED_RRT import Informed_RRT_star
from modules.path_planning.RRT import RRTtree
from modules.path_planning.RRA import RRA
from modules.path_planning.BIT import BIT, ELLIPSE
from modules.path_planning.Theta import Theta
from modules.path_planning.FIXED_ARA import ARA
from modules.path_planning.DLITE import DLITESEARCH
from modules.path_planning.over_under_search import over_under_search


# In[2]:


import time
import random
import math
import pandas as pd
import os
import copy
from os.path import isfile,join
import argparse


# In[3]:


startX, startY, endX, endY = 0,0,0,0
selectStart = True
selectEnd = True
img1 = None
img = None
prediction_matrix = None
im_Size = 0
sat_image = None
filename = None


# In[4]:


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
    prediction_matrix_copy = copy.deepcopy(prediction_matrix)
    img_copy = copy.deepcopy(img)
    ground_truth_im_copy = copy.deepcopy(img1)
    sat_copy = copy.deepcopy(sat_image)
    
    for algoName in path_planner_names:
        
        if algoName == "A":

            #Run the pathfinding algorithm
            path_runner = AStar(startPos,endPos,copy.deepcopy(img1),"image", prediction_matrix)
            path = path_runner.run()

        elif algoName == "CSearch":
            path_runner = over_under_search(startPos, endPos, prediction_matrix, img1)
            path = path_runner.static_path_main()
        elif algoName == 'Theta':
            path_runner = Theta(startPos,endPos, copy.deepcopy(img1), "image", prediction_matrix)
            path = path_runner.run()

        elif algoName == "BIT":
            path_runner = BIT(startPos, endPos, 5000, copy.deepcopy(img1))
            path = path_runner.run()
        elif algoName == "BFS":
            path_runner = BFS(startPos,endPos,copy.deepcopy(img),"image")
            path = path_runner.run()

        elif algoName == "URA":
            line_heuristic = [random.uniform(0.1, 2.3) for _ in range(5)]
            param_grid = {'self.first_hyperparameter': [random.uniform(0.1, 30.0) for _ in range(5)], 'self.second_hyperparameter': [random.uniform(0.1, 30.0) for _ in range(5)] , 'self.third_parameter': [random.uniform(0.1, 30.0) for _ in range(5)], 'self.fourth_parameter': [random.uniform(0.1, 30.0) for _ in range(5)]}   
            ap = list(itertools.product(*list(param_grid.values())))
            my_dict = {tup: 0 for tup in ap}
            best_Cost = math.inf
            best_Similarity = -math.inf
            best_Combo = None
            for combination in ap:
                path_runner = ARA(startPos, endPos, img, prediction_matrix, 0.75, img1, combination[0], combination[1], combination[2])
                path = path_runner.static_path_main()
                if (gt_sim:=GTsimilarity(path, img1)) > best_Similarity or best_Similarity == -math.inf:
                        best_Similarity = gt_sim
                        best_Combo = combination
                        best_Cost = path_runner.Tree[endPos].cost
                print(best_Combo, best_Similarity, best_Cost)

            my_dict[best_Combo] += 1
            with open('ronniedrake', 'w+') as f:
                f.write(json.dumps(pet))



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
            path_runner = RRT_Star(startPos, endPos, 5, 10000, 50, copy.deepcopy(img1))
            path = path_runner.run()


        elif algoName == "InformedRRT":
            path_runner = Informed_RRT_star(startPos, endPos, 8000, 30, 50, 150, copy.deepcopy(img1), prediction_matrix, copy.deepcopy(img1))
            path = path_runner.run()

        elif algoName == "RRA":
            path_runner = RRA(startPos, endPos, img, img1)
            path = path_runner.ComputePath()

        elif algoName == "DLITE":
            path_runner = DLITESEARCH(startPos, endPos, img, img1)
            path = path_runner.DLITERUN()


        else:
            print("No valid algorithm found for the algorithm name given...exiting")
            exit()


        #Future path finding algorithms go here..each should return a list of coordinates

        end = time.time()
        duration = end - start
        similarity_Score = GTsimilarity(path, img1)
        goal_found = False
        
        solution_quality = 0.0
        map_path = []
        first_point = path[0]
        second_point = None

        if path is not None and endPos in path:            
            for pos in path:
                map_path.append(pos)
                if pos != first_point:
                    second_point = pos
                    cv2.line(sat_image, first_point, second_point, color=(0, 255, 0), thickness=2)
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
            print(filename)
            directory = os.path.join(args.output_image_path, static_or_dynamic, algoName, 'generated_path_')
            print(directory)
            cv2.imwrite(os.path.join(directory, filename), sat_image)

            algoPlanner_information.append(csvCellHolder)
            sat_image = copy.deepcopy(sat_copy)
            prediction_matrix = copy.deepcopy(prediction_matrix_copy)
        else:
            print("Time took: ", duration)
            print('Quality/Cost of solution: ', solution_quality)
    return algoPlanner_information


# In[5]:


def GTsimilarity(path, gt_img):
    if path is None:
        path = []
        return 0.0
    ratio = 0
    for a in path:
        if np.array_equal(gt_img[a[1]][a[0]], [255, 255, 255]):
            ratio += 1

    return (ratio/len(path)) * 100


# In[6]:


def select_point(event,x,y,flags,param):
    """
    Grabs points selected from the cv2 image shown on screen
    """
    global startX,startY,endX,endY,img,selectStart, img1, selectEnd, prediction_matrix, sat_image

        

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

            #Start the path planning algoirthm


            runPathPlanning()

            #Display the results to the user
            #Resize the image if needed
            #img = cv2.resize(img,(1024,1024),interpolation=cv2.INTER_AREA)
            cv2.imshow('image',sat_image)
            cv2.imshow('dimage', img1)
         #   selectStart = True


# In[7]:


def getMaxValInList(given_list):
    t = 0.0
    for a in range(len(given_list)):
        if float(given_list[a][1]) > t and given_list[a][1] != 'inf':
            t = float(given_list[a][1])
            
    return t


# In[8]:


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
    
    parser.add_argument('--im_folder', type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\Filtered_Ensemble_DeepGlobe\Prediction")
    parser.add_argument('--gt_folder', type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\Filtered_Ensemble_DeepGlobe\Ground_Truth")
    parser.add_argument('--pred_matrix_folder', type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\Filtered_Ensemble_DeepGlobe\Prediction_Matrix\\")
    parser.add_argument('--sat_folder', type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\Filtered_Ensemble_DeepGlobe\Satellite_Image")
    parser.add_argument('--number_of_images', type=int, default=2)
    parser.add_argument('--output_image_path', type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\DEEPGLOBE")
    parser.add_argument('--image_size', type=int, default=600)
    parser.add_argument('--csv_file_name', type=str, default='static_run.csv')
    parser.add_argument('--logged_points', type=str, default=r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\deepglobepoints.csv")
    parser.add_argument('--static_or_dynamic', type=str, default="STATIC")
    parser.add_argument('--path_planners', nargs="*", help='Available path planners: URA, Theta*, BIT*, RRT STAR, Informed RRT*, RRT, BFS, A*, Greedy Best First, Bidirectional Dijkstra, C Search')
    

    args = parser.parse_args(r'--path_planners A* RRT'.split())
    
    print(args)
    #groundTruthFolder = "C:/Users/charles/Downloads/sample_predictions3/ground_trut
    if len(os.listdir(args.im_folder)) == 0:
        raise Exception("Number of images in image folder is not sufficient.")
        
    elif args.number_of_images > len(os.listdir(args.im_folder)):
        raise Exception("Number of images entered are greater than the number of images in the directory.")
        
        
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
    
        path_planner_names = []
        #temp_file_name = input("Input path to output image files: ") or r"C:\Users\charles\Downloads\Path-Planning-On-Aerial-Images-main-20220523T022800Z-001\Path-Planning-On-Aerial-Images-main\DEEPGLOBE"
        if args.static_or_dynamic.lower() == "static":
            args.static_or_dynamic = "STATIC"

            for pp_condition in args.path_planners:
                pp_condition = (pp_condition.replace(' ', '')).translate({ord('*'): None})
                path_planner_names.append(pp_condition)
                directory = os.path.join(args.output_image_path, 'STATIC', pp_condition, 'generated_path_')
                print(directory)
                if not os.path.exists(directory):
                    os.makedirs(directory)
                else:
                    shutil.rmtree(directory, ignore_errors=True)
                    os.makedirs(directory)




        else:    
            for pp_condition in args.path_planners:
                pp_condition = (pp_condition.replace(' ', '')).translate({ord('*'): None})
                path_planner_names.append(pp_condition)
                directory = os.path.join(args.output_image_path, 'DYNAMIC', pp_condition, 'generated_path_')
                if not os.path.exists(directory):
                    os.makedirs(directory)
                else:
                    shutil.rmtree(directory, ignore_errors=True)
                    os.makedirs(directory)


                
        with open(args.csv_file_name, 'w+') as csvfile:
            fieldnames = ['Image Name', 'Image Size', 'Start Position', 'Goal Position', 'Success?']

            fieldnames.extend(path_planner_names)

            thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter = ';')
            thewriter.writeheader()

            for point_arrays in appender:
                    t = shlex.split(point_arrays[0])
                    print(t)
                    filename = t[0]
                    print(filename)
                    with open(args.pred_matrix_folder+filename[0:-3]+'pkl', 'rb') as f:
                        prediction_matrix = pickle.load(f)
                    img = cv2.resize(cv2.imread(join(args.im_folder, filename)), (args.image_size, args.image_size))
                    img1 = cv2.resize(cv2.imread(join(args.gt_folder, filename)), (args.image_size, args.image_size))
                    sat_image = cv2.resize(cv2.imread(join(args.sat_folder, filename)), (args.image_size, args.image_size))
                    (startX, startY) =  eval(t[1])
                    (endX, endY) =  eval(t[2])
                    prediction_matrix = cv2.resize(prediction_matrix, (args.image_size, args.image_size))
                    return_array = runPathPlanning(args.static_or_dynamic, path_planner_names)
                    calculate_ret_array.append(return_array)
                   # thewriter.writerow({'Image Name': filename, 'Image Size': im_Size, 'Start Position': (startX, startY), 'Goal Position': (endX, endY), pathPlannerNames[0]: return_array[0] , pathPlannerNames[1]: return_array[1], pathPlannerNames[2]: return_array[2], pathPlannerNames[3]: return_array[3]})

                    thewriter.writerow({'Image Name': filename, 'Image Size': im_Size, 'Start Position': (startX, startY), 'Goal Position': (endX, endY), path_planner_names[0]: return_array[0] })

                   # thewriter.writerow({'Image Name': filename, 'Image Size': im_Size, 'Start Position': (startX, startY), 'Goal Position': (endX, endY), pathPlannerNames[0]: return_array[0]})


                
                
                
        data = pd.read_csv(args.csv_file_name, delimiter=';')
        head_data = []
        success_rate = [0] * len(path_planner_names)
        avg_path_acc = [0.0] * len(path_planner_names)
        norm_path_length = [0.0] * len(path_planner_names)
        nodes_expanded_rate = [0] * len(path_planner_names)



        print(avg_path_acc)

        constant = -1


        with open(args.csv_file_name) as csv_file:

            # creating an object of csv reader
            # with the delimiter as ,
            csv_reader = csv.reader(csv_file, delimiter = ',')

            # list to store the names of columns

            # loop to iterate through the rows of csv
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

                # adding the first row
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


            for z in range(len(path_planner_names)):
                success_rate[z] /= constant
                norm_path_length[z] /= constant
                avg_path_acc[z] /= constant
                nodes_expanded_rate[z] = int(nodes_expanded_rate[z] / constant)
                print(path_planner_names[z] + ':', [success_rate[z] * 100, norm_path_length[z], avg_path_acc[z], nodes_expanded_rate[z]])



    else:
        cv2.namedWindow('image')
        cv2.namedWindow('dimage')
        imgPath = random.choice(os.listdir(args.im_folder))

        img = cv2.resize(cv2.imread(os.path.join(args.im_folder, imgPath)), (args.image_size, args.image_size))
        img1 = cv2.resize(cv2.imread(os.path.join(args.gt_folder, imgPath)), (args.image_size, args.image_size))
        sat_image = cv2.resize(cv2.imread(os.path.join(args.sat_folder , imgPath)) , (args.image_size, args.image_size), interpolation=cv2.INTER_LINEAR)
        with open(os.path.join(args.pred_matrix_folder,imgPath[0:-3]+'pkl'), 'rb') as f:
            prediction_matrix = pickle.load(f)        
        imcopy = copy.deepcopy(img)
        cv2.setMouseCallback('dimage',select_point)
        cv2.imshow('dimage', img1)
        cv2.imshow('image', sat_image)
        k = cv2.waitKey(0) 
        cv2.destroyAllWindows()

        
if __name__ == "__main__":
    main()
        

