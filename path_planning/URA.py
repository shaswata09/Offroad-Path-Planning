import cv2
import numpy as np
import os
import math
import heapq
import random
import threading
from sklearn.model_selection import GridSearchCV

import time
from sklearn.cluster import ward_tree, AgglomerativeClustering, SpectralClustering, KMeans
from sklearn.neighbors import kneighbors_graph


class Node:
    def __init__(self, cost=np.inf, hval=np.inf, predecessor=None):
        self.cost = cost
        self.hval = hval
        self.predecessor = predecessor
        self.TRAVERSED = False
        self.replan_cost = 0
        self.reSearched = True
        self.seen = -1
        self.predecessor_list = set()
        self.successor_list = set()
        self.rhs = np.inf
        self.goal_biaser = 0


class URA:
    def __init__(self, start, goal, predictedImage, predMatrix, model_probability_accuracy, groundTruthImage=None, first_param=0.4, second_param=35, third_param=3.3):
        self.start = start
        self.goal = goal
        self.first_param, self.second_param, self.third_param = first_param, second_param, third_param
        #self.model_probability_accuracy = model_probability_accuracy
        #Future work
        print("Beginning ARA* search from", self.start, "to", self.goal)
        self.predictedImage = predictedImage
        temp_pred_mat = predMatrix[:, :, 0]
        self.groundTruthImage = groundTruthImage
        self.predictionMatrix = predMatrix
        self.alreadyTraversing = False
        self.img_width = len(self.predictedImage)
        self.img_height = len(self.predictedImage[0])
        self.closed = set()
        self.Tree = {}
        self.get_First_Path_metric = True
        self.path = None
        self.current_replan = 0
        self.replan_limit = 50
        self.FORCE_SAMPLE_TRAVERSAL = True
        self.ep_val = 6.2
        self.randarray = np.linspace(0, 2 * np.pi, 300)
        self.sample_heuristic = 0.0
        self.nodes_expanded = 0
        self.open_set_check = set()
        self.sample_point = None
        self.observable_distance = 30
        self.global_queue = set()
        self.interrupt_limit = 5
        self.move_queue = []
        #self.cluster_vals = KMeans(n_clusters=5).fit(temp_pred_mat.reshape(temp_pred_mat.shape[0]*temp_pred_mat.shape[1], 1))
        #self.clust_labels = self.cluster_vals.labels_.reshape((600,600))
        self.open = []
        self.predictionMatrix_heuristic = 35
        self.cost_heuristic = 10
        self.INCONS = set()
        self.current_location = self.start
        self.s, self.g = np.asarray(self.start), np.asarray(self.goal)

    #    param_grid = {'self.first_hyperparameter': [random.uniform(0.1, .0) for _ in range(10)], 'self.second_hyperparameter': [random.uniform(0.1, 30.0) for _ in range(10)] , 'self.third_parameter': [random.uniform(0.1, 30.0) for _ in range(10)], 'self.fourth_parameter': [random.uniform(0.1, 30.0) for _ in range(10)]}
        #np.exp(value)30
        mesh = np.dstack(np.meshgrid(range(self.img_height), range(self.img_width)))
        D = -np.abs((np.cross(self.g - mesh, mesh - self.s) / np.linalg.norm(self.g - self.s)))
        for a in range(self.img_width):
            for b in range(self.img_height):
                #self.Tree[(b, a)] = Node(hval= 1/(D[b,a]) * D[b,a])
                self.Tree[(b, a)] = Node(hval= self.first_param*D[b, a])
                #self.Tree[(b, a)].goal_biaser = 0.05 * self.eudis5((a, b), self.goal)

        self.path = []
     #   for a in range(self.img_width):
     #       for b in range(self.img_height):
     #           self.Tree[(b, a)] = Node(hval=0.3*self.distanceToLine(np.array([b, a])))

        self.Tree[self.start] = Node(cost=0)
        self.Tree[self.start].hval = self.eudis5(self.start, self.goal)
        self.Tree[self.current_location].rhs = 0
        self.Tree[self.goal].hval = 0
        

    def fvalue(self, s):
        return self.Tree[s].cost + (self.eudis5(s, self.goal)  - (self.predictionMatrix[s[1]][s[0]][1] * 1000))
     #   return  self.Tree[s].cost + self.ep_val * self.first_param * (self.Tree[s].hval * (self.predictionMatrix[s[1]][s[0]][1]) * self.second_param)  
       # return self.Tree[s].cost + self.ep_val * (self.Tree[s].hval * (self.UnpredictableCostGetter(s)) * 15)
  #      return self.Tree[s].cost + self.ep_val * (self.Tree[s].hval * (np.exp(self.predictionMatrix[s[1]][s[0]][1]) * 30)) + self.Tree[s].goal_biaser
       # return self.Tree[s].cost + self.ep_val * (self.eudis5(self.s, self.goal))
       #return self.Tree[s].hval * self.ep_val
       #return self.eudis5(s, self.goal) + self.Tree[s].cost
     #  return self.eudis5() + self.ep_val 
      # return self.Tree[s].cost

    def GTsimilarity(self, path, gt_img):
        if path == None:
            return 0.0
        ratio = 0
        for a in path:
            if np.array_equal(gt_img[a[1]][a[0]], [255, 255, 255]):
                ratio += 1

        return (ratio/len(path)) * 100

    def distanceToLine(self, p):
        return -np.abs(np.cross(self.g-self.s,p-self.s)/np.linalg.norm(self.g-self.s))
        #return np.cross(self.g-self.s,p-self.s)/np.linalg.norm(self.g-self.s)

# line cast. 


    def UnpredictableCostGetter(self, state):
        # label -> clust cent
       # return np.exp(self.cluster_vals.cluster_centers_[self.clust_labels[state[1]][state[0]]] * self.third_param)
        return np.exp(self.predictionMatrix[state[1]][state[0]][0] * 2.5) 


    def get_neighbors(self,pos,get_closed=True):
        neighbors = []

        #Calculate all the possible adjacent pixels
        adj = [(-1,0),(1,0),(0,-1),(0,1),
                (-1,1),(1,1),(-1,-1),(1,-1)]

        for move in adj:
            #(x,y format)
            
            nextCell = (pos[0]+move[0],pos[1]+move[1])

            #Make sure the pixel is a valid pixel inside the road
            if nextCell[0] >= self.img_width or nextCell[0] < 0 or nextCell[1] >= self.img_height or nextCell[1] < 0:
                continue
            if nextCell in self.closed and get_closed == False:
                continue


            neighbors.append(nextCell)

        return neighbors


    def eudis5(self, v1, v2):
        dist = [(a - b)**2 for a, b in zip(v1, v2)]
        return np.sqrt(sum(dist))
         
    def buildPath(self):
        cur_Node = self.goal
        tmp_path = []
        while cur_Node != None:
            tmp_path.append(cur_Node)
            if cur_Node == self.current_location:
                break
            cur_Node = self.Tree[cur_Node].predecessor
        tmp_path.reverse()
        return tmp_path

    def ImprovePath(self):
        while self.fvalue(self.goal) > (self.open[0])[0] or self.open[0][1] != self.goal:
            top = heapq.heappop(self.open)
            self.nodes_expanded += 1
            self.closed.add(top[1])
            top_neighbors = self.get_neighbors(top[1])
            for a in top_neighbors: 
                if a == self.Tree[top[1]].predecessor:
                    continue

              #  if self.Tree[a].cost > (tmp_cost:=self.Tree[top[1]].cost+self.UnpredictableCostGetter(a)):
                if self.Tree[a].cost > (tmp_cost:=self.Tree[top[1]].cost+self.predictionMatrix[a[1]][a[0]][0]):
                #if self.Tree[a].cost > (tmp_cost:=self.Tree[top[1]].cost+1):
                    self.Tree[a].cost = tmp_cost
                    self.Tree[a].predecessor = top[1]
                    if a not in self.closed:
                        heapq.heappush(self.open, (self.fvalue(a), a))
                    else:
                        self.INCONS.add(a)



    def get_minimum_e_val(self):
       # incons = list(self.INCONS).append(self.open[0][1])
        incons = list(self.INCONS)
        for open_node in self.open:
            incons.append(open_node[1])
        minimum = np.inf
        for a in incons:
            if (tg:=self.Tree[a].cost + self.Tree[a].hval) < minimum:
                minimum = tg

        return minimum
  


    def static_path_main(self):
        self.Tree[self.goal].cost = np.inf
        self.Tree[self.current_location].cost = 0

        self.open = []
        self.closed = set()
        self.INCONS = set()

        heapq.heappush(self.open, (self.fvalue(self.current_location), self.current_location))


        self.ImprovePath()
        self.path = self.buildPath()
        optimal_e = min(self.ep_val, self.Tree[self.goal].cost / self.get_minimum_e_val())
        while optimal_e > 1.0:
            self.ep_val -= 0.8
            for a in list(self.INCONS): 
                self.INCONS.discard(a)
                self.open.append((np.inf, a))
            for l in range(len(self.open)):
                self.open[l] = (self.fvalue(self.open[l][1]), self.open[l][1])

            heapq.heapify(self.open)
            self.closed = set()
            self.ImprovePath()
            self.path = self.buildPath()
           # yield self.path
            optimal_e = min(self.ep_val, self.Tree[self.goal].cost / self.get_minimum_e_val())

        self.path = self.buildPath()


        if self.get_First_Path_metric:
            ratio = 0
            for a in self.path:
                if np.array_equal(self.groundTruthImage[a[1]][a[0]], [255, 255, 255]):
                    ratio += 1

            self.get_First_Path_metric = False

        self.predictionMatrix_heuristic = 35
        self.cost_heuristic = 10

        return self.path
