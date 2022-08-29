import cv2
import numpy as np
import os
import heapq
import time


class Node:
    def __init__(self, cost=np.inf, hval=np.inf, predecessor=None):
        self.cost = cost
        self.hval = hval
        self.predecessor = predecessor


class ARA:
    def __init__(self, start, goal, predictedImage, predMatrix, model_probability_accuracy, groundTruthImage=None):
        self.start = start
        self.goal = goal
        self.model_probability_accuracy = model_probability_accuracy
        print("Beginning ARA* search from", self.start, "to", self.goal)
        self.predictedImage = predictedImage
        self.groundTruthImage = groundTruthImage
        self.predictionMatrix = predMatrix
        self.img_width = len(self.predictedImage)
        self.img_height = len(self.predictedImage[0])
        self.closed = set()
        self.Tree = {}
        self.path = None
        self.ep_val = 12.5
        self.open = []
        self.INCONS = set()
        self.Tree[self.goal] = Node()
       # self.Tree[self.goal].hval = 0
        self.Tree[self.start] = Node(cost=0)
        self.Tree[self.start].hval = self.eudis5(self.start, self.goal)
        self.path = []
    def fvalue(self, s):
        return self.Tree[s].cost + self.ep_val * self.Tree[s].hval * (11.0-self.model_probability_accuracy * 10)

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

    def UnpredictableCostGetter(self, state, regular=False):
        if regular: 
            if np.array_equal(self.predictedImage[state[1]][state[0]], [0,0,0]):
                return np.inf
            return 1

        if self.predictionMatrix[state[1]][state[0]][1] >= 0.65:
            return 1.0

        elif self.predictionMatrix[state[1]][state[0]][1] >= 0.50 and self.predictionMatrix[state[1]][state[0]][1] < 0.65:
                return 2.0

        elif self.predictionMatrix[state[1]][state[0]][1] >= 0.4 and self.predictionMatrix[state[1]][state[0]][1] < 0.5:
                return 2.5

        elif self.predictionMatrix[state[1]][state[0]][1] >= 0.3 and self.predictionMatrix[state[1]][state[0]][1] < 0.4:
                return 3.5

        elif self.predictionMatrix[state[1]][state[0]][1] >= 0.2 and self.predictionMatrix[state[1]][state[0]][1] < 0.3:
                return 4.3

        elif self.predictionMatrix[state[1]][state[0]][1] >= 0.1 and self.predictionMatrix[state[1]][state[0]][1] < 0.2:
               return 5.4

        elif self.predictionMatrix[state[1]][state[0]][1] >= 0.04 and self.predictionMatrix[state[1]][state[0]][1] < 0.1:
                return 6.1
        else:
                return 14.5


    def eudis5(self, v1, v2):
        dist = [(a - b)**2 for a, b in zip(v1, v2)]
        dist = np.sqrt(sum(dist))
        return dist

    def buildPath(self):
        cur_Node = self.goal
        tmp_path = []
        while cur_Node != None:
            tmp_path.append(cur_Node)
            if cur_Node == self.start:
                break
            cur_Node = self.Tree[cur_Node].predecessor
        tmp_path.reverse()
        return tmp_path


    

    def ImprovePath(self):
        while self.fvalue(self.goal) > self.open[0][0]:
            top = heapq.heappop(self.open)
            self.closed.add(top[1])
            laugh = self.get_neighbors(top[1])
            for a in laugh: 
                if a == self.Tree[top[1]].predecessor:
                    continue
                if a not in self.Tree:
                    self.Tree[a] = Node(predecessor=top[1])
                    self.Tree[a].hval = self.eudis5(a, self.goal)#+1.0-self.predictionMatrix[a[1]][a[0]][1]
                if self.Tree[a].cost > (tmp_cost:=self.Tree[top[1]].cost+self.UnpredictableCostGetter(a)):
                    self.Tree[a].cost = tmp_cost
                    self.Tree[a].predecessor = top[1]
                    if a not in self.closed:
                        heapq.heappush(self.open, (self.fvalue(a), a))
                    else:
                        self.INCONS.add(a)


    def get_minimum_e_val(self):
       # incons = list(self.INCONS).append(self.open[0][1])
        incons = list(self.INCONS)
        incons.append(self.open[0][1])
        minimum = np.inf
        for a in incons:
            if (tg:=self.Tree[a].cost + self.Tree[a].hval) < minimum:
                minimum = tg

        return minimum

    
    
   # def jumbleProbabilities(self):
        
    
    
    
    def getPathCost(self):
        if len(self.path)==0:
            return np.inf
        solution_quality=0
        first_point=self.path[0]
        map_path = []
        for pos in self.path:
            map_path.append(pos)
            if pos != first_point:
                first_point = pos
            if pos != self.path[0]:
                solution_quality += np.hypot((map_path[-1][0] - map_path[-2][0]),(map_path[-1][1] - map_path[-2][1]))

        return solution_quality    
   

    def path_getter(self):
        heapq.heappush(self.open, (self.fvalue(self.start), self.start))
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
            print(optimal_e)
            self.ImprovePath()
            self.path = self.buildPath()
           # yield self.path
            optimal_e = min(self.ep_val, self.Tree[self.goal].cost / self.get_minimum_e_val())
            print(self.getPathCost())
        return self.buildPath()
         