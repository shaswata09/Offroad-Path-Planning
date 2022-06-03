from PIL import Image, ImageDraw
from collections import deque
import math
import time
import numpy as np
import heapq
import queue
import sys
import os.path
import pickle
from random import randrange
import cv2


class SearchNode:
    def __init__(self,state,parent):
        self.parent = parent # Pointer to parent node (will be None for root node)
        self.state = state # The state (grid cell) this node points to
        self.cost = 0
        self.heuristic_val = 0
        self.double_parent = None
        self.direction = -1
        #0 is backward 1 is forward       



def euclidean_distance_grid(a,b):
    d = math.hypot(b[0]-a[0],b[1]-a[1])
    return d

class BiDirectionalDijkstra:
    def __init__(self, start, goal, image, imageWindowName):
        self.start = start
        self.goal = goal
        self.img = image
        self.imgWidth = len(image[0])
        self.imgHeight = len(image)
        self.imgWindow = imageWindowName
        self.G = {}
        
    def get_neighbors(self,pos,closed):

        neighbors = []

        #Calculate all the possible adjacent pixels
        adj = [(-1,0),(1,0),(0,-1),(0,1),
                (-1,1),(1,1),(-1,-1),(1,-1)]

        for move in adj:
            #(x,y format)
            nextCell = (pos[0]+move[0],pos[1]+move[1])

            #Make sure the pixel is a valid pixel inside the road
            if nextCell[0] >= self.imgWidth or nextCell[0] < 0 or nextCell[1] >= self.imgHeight or nextCell[1] < 0:
                continue
            if nextCell in closed:
                continue
            if np.array_equal(self.img[nextCell[1]][nextCell[0]],[0,0,0]):
                continue

            neighbors.append(nextCell)

        return neighbors
    
    
    
    def run(self):
        path = []
        forward_visited_nodes = set()
        backward_visited_nodes = set()
        forward_queue = queue.PriorityQueue()
        backward_queue = queue.PriorityQueue()
        found = False
        self.G[self.goal] = SearchNode(self.goal, None)
        self.G[self.start] = SearchNode(self.start, None)
        
        forward_queue.put((1, self.start))
        backward_queue.put((1, self.goal))
        break_statement = False
        final_state_hold = None
        expansions = 0
        while not forward_queue.empty() and not backward_queue.empty() and break_statement == False:
            

            forward_node = forward_queue.get()[1]
            if forward_node in forward_visited_nodes:
                continue
        
            forward_visited_nodes.add(forward_node)
            self.img[forward_node[1]][forward_node[0]] = [0,255,0]
            expansions += 1
            tmp_queue = self.get_neighbors(forward_node, forward_visited_nodes)
            for a in tmp_queue:         
                if a in self.G:
                    if self.G[a].direction == 0:
                        found = True
                        self.G[a].double_parent = forward_node
                        final_state_hold = a
                        break_statement = True
                        current_node = a
                        break
            
                else:
                    self.G[a] = SearchNode(a, forward_node)
                    self.G[a].cost = self.G[forward_node].cost + 1
                    forward_queue.put((self.G[a].cost, a))
                    self.G[a].direction = 1
            
            if break_statement:
                    break
                    
                
            backward_node = backward_queue.get()[1]
            expansions += 1
            if backward_node in backward_visited_nodes:
                continue
        
            backward_visited_nodes.add(backward_node)
            self.img[backward_node[1]][backward_node[0]] = [0,255,0]

            tmp_queue = self.get_neighbors(backward_node, backward_visited_nodes)
        
            for a in tmp_queue:      
                if a in self.G:
                    if self.G[a].direction == 1:
                        found = True
                        self.G[a].double_parent = backward_node
                        final_state_hold = a
                        break_statement = True
                        current_node = a
                        break
                    
                else:
                    self.G[a] = SearchNode(a, backward_node)
                    self.G[a].cost = self.G[backward_node].cost + 1
                    backward_queue.put((self.G[a].cost, a))
                    self.G[a].direction = 0
            if expansions % 100 == 0:
                cv2.imshow(self.imgWindow,self.img)
                cv2.waitKey(1)                
    
     
        forward = []
        backward = []
        middle = []   
        if found:
            print('Found a path!')
            cur_state = final_state_hold

            forward = []
            middle = [cur_state]
            backward = []
            if self.G[cur_state].direction == 1:

                cur_state = self.G[cur_state].parent
                while cur_state is not None:
                        forward.append(cur_state)
                        cur_state = self.G[cur_state].parent

                forward.reverse()


                cur_state = self.G[final_state_hold].double_parent
                while cur_state is not None:
                        backward.append(cur_state)
                        cur_state = self.G[cur_state].parent


            else:

                cur_state = self.G[cur_state].parent
                while cur_state is not None:
                        backward.append(cur_state)
                        cur_state = self.G[cur_state].parent
                
                
                cur_state = self.G[final_state_hold].double_parent

                while cur_state is not None:
                        forward.append(cur_state)
                        cur_state = self.G[cur_state].parent

                forward.reverse()




                
        path = forward + middle + backward
        return path


        