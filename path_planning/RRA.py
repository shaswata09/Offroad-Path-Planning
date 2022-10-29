from email.mime import image
from inspect import trace
import math
import queue
import heapq
import time
from PyHeap import Heap
import timeit
from fastdist import fastdist


from tracemalloc import start
import numpy as np
import random
import cv2

class Node:
    def __init__(self):
        self.best_predecessor = None
        self.cost = np.inf
        self.f = np.inf
        self.seen = -1
        

class RRASEARCHTREE:
    def __init__(self, start, goal, segmentImage, GROUNDIMAGE, inclination_angle=None):
        self.start = start
        self.goal = goal
        self.segmentedImage = segmentImage
        self.GROUNDTRUTHIMAGE = GROUNDIMAGE
        self.inc_angle = inclination_angle
        self.img_height = len(segmentImage[0])
        self.initializeComputePath = True
        self.img_width = len(segmentImage)
        self.path = []
        self.closed = set()
        self.SEARCH_TREE = {}
        self.observable_distance = 50
        self.replans = 0
        self.open = []
        self.current_location = self.start
        self.open_Set_check = set()
        self.SEARCH_TREE[self.goal] = Node()
        self.SEARCH_TREE[self.goal].cost = 0
        for a in range(self.img_height):
            for b in range(self.img_width):
                self.SEARCH_TREE[(b, a)] = Node()
        print('Beginning search from:', self.current_location, 'to:', self.goal)

    def eudis5(self, v1, v2):
        dist = [(a - b)**2 for a, b in zip(v1, v2)]
        dist = math.sqrt(sum(dist))
        return dist

    def castRays(self, x,y):
        randarray = np.linspace(0, 2* np.pi, 200)
        for xr in randarray:
            x1 = int(x + self.observable_distance * np.cos(xr))
            y1 = int(y + self.observable_distance * np.sin(xr))
            self.bresenham((x,y), (x1,y1))[1]
            
        return 

    def bresenham(self, start, end):
        (x0,y0) = start
        (x1,y1) = end

        if start[0] < 0 or start[1] < 0 or start[1] >= self.img_height or start[0] >= self.img_width:
            return False, None
        elif np.array_equal(self.GROUNDTRUTHIMAGE[start[1]][start[0]], [0,0,0]):
            self.segmentedImage[start[1]][start[0]] = self.GROUNDTRUTHIMAGE[start[1]][start[0]]
            return False, None
        line = []
        line.append(start)
        xi = yi = D = None
        dX = x1 - x0
        dY = y1 - y0

        if (dX > 0):
            xi = 1
        else:
            xi = -1
        if (dY > 0): 
            yi = 1
        else:
            yi = -1

        dX = abs(dX)
        dY = abs(dY)

        if (dY < dX):

            D  = 2*dY - dX
            while (x0 != x1):

                if (D > 0):
            
                    y0 += yi
                    D -= 2*dX
            
                D += 2*dY
                x0+=xi

                if x0 >= self.img_width or x0 < 0 or y0 >= self.img_height or y0 < 0:
                    return False, line[-1]

                self.segmentedImage[y0][x0] = self.GROUNDTRUTHIMAGE[y0][x0]
                self.SEARCH_TREE[(x0,y0)].seen = 0

                if np.array_equal(self.GROUNDTRUTHIMAGE[y0][x0], [0,0,0]):
                    self.SEARCH_TREE[(x0,y0)].cost = np.inf

                    return (False, (x0,y0))
                
                line.append((x0,y0))

                    

        else:
            D = 2*dX - dY
            while (y0 != y1):

            
                if (D > 0):
            
                    x0 += xi
                    D -= 2*dY
            
                D +=2*dX
                y0+=yi


                if x0 >= self.img_width or x0 < 0 or y0 >= self.img_height or y0 < 0:

                    return False, line[-1]
                self.SEARCH_TREE[(x0,y0)].seen = 0

                self.segmentedImage[y0][x0] = self.GROUNDTRUTHIMAGE[y0][x0]
                if np.array_equal(self.GROUNDTRUTHIMAGE[y0][x0], [0,0,0]):
                    self.SEARCH_TREE[(x0,y0)].cost  = np.inf

                    return (False, (x0, y0))

                line.append((x0,y0))


        return True, end

    def get_neighbors(self,pos,get_closed=False):
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

    def linkCostGrabber(self, state):
        if state not in self.SEARCH_TREE:
            return 1
        elif self.SEARCH_TREE[state].seen != -1 and self.SEARCH_TREE[state].cost == np.inf:
            return np.inf
        else: 
            return 1


    def OpenQueueAndIndexCheck(self, state, cost_val=None):
        if cost_val is None:
            for a in range(len(self.open)):
                if self.open[a][1] == state:
                    return True, a
            print(state, 'not found on open list.')
            return False, None

        for a in range(len(self.open)):
            if self.open[a][1] == state:
                if self.open[a][0] > cost_val:
                    return True, a
                else:
                    return True, None
        return False, None


    def Initialize(self):
        if self.goal not in self.SEARCH_TREE:
            self.SEARCH_TREE[self.goal] = Node()
        self.SEARCH_TREE[self.goal].cost = 0
        self.SEARCH_TREE[self.goal].f = self.eudis5(self.goal, self.current_location)
        heapq.heappush(self.open, (self.SEARCH_TREE[self.goal].f, self.goal))
        self.closed.discard(self.goal)
        self.open_Set_check.add(self.goal)







    def ComputePath(self):
        while True:
            if self.initializeComputePath:
                self.Initialize()

            while self.open:
                head_node = heapq.heappop(self.open)
                self.closed.add(head_node[1])
                self.open_Set_check.discard(head_node[1])
                if head_node[1] == self.current_location: 
                    print(self.SEARCH_TREE[self.current_location].cost)
                    break
            
                if head_node[0] == np.inf:
                    print("No path found.")
                    return None

                neighbors = self.get_neighbors(head_node[1])
                for a in neighbors:
                    if a == self.SEARCH_TREE[head_node[1]].best_predecessor:
                        continue


                    g_cost = self.linkCostGrabber(a)+self.SEARCH_TREE[head_node[1]].cost
                    f_cost = g_cost+self.eudis5(a, self.current_location)
                    if self.SEARCH_TREE[a].best_predecessor == None:
                        self.SEARCH_TREE[a].cost = g_cost
                        self.SEARCH_TREE[a].f = f_cost
                        self.SEARCH_TREE[a].best_predecessor = head_node[1]

                    if f_cost < self.linkCostGrabber(a)+self.SEARCH_TREE[self.SEARCH_TREE[a].best_predecessor].cost:
                        self.SEARCH_TREE[a].f = f_cost
                        self.SEARCH_TREE[a].cost = g_cost
                        self.SEARCH_TREE[a].best_predecessor = head_node[1]

                    if a in self.open_Set_check:
                        if (qg:=self.OpenQueueAndIndexCheck(a,f_cost))[1] != None:
                            self.open[qg[1]] = (f_cost, a)
                            self.SEARCH_TREE[a].f = f_cost
                            heapq.heapify(self.open)
                            self.SEARCH_TREE[a].best_predecessor = head_node[1]
                    else:
                        heapq.heappush(self.open, (f_cost, a))
                        self.open_Set_check.add(a)
                        self.SEARCH_TREE[a].best_predecessor = head_node[1]


            tmp_path = self.buildPath()
            for x in range(len(tmp_path)):
                self.castRays(self.current_location[0], self.current_location[1])
                self.path.append(self.current_location)
                if self.current_location == self.goal:
                    return self.path
                if self.SEARCH_TREE[tmp_path[x+1]].cost == np.inf:
                    break
            
                self.current_location = tmp_path[x+1]

            TEMP_LIST = [self.SEARCH_TREE[self.current_location].best_predecessor]
            self.closed.discard(self.SEARCH_TREE[self.current_location].best_predecessor)


            while TEMP_LIST:
                tt = TEMP_LIST.pop(random.randrange(len(TEMP_LIST)))
                tt_succ = self.get_neighbors(tt, True)
                for t in tt_succ:
                    if t == self.SEARCH_TREE[tt].best_predecessor:
                        continue

                    if self.SEARCH_TREE[t].best_predecessor == tt and t in self.open_Set_check:
                        self.open_Set_check.discard(t)
                        self.open.pop(self.OpenQueueAndIndexCheck(t)[1])
                        TEMP_LIST.append(t)

                    if t in self.closed and self.SEARCH_TREE[t].best_predecessor == tt:
                        self.closed.discard(t)
                        TEMP_LIST.append(t)

                    if self.SEARCH_TREE[t].best_predecessor != tt and t in self.closed:
                        self.closed.discard(t)
                        self.open.append((np.inf, t))

            if len(self.open) > 0:
                self.initializeComputePath = False
                for x in range(len(self.open)):
                    self.open[x] = (self.SEARCH_TREE[self.open[x][1]].cost+self.eudis5(self.open[x][1], self.current_location) , self.open[x][1])

                heapq.heapify(self.open)

            else:
                self.initializeComputePath = True
                self.closed = set()
                self.open_Set_check = set()

          
        print('Goal not found.')
        return []

    def buildPath(self):
        cur_Node = self.current_location
        tmp_path = []
        while cur_Node != None:
            tmp_path.append(cur_Node)
            if cur_Node == self.goal:
                break
            cur_Node = self.SEARCH_TREE[cur_Node].best_predecessor
        return tmp_path



