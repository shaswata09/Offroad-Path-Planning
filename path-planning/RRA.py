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
    def __init__(self, predecessor=None):
        self.predecessor = predecessor
        self.cost = math.inf
        self.heuristic_queue_val = math.inf


class RRASEARCHTREE:
    def __init__(self, start_config, goal_config, seg_image, true_image, visible_cells=None, non_visible_cells=None, imageWindowName=None, inclination_angle_in_degrees = None):
        assert(len(seg_image)==len(true_image) and len(seg_image[0])==len(true_image[0]))
        self.start = start_config
        self.current_location = self.start
        self.goal = goal_config
        self.initial_image = seg_image
        self.randarray = np.linspace(0, 2*np.pi, 100)
        self.pps = np.average(self.randarray)-self.randarray[0]
        self.visible_cells = visible_cells
        self.non_visible_cells = non_visible_cells
        self.img_width = len(seg_image)
        self.img_height = len(seg_image[0])
        self.true_image = true_image
        self.imageWindowName = imageWindowName
        self.D3_inc_angle = inclination_angle_in_degrees
        self.first_run = True
        self.path = []
        self.closed = set()
        self.open_Set_Check = set()
        self.open = []
        self.agent_tree = {}
        self.agent_tree[self.goal] = Node()
        self.agent_tree[self.goal].cost = 0
     #   self.agent_tree[self.goal].heuristic_queue_val = self.eudis5(self.goal, self.start)


        print('Beginning search from:', self.current_location, 'to:', self.goal)

    def eudis5(self, v1, v2):
        dist = [(a - b)**2 for a, b in zip(v1, v2)]
        dist = math.sqrt(sum(dist))
        return dist



    def changeCurrentLocation(self, new_location):
        self.current_location = new_location

    def castRays(self, x,y):
        final_dist = 100
        angle_gen=0
        #angle_gen = math.atan2(y-self.current_location[1],x-self.current_location[0])
        for xr in self.randarray:
            #x1 = int(x + final_dist * np.cos(xr+angle_gen-self.pps))
           # y1 = int(y + final_dist * np.sin(xr+angle_gen-self.pps))
            x1 = int(x+final_dist*np.cos(xr))
            y1 = int(y+final_dist*np.sin(xr))
            self.bresenham((x,y), (x1,y1)) 
             #   cv2.line(img_Copy,(x,y),(x1,y1),(0,255,0),1)
            #else:
             #   cv2.line(img_Copy, (x,y), lop[1], (0,0,255),1)

            
        
        return 
    def bresenham(self, start, end):
        if start[0] < 0 or start[1] < 0 or start[1] >= len(self.initial_image) or start[0] >= len(self.initial_image) or np.array_equal(self.true_image[start[1]][start[0]], [0,0,0]):
            self.agent_tree[start].cost = math.inf
            self.agent_tree[start].f_val = math.inf
            self.initial_image[start[1]][start[0]] = self.true_image[start[1]][start[0]]
            return False
        #, (start[0],start[1])
        
        (x0, y0) = start
        (x1, y1) = end
    
    
        m = None

    
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        if dx == 0:
            m = 1.0
        else:
            m = dy/dx
        
        mm = False
        # step 3 perform test to check if pk < 0
        flag = True
    
        line_pixel = []

      #  line_pixel.append((x0,y0))
    
        step = 1
        if x0>x1 or y0>y1:
            step = -1
        
        if m < 1:
            x0, x1 ,y0 ,y1 = y0, y1, x0, x1
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            mm = True
        
        p0 = 2*dx - dy
        x = x0
        y = y0
        term = None
        for i in range(abs(y1-y0)):
            if flag:
                x_previous = x0
                p_previous = p0
                p = p0
                flag = False
            else:
                x_previous = x
                p_previous = p
        
    
            if p >= 0:
                x = x + step

            p = p_previous + 2*dx -2*dy*(abs(x-x_previous))
            y = y + step
        
            if mm:
                self.initial_image[x][y] = self.true_image[x][y]

                if x < 0 or y < 0 or y >= len(self.initial_image) or x >= len(self.initial_image) or  np.array_equal(self.true_image[x][y],[0,0,0]):
                    if (y,x) in self.agent_tree:
                        self.agent_tree[(y,x)].cost = math.inf
                        neighbors = self.get_neighbors((y,x),True)
                        for p in neighbors:
                            if p in self.agent_tree and self.agent_tree[p].predecessor == (y,x):
                                self.agent_tree[p].cost = math.inf
                                self.agent_tree[p].heuristic_queue_val = math.inf
                            #    self.agent_tree[p].predecessor = None

                    return False

                #, line_pixel[-1]
         #       line_pixel.append((y,x))
            else:
                self.initial_image[y][x] = self.true_image[y][x]

                if x < 0 or x >= len(self.initial_image) or y < 0 or y >= len(self.initial_image) or np.array_equal(self.true_image[y][x],[0,0,0]):                  
                    if (x,y) in self.agent_tree:
                        self.agent_tree[(x,y)].cost = math.inf
                        neighbors = self.get_neighbors((x,y),True)
                        for p in neighbors:
                            if p in self.agent_tree and self.agent_tree[p].predecessor == (x,y):
                                self.agent_tree[p].cost = math.inf
                                self.agent_tree[p].heuristic_queue_val = math.inf
                               # self.agent_tree[p].predecessor = None
                    
                    
                    return False
                #, line_pixel[-1]
        #        line_pixel.append((x,y))
            
        return True
    def get_neighbors(self,pos,get_closed=False):
        neighbors = set()

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
           # if np.array_equal(self.img[nextCell[1]][nextCell[0]],[0,0,0]):
           # if self.inclination_angle is not None and self.inclination_angle < self.angleOfInclination(pos, nextCell):
           #     continue

            neighbors.add(nextCell)

        return neighbors

# self.euclidean_distance_grid(g,self.goal) + (2 * max(abs(g[0]-self.goal[0]) , abs(g[1]-self.goal[1])))

# use cost val to compare. 
    def isOnOpenQueueAndCostCheck(self, cost_val, state):
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
# if in open queue, check if dictionary cost is less than computed cost. If dictionary cost is less than, keep the dictionary cost. Otherwise, return index.
    def Initialize(self):
        if self.first_run:
            self.first_run = False
            self.agent_tree[self.goal].cost = 0
            self.agent_tree[self.goal].predecessor = None
            self.agent_tree[self.goal].heuristic_queue_val = self.eudis5(self.goal, self.current_location)
        #self.open.insert((self.agent_tree[self.goal].heuristic_queue_val, self.goal))
        else:
            self.closed.discard(self.goal)
            self.agent_tree[self.goal].cost, self.agent_tree[self.goal].predecessor = 0,None
        heapq.heappush(self.open, (0.0, self.goal))
        self.open_Set_Check.add(self.goal)


#problem is in the search method.
    def CalculatePath(self):
        q_check = False
        self.Initialize()
        g_cost = f_cost = None
        while self.open:
            front = heapq.heappop(self.open)
            if front[0] == math.inf:
                print('no path')
              #  return self.Navigate()
                pred_temp = self.current_location
                print(pred_temp)
                pt = []
                while pred_temp != None:
                    pt.append(pred_temp)

                    pred_temp = self.agent_tree[pred_temp].predecessor
                return pt
            #make update visibility to check for changes.

            self.open_Set_Check.discard(front[1])
            self.closed.add(front[1])
            front_neighbros = self.get_neighbors(front[1])
            # g_cost would be different in PRM. IN THIS CASE IT WOULD JUST BE 1.
            for a in front_neighbros:
                if np.array_equal(self.initial_image[a[1]][a[0]], [0,0,0]) == False:
                    g_cost = 1 + self.agent_tree[front[1]].cost
                    f_cost = self.eudis5(a,self.start) + (2 * max(abs(a[0]-self.start[0]) , abs(a[1]-self.start[1])))

                else:
                    g_cost = f_cost = math.inf



                if a == self.current_location:
                    self.closed.add(self.current_location)
                    if a not in self.agent_tree:
                        self.agent_tree[a] = Node()
                        self.agent_tree[a].cost = g_cost
                        self.agent_tree[a].heuristic_queue_val = f_cost
                        self.agent_tree[a].predecessor = front[1]
                    else:
                        self.agent_tree[a].cost = g_cost
                        self.agent_tree[a].predecessor = front[1]
                        self.agent_tree[a].heuristic_queue_val = f_cost

                    q_check = True
                    #return self.Navigate()



                if a not in self.agent_tree:
                    self.agent_tree[a] = Node()
                    self.agent_tree[a].cost = g_cost
                    self.agent_tree[a].heuristic_queue_val = f_cost
                    self.agent_tree[a].predecessor = front[1]
                    heapq.heappush(self.open, (f_cost, a))
                    self.open_Set_Check.add(a)
                    


                elif a in self.open_Set_Check:
                    lop = self.isOnOpenQueueAndCostCheck(f_cost, a)
                    if lop[0] == True and lop[1] is not None:
                        self.agent_tree[a].predecessor = front[1]
                        self.agent_tree[a].cost = g_cost
                        self.agent_tree[a].heuristic_queue_val = f_cost
                        self.open.pop(lop[1])
                        self.open.append((f_cost, a))
                        heapq.heapify(self.open)

            if q_check:
                return self.Navigate()
        print('initial path not found.')
        return self.Navigate()
       



    def Navigate(self):
        print('Navigation stage')
        self.castRays(self.current_location[0],self.current_location[1])
        if self.agent_tree[self.current_location].cost != math.inf and self.current_location not in self.path:
            self.path.append(self.current_location)
        while self.agent_tree[self.current_location].predecessor != None and np.array_equal(self.initial_image[self.agent_tree[self.current_location].predecessor[1]][self.agent_tree[self.current_location].predecessor[0]], [0,0,0])==False:
            if self.agent_tree[self.current_location].predecessor == None:
                print('Original path invalidated.')
                break
            self.current_location = self.agent_tree[self.current_location].predecessor
            self.path.append(self.current_location)
            self.castRays(self.current_location[0],self.current_location[1])

            if self.current_location == self.goal:
                print('Path found')
                return self.path
          #  self.castRays(self.agent_tree[self.current_location].predecessor[0],self.agent_tree[self.current_location].predecessor[1])

#        if np.array_equal(self.initial_image[self.current_location[1]][self.current_location[0]], [0,0,0]) == True:
 #           print('current location is blocked.')
 #       elif np.array_equal(self.initial_image[self.agent_tree[self.current_location].predecessor[1]][self.agent_tree[self.current_location].predecessor[0]], [0,0,0]) == True:
 #           print('predecessor is blocked')
        print(self.agent_tree[self.current_location].predecessor, self.goal)
        #cv2.imshow(self.imageWindowName, self.initial_image)
        return self.rearrange_Lists()

    #If theres a problem, check here or castrays.
    def rearrange_Lists(self):
        self.closed.discard(self.agent_tree[self.current_location].predecessor)
        TEMP_LIST = [self.agent_tree[self.current_location].predecessor]
        print(len(self.open))
        while TEMP_LIST:
            tt = TEMP_LIST.pop(random.randrange(len(TEMP_LIST)))
            tt_successors = self.get_neighbors(tt, True)
            for t in tt_successors:
                if t == self.agent_tree[tt].predecessor or t not in self.agent_tree:
                    continue
                if t in self.agent_tree:
                    if self.agent_tree[t].predecessor == tt:
                        if t in self.open_Set_Check:
                            self.open_Set_Check.discard(t)
                            if (nop:=self.isOnOpenQueueAndCostCheck(None, t))[0] == True:
                                self.open.pop(nop[1])
                            TEMP_LIST.append(t)

                        elif t in self.closed:
                            self.closed.discard(t)
                            TEMP_LIST.append(t)

                    elif t in self.closed:
                        self.closed.discard(t)
                        self.open_Set_Check.add(t)
                        heapq.heappush(self.open, (self.agent_tree[t].heuristic_queue_val, t))

        for a in range(len(self.open)):
            if self.agent_tree[self.open[a][1]].cost == math.inf:
                self.agent_tree[self.open[a][1]].heuristic_queue_val
            else:
                self.agent_tree[self.open[a][1]].heuristic_queue_val = self.agent_tree[self.open[a][1]].cost+self.eudis5(self.current_location,self.open[a][1])
            self.open[a] = (self.agent_tree[self.open[a][1]].heuristic_queue_val , self.open[a][1])

        heapq.heapify(self.open)
        print(self.open[0], self.open[1])
        print(len(self.open))
        return self.CalculatePath()



                
                        

                        






