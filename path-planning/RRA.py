import math
import queue
import heapq
import time
import numpy as np
import random
import cv2


#predecessor is pointing towards the -> goal
class Node:
    def __init__(self, predecessor=None):
        self.predecessor = predecessor
        self.children = []
        self.cost = math.inf
         

    def change_parent(self, new_parent):
        self.predecessor = new_parent

    def remove_child(self, child):
        if child in self.children:
            self.children.remove(child)
        return None

    def add_Child(self, child):
        self.children.append(child)


class RRASEARCHTREE:
    def __init__(self, start_config, goal_config, image = None, imageWindowName = None, inclination_angle_in_degrees = None, current_location = None):
        self.start = start_config
        if current_location is None:
            self.current_location = self.start
        else:
            self.current_location = current_location
        self.goal = goal_config
        self.img = image
        self.img2 = None
        self.first_run = True
        self.window = imageWindowName
        self.imgHeight = len(image)
        self.imgWidth = len(image[0])
        self.open = []
        self.path = []
        self.closed = []
        self.agent_tree = {}
        self.inclination_angle = inclination_angle_in_degrees
        self.agent_tree[self.goal] = Node()
        self.agent_tree[self.goal].cost = 0
        print('Beginning search from:', self.current_location, 'to:', self.goal)
        
    def euclidean_distance_grid(self,a,b):
        d = math.hypot(b[0]-a[0],b[1]-a[1])
        return d

   # def angleOfInclination(self, point1, point2):
        #slope = mtantheta
        #add 180 if angle is negative

    def get_neighbors(self,pos,get_closed=False):
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
            if nextCell in self.closed and get_closed == False:
                continue
           # if np.array_equal(self.img[nextCell[1]][nextCell[0]],[0,0,0]):
           # if self.inclination_angle is not None and self.inclination_angle < self.angleOfInclination(pos, nextCell):
           #     continue

            neighbors.append(nextCell)

        return neighbors

    def changeCurrentLocation(self, new_location):
        self.current_location = new_location


    def isOnOpenQueueAndCostCheck(self, cost_val, state):
        if cost_val is None:
            for a in range(len(self.open)):
                if self.open[a][1] == state:
                    return True, a
            return False, None

        for a in range(len(self.open)):
            if self.open[a][1] == state:
                if self.open[a][0] > cost_val:
                    return True, a
                else:
                    return True, None
        return False, None


    def Initialize(self):
        if self.goal in self.closed:
            self.closed.remove(self.goal)

        checku = self.isOnOpenQueueAndCostCheck(None, self.goal)
        if checku[0] == False:
            heapq.heappush(self.open, (self.euclidean_distance_grid(self.current_location, self.goal), self.goal))
        self.agent_tree[self.goal].predecessor = None


    def getInitialPath(self):
        return self.pathCalculation()

    def updateImage(self, new_img):
        self.img = new_img

    def replan(self):
        return self.Navigate()

    def pathCalculation(self):
        if len(self.open) == 0:
            self.Initialize()
        self.agent_tree[self.goal].cost = 0
        while self.open:
            tmp_state = heapq.heappop(self.open)
            self.closed.append(tmp_state[1])
            if tmp_state[0] == math.inf:
                print('path not found')
                return None
            loop_cont = self.get_neighbors(tmp_state[1])
            for p in loop_cont:
                cost_va = 0
                f_val = 0
                if np.array_equal(self.img[p[1]][p[0]],[0,0,0]) == False and self.first_run:
                    cost_va = 1 + self.agent_tree[tmp_state[1]].cost 
                    f_val = cost_va + self.euclidean_distance_grid(self.current_location, p)

                elif self.first_run:
                    cost_va = math.inf
                    f_val = math.inf

                elif self.img2 is not None: 
                    if np.array_equal(self.img2[p[1]][p[0]], [0,0,0]) == False:
                        cost_va = 1 + self.agent_tree[tmp_state[1]].cost 
                        f_val = cost_va + self.euclidean_distance_grid(self.current_location, p)

                    else:
                        cost_va = math.inf
                        f_val = math.inf




                checker, tmp_c = self.isOnOpenQueueAndCostCheck(f_val, p)
                if checker == True and tmp_c is not None:
                    self.open.pop(tmp_c)
                    heapq.heapify(self.open)
                    self.agent_tree[p].predecessor, self.agent_tree[p].cost = tmp_state[1], cost_va
                    heapq.heappush(self.open, (f_val, p))
                    
                elif checker == False and p not in self.agent_tree: 
                    self.agent_tree[p] = Node()
                    self.agent_tree[p].predecessor = tmp_state[1]
                    self.agent_tree[p].cost = cost_va
                    heapq.heappush(self.open, (f_val , p))

                elif checker == False:
                    self.agent_tree[p].predecessor = tmp_state[1]
                    self.agent_tree[p].cost = cost_va
                    heapq.heappush(self.open, (f_val, p))

            if tmp_state[1] == self.current_location:
                self.first_run = False
                return self.Navigate()

    def Navigate(self):
        print('In navigation process')
        bool_checker = False
        while True:
            if self.agent_tree[self.current_location].predecessor is not None:
                if self.img2 is not None:
                    if np.array_equal(self.img2[self.agent_tree[self.current_location].predecessor[1]][self.agent_tree[self.current_location].predecessor[0]], [0,0,0]):
                        print('path not found. replanning.')
                        break
                else:
                    if np.array_equal(self.img[self.agent_tree[self.current_location].predecessor[1]][self.agent_tree[self.current_location].predecessor[0]], [0,0,0]):
                        print('path not found. replanning.')
                        break
            self.current_location = self.agent_tree[self.current_location].predecessor

            neighbor_list = self.get_neighbors(self.current_location)
            for p in neighbor_list:
                if self.img2 is not None:
                    if np.array_equal(self.img2[p[1]][p[0]], [0,0,0]) and self.agent_tree[p] != math.inf:
                        self.agent_tree[p].cost = math.inf

                else:
                    if np.array_equal(self.img[p[1]][p[0]], [0,0,0]) and self.agent_tree[p] != math.inf:
                        self.agent_tree[p].cost = math.inf

            if self.current_location == self.goal:
                print('path found.')
                bool_checker = True
                break

       

        if bool_checker:
            self.current_location = self.start
            while self.current_location is not None:
                self.path.append(self.current_location)
                self.current_location = self.agent_tree[self.current_location].predecessor
            self.current_location = self.start
            return self.path 

        else:
            print('rearranging.')
            return self.rearrange_lists()
            

    def rearrange_lists(self):
        print('rearranging lists.')
        put_on_temp = None
        temp_LIST = []
        if self.agent_tree[self.current_location].predecessor in self.closed:
            put_on_temp = self.closed.pop(self.closed.index(self.agent_tree[self.current_location].predecessor))
        if put_on_temp is not None:
            temp_LIST = [put_on_temp]
        while temp_LIST:
            pg = temp_LIST.pop(temp_LIST.index(random.sample(temp_LIST, 1)[0]))
            pg_neighbors = self.get_neighbors(pg, get_closed=True)
            for i in pg_neighbors:
                open_queue_check = self.isOnOpenQueueAndCostCheck(None, i)
                if i not in self.agent_tree:
                    continue
                elif self.agent_tree[i].predecessor == pg and open_queue_check[0]:
                    check = self.open.pop(open_queue_check[1])[1]
                    heapq.heapify(self.open)
                    temp_LIST.append(check)

                elif self.agent_tree[i].predecessor == pg and i in self.closed:
                    self.closed.remove(i)
                    temp_LIST.append(i)

                elif self.agent_tree[i].predecessor != pg and i in self.closed:
                    self.closed.remove(i)
                    heapq.heappush(self.open, (math.inf, i))
        updateList = []
        for point in range(len(self.open)):
            f_val = 0
            if np.array_equal(self.img2[self.open[point][1][1]][self.open[point][1][0]], [0,0,0]):
                f_val = math.inf

            else:
                f_val = self.euclidean_distance_grid(self.current_location, self.open[point][1]) + self.agent_tree[self.open[point][1]].cost
            self.open[point] = (f_val, self.open[point][1])

        heapq.heapify(self.open)
        print('done')

        return self.pathCalculation()




        

















