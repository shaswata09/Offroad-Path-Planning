import os
from pickle import NONE
import numpy as np
import os
import torch
import random
import math
from copy import deepcopy
import heapq
from torch.autograd import Variable
from sklearn.pipeline import make_pipeline
import torch
import cv2
import sys
sys.path.insert(1, 'C:/Users/charles/Downloads/Path-Planning-On-Aerial-Images-main-20220523T022800Z-001/Path-Planning-On-Aerial-Images-main/modules/path_planning')


class Node:
    def __init__(self,  rhs = np.inf, cost=np.inf):
        self.rhs = rhs 
        self.cost = cost
        self.key = None
        self.hval = 0
        self.predecessor = None

class URD:
    def __init__(self, start, goal, GroundTruthImage, threshold_replan_limit = 0.5, saved=False, pred_matrix=None, input_window=600):
        print("Beginning URD Traversal from", start, 'to', goal)
        self.start = start
        self.goal = goal
        #self.pred_matrix = pred_matrix
        self.pred_matrix = pred_matrix
        self.img_width = len(GroundTruthImage)
        self.img_height = len(GroundTruthImage[0])
        self.GroundTruthImage = GroundTruthImage
        self.replan_to_ellipse = 20
        self.nodeTree = {}
        self.threshold_lim = threshold_replan_limit
        self.kM = 0
        self.changedNodeList = []
        self.input_window = input_window
        self.open_Set_check = set()
        self.randarray = np.linspace(0, 2 * np.pi, 300)
        self.ep_val = 6.2
        self.nodes_expanded_in_x = 0
        self.nodes_expanded_in_y = 0
        self.confidence_metric = 0.0
        self.nodes_expanded = 0
        self.local_squares = 5
        self.path = []
        self.reinforcement_network = None
        self.saved_pre_search = saved
        self.replans = 0
        self.reference_points = []
        self.U = []
        self.segmentatedImage = np.zeros([self.img_width,self.img_height,3],dtype=np.uint8)
        self.segmentatedImage.fill(255)
        self.old_segmentatedImage = deepcopy(self.segmentatedImage)
        self.u_val_const = 0.7
        self.custom_h_weight = np.sqrt(2)

    def get_neighbors(self,pos,closed=None,get_closed=True):
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
            if closed is not None and nextCell in closed and get_closed == False:
                continue


            neighbors.append(nextCell)

        return neighbors
    def static_fvalue(self, s):
        return self.nodeTree[s].cost + (self.eudis5(s, self.start)  - (self.pred_matrix[s[1]][s[0]][1] * 1000))
    def get_diagonals(self, state):
        diagonals = [tuple((state[0]+1, state[1]+1)), tuple((state[0]-1, state[1]+1)), tuple((state[0]+1, state[1]-1)), tuple((state[0]-1, state[1]-1))]
        diagonals = [x for x in diagonals if x[0] >=0 and x[0] < self.img_height and x[1]>=0 and x[1] < self.img_width]		
        return diagonals
    def OpenQueueAndIndexCheck(self, state, cost_val=None):
        if cost_val is None:
            for a in range(len(self.U)):
                if self.U[a][1] == state:
                    return True, a
            print(state, 'not found on open list.')
            return False, None

        for a in range(len(self.U)):
            if self.U[a][1] == state:
                if self.U[a][0] > cost_val:
                    return True, a
                else:
                    return True, None
        return False, None
    def UpdateVertex(self, state):
        if self.nodeTree[state].rhs != self.nodeTree[state].cost and state in self.open_Set_check:
            oop = self.OpenQueueAndIndexCheck(state)[1]
            self.U[oop] = (self.ComputeKey(state), state)
            heapq.heapify(self.U)

        elif self.nodeTree[state].rhs != self.nodeTree[state].cost and state not in self.open_Set_check:
            self.open_Set_check.add(state)
            heapq.heappush(self.U, (self.ComputeKey(state), state))

        elif self.nodeTree[state].rhs == self.nodeTree[state].cost and state in self.open_Set_check:
            self.open_Set_check.discard(state)
            oop = self.OpenQueueAndIndexCheck(state)[1]
            self.U.pop(oop)
            heapq.heapify(self.U)
    def checkPointinEllipse(self, state, ring_ops):
       if ring_ops == 1:
            if (state[1]/self.ring_1_x/2.)**2 + (state[0]/self.ring_1_y/2.)**2 - 1 < 0:
                return True
       elif ring_ops == 2:
           if (state[1]/self.ring_2_x/2.)**2 + (state[0]/self.ring_2_y/2.)**2 - 1 < 0:
               return True
       return False
    def buildPath(self):
        cur_Node = self.start
        self.path = []
        while cur_Node != None:
            self.path.append(cur_Node)
            if cur_Node == self.goal:
                break
            cur_Node = self.nodeTree[cur_Node].predecessor
        self.path.reverse()
    def retrieve_solution_quality(self):
        self.solution_quality = 0.0
        map_path = []
       # print(path_runner.current_location)
        first_point = self.path[0]
        second_point = None
    
        for pos in self.path:
            map_path.append(pos)
      #      cv2.rectangle(img,pos,(pos[0]+2,pos[1]+2),color=(0,0,255),thickness=-1)
            if pos != first_point:
                second_point = pos
                first_point = pos
            if pos != self.path[0]:
                self.solution_quality += math.hypot((map_path[-1][0] - map_path[-2][0]),(map_path[-1][1] - map_path[-2][1]))
        return self.solution_quality
    def Initialize(self, search_method='URA'):
        if self.saved_pre_search == True:
            print('Loading Tree')
#load saved dict here. 
        else:
            if search_method == 'URA':
                        assert(self.pred_matrix is not None)
                        INCONS = set()
                        open_q = []
                        closed = set()
                        def get_minimum_e_val():
                           # incons = list(self.INCONS).append(self.open[0][1])
                            incons = list(INCONS)
                            for open_node in open_q:
                                incons.append(open_node[1])
                            minimum = np.inf
                            for a in incons:
                                if (tg:=self.nodeTree[a].cost + self.nodeTree[a].hval) < minimum:
                                    minimum = tg
                            return minimum


                        def ImprovePath():
                            while self.static_fvalue(self.start) > (open_q[0])[0] or open_q[0][1] != self.start:
                                top = heapq.heappop(open_q )
                                self.nodes_expanded += 1
                                closed.add(top[1])
                                top_neighbors = self.get_neighbors(top[1], closed)
                                for a in top_neighbors: 
                                    if a == self.nodeTree[top[1]].predecessor:
                                        continue

                                  #  if self.Tree[a].cost > (tmp_cost:=self.Tree[top[1]].cost+self.UnpredictableCostGetter(a)):
                                    if self.nodeTree[a].cost > (tmp_cost:=self.nodeTree[top[1]].cost+self.pred_matrix[a[1]][a[0]][0]+self.linkCostGrabber(top[1],a)):
                                    #if self.Tree[a].cost > (tmp_cost:=self.Tree[top[1]].cost+1):
                                        self.nodeTree[a].cost = tmp_cost
                                        self.nodeTree[a].rhs = tmp_cost
                                        self.nodeTree[a].predecessor = top[1]
                                        if a not in closed:
                                            heapq.heappush(open_q, (self.static_fvalue(a), a))
                                        else:
                                            INCONS.add(a)

                        first_param = 0.4
                        second_param = 35
                        third_param = 3.3
                        temp_pred_mat = self.pred_matrix[:, :, 0]

                        open_set_check = set()
                        predictionMatrix_heuristic = 35
                        cost_heuristic = 10
                        current_location = self.start
                        self.s, self.g = np.asarray(self.start), np.asarray(self.goal)
                        for a in range(self.img_width):
                           for b in range(self.img_height):
                               self.nodeTree[(b, a)] = Node()

                        self.nodeTree[self.goal] = Node(cost=0)
                        self.nodeTree[self.goal].rhs = 0
                        self.nodeTree[self.goal].hval = self.eudis5(self.start, self.goal)

                       # self.Tree[self.start].hval = self.eudis5(self. start, self.goal)

                        heapq.heappush(open_q, (self.static_fvalue(self.goal), self.goal))


                        ImprovePath()
                        optimal_e = min(self.ep_val, get_minimum_e_val())
                        while optimal_e > 1.0:
                            self.ep_val -= 0.8
                            for a in list(INCONS): 
                                INCONS.discard(a)
                                open_q.append((np.inf, a))
                            for l in range(len(open_q)):
                                open_q[l] = (self.static_fvalue(open_q[l][1]), open_q[l][1])

                            heapq.heapify(open_q)
                            closed = set()
                            ImprovePath()
                           # yield self.path
                            optimal_e = min(self.ep_val, self.nodeTree[self.start].cost / get_minimum_e_val())
                        self.buildPath()
                     
            else:
                print('stuff goes here')


	#def ComputeCost(self, )
    def linkCostGrabber(self, state1, state2, old=False):
        if old == False and np.array_equal(self.segmentatedImage[state2[1]][state2[0]], [255,0,0]) or np.array_equal(self.segmentatedImage[state1[1]][state1[0]], [255,0,0]):
            return math.inf
        elif old == True and np.array_equal(self.old_segmentatedImage[state2[1]][state2[0]], [255,0,0]) or np.array_equal(self.old_segmentatedImage[state1[1]][state1[0]], [255,0,0]):
            return math.inf
        diagonals = self.get_diagonals(state1)
        if state2 in diagonals:
            return 1.1
        else:
            return 1
    def castRays(self, x,y):
        final_dist = 100
        randarray = np.linspace(0, 2*np.pi,	500)
        for xr in randarray:
            x1 = int(x + final_dist * np.cos(xr))
            y1 = int(y + final_dist * np.sin(xr))
            self.bresenham((x,y), (x1,y1)) 
			#self.draw_line((x,y), (x1, y1))
        return  
    def bresenham(self, start, end):
            (x0,y0) = start
            (x1,y1) = end

					#known obstacle

            if np.array_equal(self.segmentatedImage[y0][x0], [255, 0,0]):
                self.pred_matrix[y0][x0][1]  = 0.0

                return False, None

					#predicted obstacle that was correct
            elif np.array_equal(self.segmentatedImage[y0][x0], self.GroundTruthImage[y0][x0]) and np.array_equal(self.GroundTruthImage[y0][x0], [0,0,0]):
                self.changedNodeList.append(tuple((x0,y0)))
                self.segmentatedImage[y0][x0] = [255,0,0]
                self.pred_matrix[y0][x0][1]  = 0.0
                return False, None

					#predicted obstacle that was free space
            elif np.array_equal(self.segmentatedImage[y0][x0], [0,0,0]) and np.array_equal(self.GroundTruthImage[y0][x0], [255, 255, 255]):
                self.changedNodeList.append(tuple((x0,y0)))
                self.segmentatedImage[y0][x0] = [255,255,255]
                self.pred_matrix[y0][x0][1]  = 1.0


					#predicted free space that was an obstacle
            elif np.array_equal(self.segmentatedImage[y0][x0], [255, 255, 255]) and np.array_equal(self.GroundTruthImage[y0][x0], [0, 0, 0]):
                self.changedNodeList.append(tuple((x0,y0)))
                self.segmentatedImage[y0][x0] = [255,0,0]
                self.pred_matrix[y0][x0][1]  = 0.0

                return False, None





            elif start[0] < 0 or start[1] < 0 or start[1] >= self.img_height or start[0] >= self.img_width:
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

					#known obstacle
                    if np.array_equal(self.segmentatedImage[y0][x0], [255, 0,0]):
                        self.pred_matrix[y0][x0][1]  = 0.0

                        return False, None

					#predicted obstacle that was correct
                    elif np.array_equal(self.segmentatedImage[y0][x0], self.GroundTruthImage[y0][x0]) and np.array_equal(self.GroundTruthImage[y0][x0], [0,0,0]):
                        self.changedNodeList.append(tuple((x0,y0)))
                        self.segmentatedImage[y0][x0] = [255,0,0]
                        self.pred_matrix[y0][x0][1]  = 0.0

                        return False, None

					#predicted obstacle that was free space
                    elif np.array_equal(self.segmentatedImage[y0][x0], [0,0,0]) and np.array_equal(self.GroundTruthImage[y0][x0], [255, 255, 255]):
                        self.changedNodeList.append(tuple((x0,y0)))
                        self.pred_matrix[y0][x0][1]  = 1.0

                        self.segmentatedImage[y0][x0] = [255,255,255]

					#predicted free space that was an obstacle
                    elif np.array_equal(self.segmentatedImage[y0][x0], [255, 255, 255]) and np.array_equal(self.GroundTruthImage[y0][x0], [0, 0, 0]):
                        self.changedNodeList.append(tuple((x0,y0)))
                        self.segmentatedImage[y0][x0] = [255,0,0]
                        self.pred_matrix[y0][x0][1]  = 0.0

                        return False, None




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

					#known obstacle
                    if np.array_equal(self.segmentatedImage[y0][x0], [255, 0,0]):
                        self.pred_matrix[y0][x0][1]  = 0.0

                        return False, None

					#predicted obstacle that was correct
                    elif np.array_equal(self.segmentatedImage[y0][x0], self.GroundTruthImage[y0][x0]) and np.array_equal(self.GroundTruthImage[y0][x0], [0,0,0]):
                        self.changedNodeList.append(tuple((x0,y0)))
                        self.pred_matrix[y0][x0][1] = 0.0

                        self.segmentatedImage[y0][x0] = [255,0,0]
                        return False, None

					#predicted obstacle that was free space
                    elif np.array_equal(self.segmentatedImage[y0][x0], [0,0,0]) and np.array_equal(self.GroundTruthImage[y0][x0], [255, 255, 255]):
                        self.changedNodeList.append(tuple((x0,y0)))
                        self.pred_matrix[y0][x0][1] = 1.0

                        self.segmentatedImage[y0][x0] = [255,255,255]

					#predicted free space that was an obstacle
                    elif np.array_equal(self.segmentatedImage[y0][x0], [255, 255, 255]) and np.array_equal(self.GroundTruthImage[y0][x0], [0, 0, 0]):
                        self.changedNodeList.append(tuple((x0,y0)))
                        self.segmentatedImage[y0][x0] = [255,0,0]
                        self.pred_matrix[y0][x0][1] = 0.0

                        return False, None


            return True, end
    def eudis5(self, v1, v2):
        dist = [(a - b)**2 for a, b in zip(v1, v2)]
        return np.sqrt(sum(dist))
    def ComputeKey(self, state):
        #return (min(self.nodeTree[state].cost, self.nodeTree[state].rhs)+self.kM+self.eudis5(state, self.start) - self.pred_matrix[state[1]][state[0]], min(self.nodeTree[state].cost, self.nodeTree[state].rhs))
        return (min(self.nodeTree[state].cost, self.nodeTree[state].rhs)+self.kM+self.CalculateHeuristic(state), min(self.nodeTree[state].cost, self.nodeTree[state].rhs))

    def CalculateHeuristic(self, current_state):
        u_val = self.orig_u_val(current_state)
        return min(self.custom_h_weight*u_val*min(np.abs(self.start[0]-current_state[0]), np.abs(self.start[1]-current_state[1])) + u_val*np.abs(np.abs(self.start[0]-current_state[0])-np.abs(self.start[1]-current_state[1])) , self.eudis5(current_state, self.start))


    def orig_u_val(self, st):
        if st == self.start:
            return 0
        return np.reciprocal((np.sqrt( (self.start[0]- st[0])**2 + (self.start[1]- st[1])**2) / np.sqrt(  (self.start[0]-self.goal[0])**2+(self.start[1]-self.goal[1])**2  ) )+ self.u_val_const) 


    def ComputeShortestPath(self):
        while len(self.U) > 0 and (self.U[0][0] < self.ComputeKey(self.start1) or self.nodeTree[self.start1].cost < self.nodeTree[self.start1].rhs):
		#	print(len(self.U), self.nodeTree[self.start].rhs)
			
            k_old = heapq.heappop(self.U)
		#	print(self.nodeTree[k_old[1]].rhs, self.eudis5(self.start, k_old[1]), k_old[1])
            k_new = self.ComputeKey(k_old[1])
            self.open_Set_check.discard(k_old[1])

            if k_old[0] < k_new:
                self.open_Set_check.add(k_old[1])
                heapq.heappush(self.U, (k_new, k_old[1]))
			
            elif self.nodeTree[k_old[1]].cost > self.nodeTree[k_old[1]].rhs:
                self.nodeTree[k_old[1]].cost = self.nodeTree[k_old[1]].rhs

                neighbors = self.get_neighbors(k_old[1])
                for n in neighbors:
                    if n != self.goal:
                        self.nodeTree[n].rhs = min(self.nodeTree[n].rhs, self.linkCostGrabber(n, k_old[1])+self.nodeTree[k_old[1]].cost)
                    self.UpdateVertex(n)
            else:
                g_old = self.nodeTree[k_old[1]].cost
                self.nodeTree[k_old[1]].cost = math.inf
                neighbors = self.get_neighbors(k_old[1])
                neighbors.append(k_old[1])

                for n in neighbors:
                    if self.nodeTree[n].rhs == self.linkCostGrabber(n, k_old[1])+g_old:
                        if n != self.goal:
                            min_val = math.inf
                            new_neighbors = self.get_neighbors(n)
                            for p in new_neighbors:
                                if min_val > (new_c:=self.linkCostGrabber(n, p)+self.nodeTree[p].cost):
                                    min_val = new_c
                            self.nodeTree[n].rhs = min_val
                    self.UpdateVertex(n)

    def URD(self):
        s_last = self.start
        self.start1 = s_last
        start_cond = False
        self.castRays(self.start[0], self.start[1])

        ax = np.expand_dims((self.pred_matrix[:, :, 1]).flatten(), 0)
        action = None

        #self.reinforcement_network.update(reward=0, new_signal =ax, action=action)

#        self.castRays(self.start[0], self.start[1])
        self.changedNodeList = []

        self.Initialize()
        self.new_path = []
        self.new_path.append(self.start)
        cnt = 0
        search_reset = False
        second_to_last = None
        last = None
        counter = 0
        self.ind = False
        while self.start1 != self.goal:

            if self.nodeTree[self.start1].rhs == math.inf:
                print('No Known Path')
                return self.new_path

            neighbors = self.get_neighbors(self.start1)
            min_val = math.inf
            new_state = None

            counter +=1
            if counter == 1:
                last = self.start1
            elif counter == 2:
                second_to_last = self.start1 
                counter = 0

            for n in neighbors:
                if (temp_val:=self.linkCostGrabber(self.start1, n) + self.nodeTree[n].cost) < min_val:
                    min_val = temp_val
                    new_state = n

            if new_state is not None:
                self.start1 = new_state
            else:
                print('Path is blocked.')
                return self.new_path

            self.new_path.append(self.start1)
            if second_to_last == self.start1:
                self.replans += 1
                self.start = self.start1
                self.changedNodeList = []
                self.Initialize()
                self.U = []
                search_reset = True
                continue

            self.castRays(self.start1[0], self.start1[1])
            self.changedNodeList = list(set(self.changedNodeList))
            if len(self.changedNodeList) > 0:
                self.replans += 1
                if self.replans % 10 == 0:
                    if search_reset == True:
                        self.custom_h_weight /= 1.7
                        search_reset = False
                    else:
                        self.custom_h_weight += 1.2
                self.kM = self.CalculateHeuristic(self.start1) + self.kM
                s_last = self.start1
                for a in self.changedNodeList:
                    neighbors = self.get_neighbors(a)
                    for n in neighbors:
                        c_old = self.linkCostGrabber(n, a, True)
                        if c_old > (new_link_cost:=self.linkCostGrabber(n, a)):
                            if n != self.goal:
                                self.nodeTree[n].rhs = min(self.nodeTree[n].rhs, new_link_cost+self.nodeTree[a].cost)

                        elif self.nodeTree[n].rhs == c_old + self.nodeTree[a].cost:
                            if n != self.goal:
                                new_neighbors = self.get_neighbors(n)
                                min_val = math.inf
                            for ne in new_neighbors:
                                if min_val < (l_cost:=self.nodeTree[ne].cost+self.linkCostGrabber(n, ne)):
                                    min_val = l_cost
                            self.nodeTree[n].rhs = min_val
                        self.UpdateVertex(n)
                    self.UpdateVertex(a)




                heapq.heapify(self.U)
                self.changedNodeList = []
                self.ComputeShortestPath()
                self.old_segmentatedImage = deepcopy(self.segmentatedImage)

        return self.new_path
