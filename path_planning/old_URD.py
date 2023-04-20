import os
from pickle import NONE
import numpy as np
import os
import random
import math
from copy import deepcopy
import heapq
import cv2


class Node:
    def __init__(self,  rhs = np.inf, cost=np.inf):
        self.rhs = rhs 
        self.cost = cost
        self.key = None
        self.hval = 0
        self.changed = False
        self.predecessor = None


class ELLIPSE:
    def __init__(self, a, b, num_points, start, end):
        self.a = a
        self.b = b
        self.num_points = num_points
        self.start = start
        self.end = end
        self.angle_gen = math.atan2(self.end[1]-self.start[1], self.end[0]-self.start[0])
    
    def generate_theta(self, a, b):
            u = random.random() / 4.0
            theta = np.arctan(self.b/self.a * np.tan(2*np.pi*u))

            v = random.random()
            if v < 0.25:
                return theta
            elif v < 0.5:
                return np.pi - theta
            elif v < 0.75:
                return np.pi + theta
            else:
                return -theta

    def radius(self, a, b, theta):
            return self.a * self.b / np.sqrt((b*np.cos(theta))**2 + (a*np.sin(theta))**2)


    def random_point(self, major_axis, minor_axis, center, qa):
            random_theta = self.generate_theta(self.a, self.b)
            max_radius = self.radius(self.a, self.b, random_theta)
            random_radius = max_radius * np.sqrt(random.random())
            f = round(random_radius * np.cos(random_theta))
            s = round(random_radius * np.sin(random_theta))
            lio = self.rotate((0, 0), (f, s), self.angle_gen)
            return (int(lio[0]+center[0]), int(lio[1]+center[1]))


    def rotate(self, origin, point, angle):
            """
            Rotate a point counterclockwise by a given angle around a given origin.

            The angle should be given in radians.
            """
            ox, oy = origin
            px, py = point

            qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
            qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
            return qx, qy

    def midpoint(self, p1, p2):
            return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)

    def ret_list(self):
        points = [self.random_point(self.a, self.b, self.midpoint(self.start, self.end),  self.angle_gen) for _ in range(self.num_points)]
        points = [x for x in points if x[0] >= 0 and x[1] >= 0]
        return points






class URD:
    def __init__(self, start, goal, GroundTruthImage, threshold_replan_limit = 0.5, saved=False, pred_matrix=None):
        print("Beginning URD Traversal")
        self.start = start
        self.goal = goal
        self.pred_matrix = pred_matrix
        self.img_width = len(GroundTruthImage)
        self.img_height = len(GroundTruthImage[0])
        self.GroundTruthImage = GroundTruthImage
        self.replan_to_ellipse = 20
        self.nodeTree = {}
        self.threshold_lim = threshold_replan_limit
        self.kM = 0
        self.changedNodeList = []

        self.open_Set_check = set()
        self.randarray = np.linspace(0, 2 * np.pi, 300)
        self.ep_val = 6.2
        self.nodes_expanded_in_x = 0
        self.nodes_expanded_in_y = 0
        self.nodes_expanded = 0
        self.path = []
        self.saved_pre_search = saved
        self.ring_2_x = 0.0
        self.ring_2_y = 0.0
        self.ring_1_x = 0.0
        self.ring_1_y = 0.0
        self.replans = 0
        self.U = []
        self.segmentatedImage = np.zeros([self.img_width,self.img_height,3],dtype=np.uint8)
        self.segmentatedImage.fill(255)
        self.old_segmentatedImage = deepcopy(self.segmentatedImage)


    #        image_ellipse = ELLIPSE(self.c_i, math.sqrt((self.c_i**2) - (self.minimum_cost_distance)**2), self.m_samples, self.start, self.goal)



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
#scan first
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
            self.U[oop] = (self.CalculateKey(state), state)
            heapq.heapify(self.U)

        elif self.nodeTree[state].rhs != self.nodeTree[state].cost and state not in self.open_Set_check:
            self.open_Set_check.add(state)
            heapq.heappush(self.U, (self.CalculateKey(state), state))

        elif self.nodeTree[state].rhs == self.nodeTree[state].cost and state in self.open_Set_check:
            self.open_Set_check.discard(state)
            oop = self.OpenQueueAndIndexCheck(state)[1]
            self.U.pop(oop)
            heapq.heapify(self.U)



#fix
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

                        print("Beginning initial grid search", self.start, "to", self.goal)
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
                            print(optimal_e, self.ep_val, self.nodeTree[self.start].cost / get_minimum_e_val())
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
                        self.retrieve_solution_quality()
                        self.ring_1_x = self.solution_quality
                        self.ring_1_y = 0.5*math.sqrt((self.solution_quality**2) - (self.eudis5(self.goal, self.start)**2))
                        image_ellipse = ELLIPSE(self.solution_quality, 3*math.sqrt((self.solution_quality**2) - (self.eudis5(self.goal, self.start)**2)), 500, self.start, self.goal)
                        l = image_ellipse.ret_list()
                       # for a in l:
                      #      if a[1] >= self.img_height or a[0] >= self.img_width:
                      #          continue
                     #       self.GroundTruthImage[a[1]][a[0]] = [0, 255, 0]
                        cv2.imshow("image", self.GroundTruthImage)




                            #print("ARA* initial path has an accuracy of: ", (ratio/len(self.path)) * 100, "%")

            else:
                print('stuff goes here')


	#def ComputeCost(self, )
        print('Finished constructing initial tree.')
        print(self.nodeTree[self.start].cost, self.nodeTree[self.goal].cost)


    def linkCostGrabber(self, state1, state2, old=False):
        if old == False and np.array_equal(self.segmentatedImage[state2[1]][state2[0]], [255,0,0]) or np.array_equal(self.segmentatedImage[state1[1]][state1[0]], [255,0,0]):
            return math.inf
        elif old == True and np.array_equal(self.old_segmentatedImage[state2[1]][state2[0]], [255,0,0]) or np.array_equal(self.old_segmentatedImage[state1[1]][state1[0]], [255,0,0]):
            return math.inf
        diagonals = self.get_diagonals(state1)
        if state2 in diagonals:
            return 2
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
                return False, None

					#predicted obstacle that was correct
            elif np.array_equal(self.segmentatedImage[y0][x0], self.GroundTruthImage[y0][x0]) and np.array_equal(self.GroundTruthImage[y0][x0], [0,0,0]):
                self.changedNodeList.append(tuple((x0,y0)))
                self.segmentatedImage[y0][x0] = [255,0,0]
                return False, None

					#predicted obstacle that was free space
            elif np.array_equal(self.segmentatedImage[y0][x0], [0,0,0]) and np.array_equal(self.GroundTruthImage[y0][x0], [255, 255, 255]):
                self.changedNodeList.append(tuple((x0,y0)))
                self.segmentatedImage[y0][x0] = [255,255,255]

					#predicted free space that was an obstacle
            elif np.array_equal(self.segmentatedImage[y0][x0], [255, 255, 255]) and np.array_equal(self.GroundTruthImage[y0][x0], [0, 0, 0]):
                self.changedNodeList.append(tuple((x0,y0)))
                self.segmentatedImage[y0][x0] = [255,0,0]
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
                        return False, None

					#predicted obstacle that was correct
                    elif np.array_equal(self.segmentatedImage[y0][x0], self.GroundTruthImage[y0][x0]) and np.array_equal(self.GroundTruthImage[y0][x0], [0,0,0]):
                        self.changedNodeList.append(tuple((x0,y0)))

                        self.segmentatedImage[y0][x0] = [255,0,0]
                        return False, None

					#predicted obstacle that was free space
                    elif np.array_equal(self.segmentatedImage[y0][x0], [0,0,0]) and np.array_equal(self.GroundTruthImage[y0][x0], [255, 255, 255]):
                        self.changedNodeList.append(tuple((x0,y0)))

                        self.segmentatedImage[y0][x0] = [255,255,255]

					#predicted free space that was an obstacle
                    elif np.array_equal(self.segmentatedImage[y0][x0], [255, 255, 255]) and np.array_equal(self.GroundTruthImage[y0][x0], [0, 0, 0]):
                        self.changedNodeList.append(tuple((x0,y0)))
                        self.segmentatedImage[y0][x0] = [255,0,0]
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
                        return False, None

					#predicted obstacle that was correct
                    elif np.array_equal(self.segmentatedImage[y0][x0], self.GroundTruthImage[y0][x0]) and np.array_equal(self.GroundTruthImage[y0][x0], [0,0,0]):
                        self.changedNodeList.append(tuple((x0,y0)))

                        self.segmentatedImage[y0][x0] = [255,0,0]
                        return False, None

					#predicted obstacle that was free space
                    elif np.array_equal(self.segmentatedImage[y0][x0], [0,0,0]) and np.array_equal(self.GroundTruthImage[y0][x0], [255, 255, 255]):
                        self.changedNodeList.append(tuple((x0,y0)))

                        self.segmentatedImage[y0][x0] = [255,255,255]

					#predicted free space that was an obstacle
                    elif np.array_equal(self.segmentatedImage[y0][x0], [255, 255, 255]) and np.array_equal(self.GroundTruthImage[y0][x0], [0, 0, 0]):
                        self.changedNodeList.append(tuple((x0,y0)))
                        self.segmentatedImage[y0][x0] = [255,0,0]
                        return False, None


            return True, end



    def eudis5(self, v1, v2):
        dist = [(a - b)**2 for a, b in zip(v1, v2)]
        return np.sqrt(sum(dist))


    def ComputeKey(self, state):
        return (min(self.nodeTree[state].cost, self.nodeTree[state].rhs)+self.kM+self.eudis5(state, self.start), min(self.nodeTree[state].cost, self.nodeTree[state].rhs))
    

    def calculateHeuristic(self, state):
        return self.eudis5(state, self.start) + 2.5 * self.pred_matrix[state[1]][state[0]][1]

    def ComputeShortestPath(self):
        while len(self.U) > 0 and (self.U[0][0] < self.CalculateKey(self.start) or self.nodeTree[self.start].cost < self.nodeTree[self.start].rhs):
		#	print(len(self.U), self.nodeTree[self.start].rhs)
			
            k_old = heapq.heappop(self.U)
		#	print(self.nodeTree[k_old[1]].rhs, self.eudis5(self.start, k_old[1]), k_old[1])
            k_new = self.CalculateKey(k_old[1])
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
        self.castRays(self.start[0], self.start[1])
#        self.castRays(self.start[0], self.start[1])
        self.changedNodeList = []

        self.Initialize()

        self.path.append(self.start)
    #    self.ComputeShortestPath()

        second_to_last = None
        last = None
        counter = 0
        while self.start != self.goal:
		#	print(self.start)
            if self.nodeTree[self.start].rhs == math.inf:
                print('No Known Path')
                return self.path

            neighbors = self.get_neighbors(self.start)
            min_val = math.inf
            new_state = None

            counter +=1
            if counter == 1:
                last = self.start
            elif counter == 2:
                second_to_last = self.start 
                counter = 0

            for n in neighbors:
                if (temp_val:=self.linkCostGrabber(self.start, n) + self.nodeTree[n].cost) < min_val:
                    min_val = temp_val
                    new_state = n

            if new_state is not None:
                self.start = new_state
            else:
                print('Path is blocked.')
                return self.path

            self.path.append(self.start)
            if second_to_last == self.start:
                print('second')

            self.castRays(self.start[0], self.start[1])
            self.castRays(self.start[0], self.start[1])

            self.changedNodeList = list(set(self.changedNodeList))
            if len(self.changedNodeList) > 0:
                self.replans += 1
                self.kM = self.eudis5(s_last, self.start) + self.kM
                s_last = self.start
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



				#for a in range(len(self.U)):
			#		self.U[a] = (self.CalculateKey(self.U[a][1]), self.U[a][1])

                heapq.heapify(self.U)
                self.changedNodeList = []
                self.ComputeShortestPath()
                self.old_segmentatedImage = deepcopy(self.segmentatedImage)


        return self.path
