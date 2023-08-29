
import math
import queue
import time
from turtle import update
import numpy as np
import cv2
import random
import tcod
import heapq
import queue


##################################################
#
#         A* Algorithm Code
#
##################################################

class SearchNode:
	def __init__(self,state,parent,cost):
		self.parent = parent # Pointer to parent node
		self.state = state # The current grid cell this node points to
		self.cost = cost  # The cumulative cost of this search node (sum of all actions to get here)
		self.h = 0 # This node's heuristic value
	
	def __lt__(self,other):
		return self.h < other.h

class Theta:
    def __init__(self,startPos,endPos,image,imageWindowName=None):
        #startPos and endPos are tuples
        #image is the numpy array from cv2
        #imageWindowName is the name of the image to update for the visual results
        self.V = {}
        self.startPos = startPos
        self.endPos = endPos
        self.img = image
        self.imgWidth = len(image[0])
        self.imgHeight = len(image)
        self.imgWindow = imageWindowName

    # This function returns the euclidean distance between two grid cells 
    def euclidean_distance_grid(self,a,b):
        d = math.hypot(b[0]-a[0],b[1]-a[1])
        return d

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

    def get_los(self, first_point, second_point):
        line_algo_points = tcod.los.bresenham(first_point, second_point).tolist()
        for a in line_algo_points:
            if np.array_equal(self.img[a[1]][a[0]], [0,0,0]):
                return False
    
        return True



    def isOnOpenQueueAndCostCheck(self, cost_val, state, fringe):
        if cost_val is None:
            for a in range(len(fringe)):
                if fringe[a][1] == state:
                    return True, a
            return False, None

        for a in range(len(fringe)):
            if fringe[a][1] == state:
                if fringe[a][0] > cost_val:
                    return True, a
                else:
                    return True, None
        return False, None




    def UpdateVertex(self, s, sp, openq):
        if self.get_los(self.V[s].parent, sp):
            if self.V[self.V[s].parent].cost+self.euclidean_distance_grid(self.V[s].parent,sp) < self.V[sp].cost:
                self.V[sp].cost = self.V[self.V[s].parent].cost+self.euclidean_distance_grid(self.V[s].parent,sp)
                self.V[sp].parent = self.V[s].parent
                t = self.isOnOpenQueueAndCostCheck(None, sp, openq)
                if t[0] == True:
                    openq.pop(t[1])
                openq.append((self.euclidean_distance_grid(sp,self.endPos) + (2 * max(abs(sp[0]-self.endPos[0]) , abs(sp[1]-self.endPos[1]))), sp))


                heapq.heapify(openq)

        else:

            if self.V[s].cost+self.euclidean_distance_grid(s,sp) < self.V[sp].cost:
                self.V[sp].cost = self.V[s].cost+self.euclidean_distance_grid(s,sp)
                self.V[sp].parent = s
                t = self.isOnOpenQueueAndCostCheck(None, sp, openq)
                if t[0] == True:
                    openq.pop(t[1])
                openq.append((self.euclidean_distance_grid(sp,self.endPos) + (2 * max(abs(sp[0]-self.endPos[0]) , abs(sp[1]-self.endPos[1]))),sp))






                heapq.heapify(openq)


    def run(self):
        self.V[self.startPos] = SearchNode(self.startPos, self.startPos, 0)
        fringe = []
        heapq.heappush(fringe, (self.euclidean_distance_grid(self.startPos,self.endPos) + (2 * max(abs(self.startPos[0]-self.endPos[0]) , abs(self.startPos[1]-self.endPos[1]))), self.startPos))
        closed = set()
        while fringe:
            s=heapq.heappop(fringe)
            if s[1] == self.endPos:
               # print('Subgoal path was found.')
                break
            closed.add(s[1])
            neighbors = self.get_neighbors(s[1],closed)
            for p in neighbors:
                if p not in closed:
                    #if p not in self.V or p in list(dict(fringe).values()):
                    t = self.isOnOpenQueueAndCostCheck(None, p, fringe)
                    if t[0] == False:
                        self.V[p] = SearchNode(p, None, math.inf)
                    self.UpdateVertex(s[1],p,fringe)

        if self.endPos in self.V and self.V[self.endPos].cost != math.inf:
            path = []
            cur_node = self.V[self.endPos]
            while cur_node is not None:
                if cur_node.state == self.startPos:
                    path.append(self.startPos)
                    break
                path.append(cur_node.state)
                cur_node = self.V[self.V[cur_node.state].parent]
            path.reverse()
            return path
        else:
            print('no solution')
        return

    def getPathCost(self):
        arru = self.run()
        if len(arru)==0:
            return None, math.inf
        solution_quality=0
        first_point=arru[0]
        map_path = []
        for pos in arru:
            map_path.append(pos)
            if pos != first_point:
                first_point = pos
            if pos != arru[0]:
                solution_quality += math.hypot((map_path[-1][0] - map_path[-2][0]),(map_path[-1][1] - map_path[-2][1]))

        return arru, solution_quality








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
        return points






class BIT:
    def __init__(self, start, goal, m_samples, image):
        self.start = start
        self.goal = goal
        self.m_samples = m_samples
        self.radius = math.inf
        self.trajectoryList = {}
        self.V = {}
        self.img = image
        self.QV = []
        self.X_reuse = set()
        self.QE = []
        self.V[self.goal] = self.Node()
        self.V[self.start] = self.Node()
        self.c_best = math.inf
        self.X_new = self.X_unconn = set([self.goal])
        self.img_height = len(image)
        self.minimum_cost_distance = self.euclidean_distance_grid(self.start, self.goal)
        self.img_width = len(image[0])
        self.V[self.start].cost = 0
        self.V[self.start].f_val = self.V[self.start].optimal_cost = self.minimum_cost_distance
        self.V_soln = self.InGoalRegion(list_of_points=True)
        self.c_i = math.inf
        self.V_unexpnd = set(self.V.keys())
        self.nodes_expanded = 0
        if len(self.V_soln) != 0:
            for z in self.V_soln:
                if (best:=self.V[z].cost+self.euclidean_distance_grid(z,self.goal)) < self.c_i:
                    if self.goal not in self.V:
                        self.V[self.goal] = self.Node()

                    self.c_i =  self.V[self.goal].cost = best
                    self.V[self.goal].parent = z

        print('The straight line distance is:',self.minimum_cost_distance)
        print('The current path cost is:',self.c_i)
        print('The starting f_val is: ',self.V[self.start].f_val)


    class Node:
        def __init__(self, parent=None):
            self.parent = parent
            self.cost = math.inf
            self.f_val = 0
            self.optimal_cost = -1

        def change_parent(self, new_parent):
            self.parent = new_parent
    def euclidean_distance_grid(self,a,b):
        d = math.hypot(b[0]-a[0],b[1]-a[1])
        return d
    
    def get_radius(self, first_arr, second_arr):
        number = len(first_arr) + len(second_arr)
        self.rewiring_radius = (math.sqrt(number) / math.sqrt(math.log2(number)))
        return max(self.rewiring_radius,100)


    def costGetter(self, vertex):
        if vertex in self.V:
            return self.V[vertex].cost

        else:
            return math.inf




    def InGoalRegion(self, point=None, list_of_points=False):
        if list_of_points ==True:
            return set(self.k_nearest_neighbors(self.goal, self.V.keys()))
        elif self.euclidean_distance_grid(point, self.goal) <= self.radius and self.get_los(point, self.goal):
            return True
        return False


        
    def k_nearest_neighbors(self, point, array_p, num_points=None, get_radius=True, get_los_bool=True):
        array_s = list(array_p)
        if get_radius and get_los_bool:
            array_to_return = []
            for jk in array_s:
                if self.euclidean_distance_grid(jk, point) <= self.radius and self.get_los(point,jk)  and jk != point:
                    array_to_return.append(jk)
            return array_to_return

        elif get_radius:
            array_to_return = []
            for jk in array_s:
                if self.euclidean_distance_grid(jk, point) <= self.radius and jk != point:
                    array_to_return.append(jk)
            return array_to_return

        temp_dict = {}
        return_Arr = []
        for k in self.tree.vertices_and_edges.keys():
            temp_dict[k] = self.euclidean_distance_grid(k, point)
        assert(len(self.tree.vertices_and_edges) > num_points)
        for a in range(num_points):
            z = min(temp_dict, key=temp_dict.get)
            del temp_dict[z] 
            return_Arr.append(z)

        return return_Arr

    def get_los(self, first_point, second_point):
        line_algo_points = tcod.los.bresenham(first_point, second_point).tolist()
        for a in line_algo_points:
            if np.array_equal(self.img[a[1]][a[0]], [0,0,0]):
                return False    
        return True

    def bestQValue(self, arpp):
        if len(arpp) == 0:
            return math.inf
        return arpp[0][0]

 

    def get_optimal_f_val(self, point):
        if point is None:
            return -1
        else:
            return self.V[point].f_val
      #  return self.euclidean_distance_grid(point, self.goal)+self.euclidean_distance_grid(self.start, point)


    # return number of samples that were infinity. 





    def Sample(self, n_samples=0):
        if self.c_i < math.inf:
            #   assert(self.costGetter(self.goal) >= self.minimum_cost_distance)

           # image_ellipse = ELLIPSE(self.c_i, math.sqrt((self.c_i**2) - (self.minimum_cost_distance)**2), self.m_samples+n_samples, self.start, self.goal)
            #diff = image_ellipse.ret_list()
            image_ellipse = ELLIPSE(self.c_i, math.sqrt((self.c_i**2) - (self.minimum_cost_distance)**2), self.m_samples, self.start, self.goal)
            image_ellipse.num_points = int(math.pi*image_ellipse.a*image_ellipse.b)
            stuff = image_ellipse.ret_list()
            stuff = {item for item in stuff if item[0] < self.img_width and item[0] >= 0 and item[1] < self.img_height and item[1] >= 0 and item not in self.V and item not in self.X_reuse and np.array_equal(self.img[item[1]][item[0]], [0,0,0]) == False}
            stuff = set(stuff)
            if len(stuff) <= self.m_samples:
                return stuff
            return random.sample(stuff, self.m_samples)
            #return set([item for item in diff if  item[0] < self.img_width and item[0] >= 0 and item[1] < self.img_height and item[1] >= 0 and item not in self.V and np.array_equal(self.img[item[1]][item[0]], [0,0,0]) == False])
           # return set(filter(self.conditional,image_ellipse.ret_list())).add(self.goal)

        else:
          #  image_ellipse = ELLIPSE(self.img_height/2, self.img_width/2, int(math.pi*self.img_height/2*self.img_width/2), (self.img_height/2, self.img_width/2), (self.img_height, self.img_width/2))
          #  diff = image_ellipse.ret_list()
          #  return {item for item in diff if  item[0] < self.img_width and item[0] >= 0 and item[1] < self.img_height and item[1] >= 0 and item not in self.V and np.array_equal(self.img[item[1]][item[0]], [0,0,0]) == False}

            image_ellipse = ELLIPSE(self.img_height/2, self.img_width/2, int(math.pi*self.img_height/2*self.img_width/2), (self.img_height/2, self.img_width/2), (self.img_height, self.img_width/2))
            stuff = image_ellipse.ret_list()
            stuff = {item for item in stuff if item[0] < self.img_width and item[0] >= 0 and item[1] < self.img_height and item[1] >= 0 and item not in self.V and item not in self.X_reuse and np.array_equal(self.img[item[1]][item[0]], [0,0,0]) == False}
            stuff = set(stuff)
            if len(stuff) <= self.m_samples:
                return stuff
            return random.sample(stuff, self.m_samples)
        #should have a batch of samples and a queue. 


    def Prune(self):
        X_reuse1 = set()
        self.X_unconn = {x for x in self.X_unconn if self.V[x].f_val < self.c_i}
        life = sorted([(self.V[k].cost,k) for k,y in self.V.items()])
        for l in life:
            if self.V[l[1]].f_val > self.c_i or (self.V[l[1]].cost+self.euclidean_distance_grid(l[1],self.goal) > self.c_i or self.V[l[1]].cost == math.inf) and l[1] != self.goal:
                #self.V.discard(l[1])
                if l[1] in self.V_soln:
                    self.V_soln.remove(l[1])
                if l[1] in self.V_unexpnd:
                    self.V_unexpnd.remove(l[1])

                if self.V[l[1]].f_val < self.c_i:
                    X_reuse1.add(l[1])
                del self.V[l[1]]


        return X_reuse1


    def ExpandVertex(self,old_keys):
        vmin = heapq.heappop(self.QV)
        X_near = None
        if vmin[1] in self.V_unexpnd:
            X_near = self.k_nearest_neighbors(vmin[1],self.X_unconn)
        else:
            X_near = self.k_nearest_neighbors(vmin[1],self.X_new.intersection(self.X_unconn))

        for x in X_near:
            if (bubble:=self.V[x].optimal_cost+self.euclidean_distance_grid(vmin[1],x)+self.euclidean_distance_grid(x,self.goal)) < self.c_i:
                heapq.heappush(self.QE, (bubble , (vmin[1],x)))
        
        if vmin[1] in self.V_unexpnd:
            V_near = self.k_nearest_neighbors(vmin[1],old_keys)
            for x in V_near:
                if self.V[x].parent != vmin[1] and self.V[vmin[1]].optimal_cost+self.euclidean_distance_grid(vmin[1],x) < self.V[x].cost and (bubble:=self.V[x].optimal_cost+self.euclidean_distance_grid(vmin[1],x)+self.euclidean_distance_grid(x,self.goal)) < self.c_i:
                    heapq.heappush(self.QE, (bubble , (vmin[1],x)))
        
            self.V_unexpnd.discard(vmin[1])



    def processBatch(self,old_key):
        while len(self.QV) != 0 or len(self.QE) != 0:
            while self.bestQValue(self.QV) <= self.bestQValue(self.QE) and len(self.QV) != 0: 
                self.ExpandVertex(old_key)

            if self.QE:
                vmin, xmin = heapq.heappop(self.QE)[1]

            else:
                vmin = xmin = None

            if vmin and xmin and self.V[vmin].cost+self.euclidean_distance_grid(vmin,xmin)+self.euclidean_distance_grid(xmin,self.goal) < self.c_i:
                if self.V[vmin].cost + (waited:=self.euclidean_distance_grid(vmin,xmin)) < self.costGetter(xmin):
                    c_edge = waited
                    if self.V[vmin].cost+c_edge+self.euclidean_distance_grid(xmin,self.goal) < self.c_i:
                        if self.V[vmin].cost+c_edge < self.V[xmin].cost:
                            if xmin not in self.V:
                                self.V[xmin] = self.Node()
                            if xmin in self.V and self.V[xmin].cost == math.inf:
                                if xmin in self.X_unconn:
                                    self.X_unconn.remove(xmin)
                                heapq.heappush(self.QV, (self.V[vmin].cost+self.euclidean_distance_grid(vmin,xmin), xmin))
                                self.V_unexpnd.add(xmin)
                                if self.InGoalRegion(xmin):
                                    self.V_soln.add(xmin)

                            self.V[xmin].cost,self.V[xmin].parent = self.V[vmin].cost+self.euclidean_distance_grid(vmin,xmin), vmin
                            for a in self.V_soln:
                                if (hunna:=self.V[a].cost+self.euclidean_distance_grid(a,self.goal)) < self.c_i:
                                    self.c_i = self.V[self.goal].cost = hunna
                                    self.V[self.goal].parent = a
        return self.c_i


    def run(self):
       previous_solution_cost = self.costGetter(self.goal)
       if np.array_equal(self.img[self.start[1]][self.start[0]], [0,0,0]) or np.array_equal(self.img[self.goal[1]][self.goal[0]],[0,0,0]):
           print('invalid coordinates')
           return
       print("Beginning BIT* search from", self.start, "to",self.goal)
       previous_solution_cost = math.inf
       old_keys = None

       while True:
           self.nodes_expanded += 1
           if len(self.QE) == 0 and len(self.QV) == 0:
               self.X_reuse = self.Prune()
               X_sampling = set(self.Sample())
               self.X_new = X_sampling.union(self.X_reuse)
               self.radius = self.get_radius(self.V, self.X_new)
               self.X_unconn = self.X_unconn.union(self.X_new)
               old_keys = V_old = set(self.V.keys())
               for v in V_old:
                   heapq.heappush(self.QV, (self.V[v].cost+self.euclidean_distance_grid(v,self.goal) , v))

               for a in self.X_unconn:
                    if a not in self.V:
                        self.V[a] = self.Node()
                        self.V[a].optimal_cost = self.euclidean_distance_grid(self.start, a)
                        self.V[a].f_val = self.euclidean_distance_grid(a,self.goal)+self.V[a].optimal_cost


               if (breaker:=self.processBatch(old_keys)) == self.minimum_cost_distance:
                   print("Best solution cost distance found.")
                   break

               elif breaker < previous_solution_cost:
                   print('New solution found. New cost is:', self.c_i)
                   previous_solution_cost = self.c_i

               elif self.c_i == math.inf:
                   print('No solution found. Restarting search.')

               
               else:
                   print('Best solution found:', self.c_i)
                   break



       cur_node = self.goal
       path = []
       while cur_node is not None:
           path.append(cur_node)
           cur_node = self.V[cur_node].parent
        
        
       return path















           #batch_Cost = self.processBatch(old_keys)
          # if previous_solution_cost <= batch_Cost and previous_solution_cost != math.inf:
           #    print('Solution was found. Stopping search because the old solution has the same or better cost than the solution found in the batch.')
           #    break

         #  elif batch_Cost == math.inf:
          #     print('Restarting batch search. Initial solution was not found.')

         #  elif batch_Cost < previous_solution_cost:
          #      previous_solution_cost = batch_Cost
           #     print('Batch produced a better solution than the last. The new solution cost is:', batch_Cost)
#
  






