import cv2
import numpy as np
import os
import math
import heapq
import random
import time


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


class ARA:
    def __init__(self, start, goal, predictedImage, predMatrix, model_probability_accuracy, groundTruthImage=None):
        self.start = start
        self.goal = goal
        #self.model_probability_accuracy = model_probability_accuracy
        #Future work
        print("Beginning ARA* search from", self.start, "to", self.goal)
        self.predictedImage = predictedImage
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
        self.replan_limit = 10
        self.FORCE_SAMPLE_TRAVERSAL = True
        self.ep_val = 2.3
        self.randarray = np.linspace(0, 2 * np.pi, 500)
        self.sample_heuristic = 0.0
        self.open_set_check = set()
        self.sample_point = None
        self.observable_distance = 30
        self.global_queue = set()
        self.interrupt_limit = 5
        self.move_queue = []
        self.open = []
        self.predictionMatrix_heuristic = 35
        self.cost_heuristic = 10
        self.INCONS = set()
        self.current_location = self.start
        self.s, self.g = np.asarray(self.start), np.asarray(self.goal)


        self.path = []
        for a in range(self.img_width):
            for b in range(self.img_height):
                self.Tree[(b, a)] = Node(hval=0.3*self.distanceToLine(np.array([b, a])))

        self.Tree[self.start] = Node(cost=0)
        self.Tree[self.start].hval = self.eudis5(self.start, self.goal)
        self.Tree[self.current_location].rhs = 0
        self.Tree[self.goal].hval = 0
        

    def fvalue(self, s):
       # return self.Tree[s].cost + self.ep_val * self.Tree[s].hval * (self.predictionMatrix[s[1]][s[0]][0] * 35)
       return (self.Tree[s].cost * self.cost_heuristic) + self.ep_val * self.Tree[s].hval * (self.predictionMatrix[s[1]][s[0]][0] * self.predictionMatrix_heuristic)

    def checkToAddToGlobalQueue(self, point1):
        if self.Tree[point1].TRAVERSED:
            self.global_queue.discard(point1)
            return None
        for a in self.global_queue:
            if self.eudis5(a, point1) < self.observable_distance:
                return None
        self.global_queue.add(point1)
        return point1

    def distanceToLine(self, p):
        #return np.abs(np.cross(self.g-self.s,p-self.s)/np.linalg.norm(self.g-self.s))
        return np.cross(self.g-self.s,p-self.s)/np.linalg.norm(self.g-self.s)

# line cast. 



    def replanLinkCostGetter(self, state):
        if self.predictionMatrix[state[1]][state[0]][0] == 1.0:
            return 1
        elif self.predictionMatrix[state[1]][state[0]][0] == 0.0:
            return np.inf
        return self.UnpredictableCostGetter(state)


    def getreSearchedCost(self, state):
        if self.Tree[state].reSearched == False:
            self.Tree[state].reSearched = True
            return np.inf
        return self.Tree[state].cost

    def replanFvalue(self, s):
        return (self.getreSearchedCost(s) * self.cost_heuristic) + self.ep_val * self.Tree[s].hval * (self.predictionMatrix[s[1]][s[0]][0] * self.predictionMatrix_heuristic)





    def bresenham(self, start, end):
        (x0,y0) = start
        (x1,y1) = end

        if start[0] < 0 or start[1] < 0 or start[1] >= self.img_height or start[0] >= self.img_width or np.array_equal(self.groundTruthImage[start[1]][start[0]], [0,0,0]):
            self.predictedImage[start[1]][start[0]] = self.groundTruthImage[start[1]][start[0]]
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

                self.predictedImage[y0][x0] = self.groundTruthImage[y0][x0]

                if np.array_equal(self.groundTruthImage[y0][x0], [0,0,0]):
                    self.predictionMatrix[y0][x0] = np.array([1.0, 0.0])
                    self.Tree[(x0,y0)].cost = self.Tree[(x0,y0)].real_cost = np.inf
                    self.Tree[(x0,y0)].seen = -1
                    return (False, (x0,y0))
                
                self.predictionMatrix[y0][x0] = np.array([0.0, 1.0])
                line.append((x0,y0))
                self.Tree[(x0,y0)].seen = 0
                self.checkToAddToGlobalQueue((x0,y0))

                    

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

                self.predictedImage[y0][x0] = self.groundTruthImage[y0][x0]
                if np.array_equal(self.groundTruthImage[y0][x0], [0,0,0]):
                    self.predictionMatrix[y0][x0] = np.array([1.0, 0.0]) 
                    self.Tree[(x0,y0)].cost = self.Tree[(x0,y0)].real_cost = np.inf
                    self.Tree[(x0,y0)].seen = -1

                    return (False, (x0, y0))

                self.predictionMatrix[y0][x0] = np.array([0.0, 1.0])
                line.append((x0,y0))
                self.Tree[(x0,y0)].seen = 0
                self.checkToAddToGlobalQueue((x0,y0))

        return True, end


    def castRays(self, x,y):
        cast_rays_list = []
        for xr in self.randarray:
            x1 = int(x + self.observable_distance * np.cos(xr))
            y1 = int(y + self.observable_distance * np.sin(xr))
            cast_rays_list.append(self.bresenham((x,y), (x1,y1))[1])
            
        return cast_rays_list


    def UnpredictableCostGetter(self, state):
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
        elif self.predictionMatrix[state[1]][state[0]][1] < 0.04 and self.predictionMatrix[state[1]][state[0]][1] > 0.0:
                return 14.5
        else: 
            return np.inf


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

    def rebuildPath(self, node_in_path):
        cur_Node = node_in_path
        tmp_path = []
        while cur_Node != None:
            tmp_path.append(cur_Node)
            if cur_Node == self.current_location:
                break
            cur_Node = self.Tree[cur_Node].predecessor
        tmp_path.reverse()
        return tmp_path



    def immediateNeighborHeuristic(self, point):
        neighbors = self.get_neighbors(point)
        len_var = 0
        points = 0.0
        for a in neighbors:
            if self.predictionMatrix[a[1]][a[0]][0] > 0.6:
                points += 4.5
            else:
                points += 1.2
            len_var += 1

        return points / len_var

    def determineSamplePoint(self):
        min_heuristic = np.inf
        sample_point = random.sample(list(self.global_queue), 1)[0]
        for point in self.global_queue:
            if (heuristic_hold:=1.8*self.eudis5(point,self.goal) + 0.8 * self.eudis5(self.current_location, point) + 0.4 * self.immediateNeighborHeuristic(point) + 0.1 * self.Tree[point].cost) < min_heuristic:
                 sample_point = point
                 min_heuristic = heuristic_hold

        return sample_point, min_heuristic



    def ImprovePath(self):
        while self.fvalue(self.goal) > (self.open[0])[0]:
            top = heapq.heappop(self.open)
            self.closed.add(top[1])
            top_neighbors = self.get_neighbors(top[1])
            for a in top_neighbors: 
                if a == self.Tree[top[1]].predecessor:
                    continue

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
        for open_node in self.open:
            incons.append(open_node[1])
        minimum = np.inf
        for a in incons:
            if (tg:=self.Tree[a].cost + self.Tree[a].hval) < minimum:
                minimum = tg

        return minimum



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


    def path_los(self, start, end):  
        #observable range
        if self.eudis5(start, end) >= self.observable_distance:
            return False, None
        (x0,y0) = start
        (x1,y1) = end
        
    
        if start[0] < 0 or start[1] < 0 or start[1] >= self.img_height or start[0] >= self.img_width or np.array_equal(self.predictedImage[start[1]][start[0]], [0,0,0]) or start == end:
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

                    return False, None

                if self.Tree[(x0,y0)].seen == -1:

                    return False, None
              
        


                if np.array_equal(self.predictedImage[y0][x0], [0,0,0]) :
                    return False, None
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
                    return False, None

                if self.Tree[(x0,y0)].seen == -1:
                    return False, None

                if np.array_equal(self.predictedImage[y0][x0], [0,0,0]):
                    return False, None
                if np.array_equal(self.groundTruthImage[y0][x0], [0,0,0]):
                    print('something is wrong.')
                line.append((x0,y0))
        return True, line

    def replanImprovePath(self):
        while self.open and self.fvalue(self.goal) > (self.open[0])[0]:
            top = heapq.heappop(self.open)
            self.closed.add(top[1])
            top_neighbors = self.get_neighbors(top[1], False)
            for a in top_neighbors: 
                if a == self.Tree[top[1]].predecessor:
                    continue

                if self.Tree[a].cost > (tmp_cost:=self.Tree[top[1]].cost+self.replanLinkCostGetter(a)):
                    self.Tree[a].cost = tmp_cost
                    self.Tree[a].predecessor = top[1]
                    if a not in self.closed:
                        heapq.heappush(self.open, (self.fvalue(a), a))




    def replan_static_path_main(self):
        self.closed = set()
        self.open = []
        for k, v in self.Tree.items():
            self.Tree[k].predecessor = None
            self.Tree[k].cost = np.inf
        self.Tree[self.current_location].cost = 0
        if self.ep_val < 1.0:
            self.ep_val = round(random.uniform(1.1,7), 1)
        heapq.heappush(self.open, (self.fvalue(self.current_location), self.current_location))

        self.replanImprovePath()
        self.path = self.rebuildPath(self.goal)
        self.ep_val -= 0.4
        return self.path
        

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

            self.predictionMatrix_heuristic -= 8
            self.cost_heuristic -= 2
            heapq.heapify(self.open)
            self.closed = set()
            print(optimal_e)
            self.ImprovePath()
            self.path = self.buildPath()
           # yield self.path
            optimal_e = min(self.ep_val, self.Tree[self.goal].cost / self.get_minimum_e_val())
            print(self.getPathCost())

        self.path = self.buildPath()


        if self.get_First_Path_metric:
            ratio = 0
            for a in self.path:
                if np.array_equal(self.groundTruthImage[a[1]][a[0]], [255, 255, 255]):
                    ratio += 1

            print("ARA* initial path has an accuracy of: ", (ratio/len(self.path)) * 100, "%")
            self.get_First_Path_metric = False

        self.predictionMatrix_heuristic = 35
        self.cost_heuristic = 10

        return self.path





    def traverseLineInPath(self, new_location):

        start = self.current_location
        (x0,y0) = self.current_location
        (x1,y1) = new_location
        
        
        if start[0] < 0 or start[1] < 0 or start[1] >= self.img_height or start[0] >= self.img_width or np.array_equal(self.predictedImage[start[1]][start[0]], [0,0,0]):
            return False
        line = []
        xi = yi = D = None
        line.append(start)
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


                if x0 >= self.img_width or x0 < 0 or y0 >= self.img_height or y0 < 0 or np.array_equal(self.predictedImage[y0][x0], [0,0,0]) or self.Tree[(x0, y0)].seen == -1:
                    self.current_location = line[-1]
                    return False
                line.append((x0,y0))

                new_list = self.castRays(x0, y0)
                self.Tree[(x0,y0)].TRAVERSED = True

                if self.Tree[self.goal].seen != -1 and self.alreadyTraversing == False:
                    self.current_location = ((x0, y0))
                    self.navigateToSeenGoal()
                    return 

               # if self.Tree[self.goal].seen != -1 and self.Tree[self.goal].cost != np.inf and self.alreadyTraversing == False:
                #    self.navigateToSeenGoal()
               #     return 
                self.checkToAddToGlobalQueue((x0,y0))
                self.move_queue.append((x0,y0))

                


                    

        else:
            D = 2*dX - dY
            while (y0 != y1):

            
                if (D > 0):
            
                    x0 += xi
                    D -= 2*dY
            
                D +=2*dX
                y0+=yi


                if x0 >= self.img_width or x0 < 0 or y0 >= self.img_height or y0 < 0 or np.array_equal(self.predictedImage[y0][x0], [0,0,0]) or self.Tree[(x0, y0)].seen == -1:
                    self.current_location = line[-1]
                    return False
                #self.castRays(x0, y0)

                new_list = self.castRays(x0, y0)
                line.append((x0,y0))
                self.Tree[(x0,y0)].TRAVERSED = True
              #  if self.Tree[self.goal].seen != -1 and self.Tree[self.goal].cost != np.inf and self.alreadyTraversing == False:
              #      self.navigateToSeenGoal()
              #      return 



                if self.Tree[self.goal].seen != -1 and self.alreadyTraversing == False:
                    self.current_location = ((x0, y0))
                    self.navigateToSeenGoal()
                    return 
                self.checkToAddToGlobalQueue((x0,y0))
                self.move_queue.append((x0,y0))

                    
        self.current_location = new_location
        return True, None


    def navigateToSeenGoal(self):
        print('navigating')
        keys = [k for k,v in self.Tree.items() if (v.seen != -1)]
        pred_Image = self.predictedImage.copy()
        for k in keys:
            pred_Image[k[1]][k[0]] = np.array([255, 255, 0])
        print(self.current_location, self.goal)
        path = Theta(self.current_location, self.goal, pred_Image, None).run()
        for p in range(len(path)):
            self.current_location = path[p]
            self.move_queue.append(self.current_location)
            self.castRays(self.current_location[0], self.current_location[1])
        print('exiting.')


    def Traversal(self):
        self.static_path_main()
        while self.current_location != self.goal and self.Tree[self.goal].TRAVERSED == False:
            self.castRays(self.current_location[0], self.current_location[1])

            los_paths = [x for x in self.path if x != self.current_location and self.path_los(self.current_location, x)[0]]
            if self.goal in los_paths:
                print('goal was found.')
                self.alreadyTraversing = True
                self.traverseLineInPath(self.goal)
                ratio = 0
                for a in self.move_queue:
                    if np.array_equal(self.groundTruthImage[a[1]][a[0]], [255, 255, 255]):
                        ratio += 1
                    else:
                        self.predictedImage = cv2.circle(self.predictedImage, a, radius=5, color=(255, 0, 0), thickness = 1)

                print("Move queue path has an accuracy of: ", (ratio/len(self.move_queue)) * 100, "%")
                return self.move_queue

            if los_paths: 
                los_paths = los_paths[-1]
                self.traverseLineInPath(los_paths)
                if self.current_location == self.goal:
                    print('Goal has been traversed to.')
                    return self.move_queue
                self.path = self.path[self.path.index(self.current_location):]

            else:
                # use sampling procedures.
        #        if random.random() > 0.50 and self.current_replan >= self.replan_limit:
        #            print('Im static')
#
 #                   self.replan_static_path_main()
#                    self.replan_limit += 25
   #                 self.current_replan = 0
  #                  print('New path finished.')
    #                assert(self.goal in self.path and self.current_location in self.path)


     #           else:
                    #print('Im here')
                    self.current_replan += 1
                    self.sample_point, self.sample_heuristic = self.determineSamplePoint()
                    if self.sample_point == None:
                        print(len(self.global_queue))
                    keys = [k for k,v in self.Tree.items() if (v.seen != -1 or v.TRAVERSED == True) ]
                    pred_Image = self.predictedImage.copy()
                    for k in keys:
                        pred_Image[k[1]][k[0]] = np.array([255, 255, 0])
                    temp_THETA = Theta(self.current_location, self.sample_point, pred_Image, None)
                    path = temp_THETA.run()
                    for a in range(len(path)):
                        self.traverseLineInPath(path[a])
                        if self.current_location == self.goal:
                            return self.move_queue
                    self.checkToAddToGlobalQueue(self.sample_point)


                #self.static_path_main()
        print('goal found.')
         

        


import tcod

class SearchNode:
	def __init__(self,state,parent,cost):
		self.parent = parent # Pointer to parent node
		self.state = state # The current grid cell this node points to
		self.cost = cost  # The cumulative cost of this search node (sum of all actions to get here)
		self.h = 0 # This node's heuristic value
	
	def __lt__(self,other):
		return self.h < other.h



class Theta:
    def __init__(self,startPos,endPos,image,imageWindowName):
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
            if np.array_equal(self.img[nextCell[1]][nextCell[0]],[255,255,0]) == False:
                continue



            neighbors.append(nextCell)

        return neighbors

    def get_los(self, first_point, second_point):
        line_algo_points = tcod.los.bresenham(first_point, second_point).tolist()
        for a in line_algo_points:
            if np.array_equal(self.img[a[1]][a[0]], [255,255,0]) == False:
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
                openq.append((self.euclidean_distance_grid(sp,self.endPos) +    (2 * max(abs(sp[0]-self.endPos[0]) , abs(sp[1]-self.endPos[1]))), sp))


                heapq.heapify(openq)

        else:

            if self.V[s].cost+self.euclidean_distance_grid(s,sp) < self.V[sp].cost:
                self.V[sp].cost = self.V[s].cost+self.euclidean_distance_grid(s,sp)
                self.V[sp].parent = s
                t = self.isOnOpenQueueAndCostCheck(None, sp, openq)
                if t[0] == True:
                    openq.pop(t[1])
                openq.append((self.euclidean_distance_grid(sp,self.endPos)+ (2 * max(abs(sp[0]-self.endPos[0]) , abs(sp[1]-self.endPos[1]))),sp))
                heapq.heapify(openq)


    def run(self):
        if self.startPos == self.endPos:
            return [self.startPos]
        self.V[self.startPos] = SearchNode(self.startPos, self.startPos, 0)
        fringe = []
        heapq.heappush(fringe, (self.euclidean_distance_grid(self.startPos,self.endPos) + (2 * max(abs(self.startPos[0]-self.endPos[0]) , abs(self.startPos[1]-self.endPos[1]))), self.startPos))
        closed = set()
        while fringe:
            s=heapq.heappop(fringe)
            if s[1] == self.endPos:
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

        return