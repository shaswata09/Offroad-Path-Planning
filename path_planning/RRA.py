import cv2 
import os
import heapq 
import numpy as np
import random
import math

class Node:
    def __init__(self, state, cost=-1):
        self.state = state 
        self.cost = cost
        self.predecessor = None


class RRA:
    def __init__(self, start, goal, prediction_image, segmentation_image):
        self.goal = goal
        self.start = start
        self.img_width = len(prediction_image)
        self.img_height = len(prediction_image[0])
        self.segmentatedImage = np.zeros([self.img_width,self.img_height,3],dtype=np.uint8)
        self.segmentatedImage.fill(255) 
     #   self.segmentatedImage = prediction_image

        self.GroundTruthImage = segmentation_image
        self.closed = set()
        self.open = []
        self.open_set_check = set()
        self.path = []
        self.searchTree = {}
        self.current_location = self.start
        self.img_height = len(prediction_image)
        self.img_width = len(prediction_image[0])
        for a in range(self.img_height):
            for b in range(self.img_width):
                self.searchTree[(b, a)] = Node((b,a))
        self.searchTree[goal].cost = 0


        print('Starting RRA search to from', self.start, ' to', self.goal)


    def Initialize(self):
        self.searchTree[self.goal].cost = 0
        heapq.heappush(self.open, (self.eudis5(self.goal, self.current_location), self.goal))

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
        if start[0] < 0 or start[1] < 0 or start[1] >= self.img_height or start[0] >= self.img_width:
            return False, None
					#known obstacle
        if np.array_equal(self.segmentatedImage[y0][x0], [255,0,0]):
            self.searchTree[(x0,y0)].cost = math.inf
            return False, None

					#predicted obstacle that was correct

        elif np.array_equal(self.segmentatedImage[y0][x0], self.GroundTruthImage[y0][x0]) and np.array_equal(self.GroundTruthImage[y0][x0], [0,0,0]):
            self.segmentatedImage[y0][x0] = [255,0,0]
            self.searchTree[(x0,y0)].cost = math.inf
            return False, None

					#predicted obstacle that was free space

					#predicted free space that was an obstacle
        elif np.array_equal(self.segmentatedImage[y0][x0], [255, 255, 255]) and np.array_equal(self.GroundTruthImage[y0][x0], [0, 0, 0]):
            self.segmentatedImage[y0][x0] = [255,0,0]
            self.searchTree[(x0,y0)].cost = math.inf


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

                if np.array_equal(self.segmentatedImage[y0][x0], [255,0,0]):

                    self.searchTree[(x0,y0)].cost = math.inf

                    return False, None

					        #predicted obstacle that was correct

                elif np.array_equal(self.segmentatedImage[y0][x0], self.GroundTruthImage[y0][x0]) and np.array_equal(self.GroundTruthImage[y0][x0], [0,0,0]):
                    self.segmentatedImage[y0][x0] = [255,0,0]
                    self.searchTree[(x0,y0)].cost = math.inf

                    return False, None

					        #predicted obstacle that was free space

					        #predicted free space that was an obstacle
                elif np.array_equal(self.segmentatedImage[y0][x0], [255, 255, 255]) and np.array_equal(self.GroundTruthImage[y0][x0], [0, 0, 0]):
                    self.segmentatedImage[y0][x0] = [255,0,0]
                    self.searchTree[(x0,y0)].cost = math.inf
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
                if np.array_equal(self.segmentatedImage[y0][x0], [255,0,0]):
                    self.searchTree[(x0,y0)].cost = math.inf

                    return False, None

					        #predicted obstacle that was correct

                elif np.array_equal(self.segmentatedImage[y0][x0], self.GroundTruthImage[y0][x0]) and np.array_equal(self.GroundTruthImage[y0][x0], [0,0,0]):
                    self.segmentatedImage[y0][x0] = [255,0,0]
                    self.searchTree[(x0,y0)].cost = math.inf

                    return False, None

					        #predicted obstacle that was free space

					        #predicted free space that was an obstacle
                elif np.array_equal(self.segmentatedImage[y0][x0], [255, 255, 255]) and np.array_equal(self.GroundTruthImage[y0][x0], [0, 0, 0]):
                    self.segmentatedImage[y0][x0] = [255,0,0]
                    self.searchTree[(x0,y0)].cost = math.inf
                    return False, None

        return True, end

    def eudis5(self, v1, v2):
        dist = [(a - b)**2 for a, b in zip(v1, v2)]
        return np.sqrt(sum(dist))

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



    def get_neighbors(self,pos, get_closed=False):
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

    def get_diagonals(self, state):
        diagonals = [tuple((state[0]+1, state[1]+1)), tuple((state[0]-1, state[1]+1)), tuple((state[0]+1, state[1]-1)), tuple((state[0]-1, state[1]-1))]
        diagonals = [x for x in diagonals if x[0] >=0 and x[0] < self.img_height and x[1]>=0 and x[1] < self.img_width]		
        return diagonals

    
    def linkCostGrabber(self, state1, state2):
        if np.array_equal(self.segmentatedImage[state2[1]][state2[0]], [255,0,0]) == True or np.array_equal(self.segmentatedImage[state1[1]][state1[0]], [255,0,0]) == True:
            return math.inf

        return 1

    def ComputePath(self):
        self.searchTree[self.goal].cost = 0
        self.searchTree[self.goal].predecessor = None
        heapq.heappush(self.open, (self.eudis5(self.goal, self.current_location), self.goal))
        while True:
            while len(self.open) > 0:
                top = heapq.heappop(self.open)
                self.closed.add(top[1])
                self.open_set_check.discard(top[1])
                if top[0] == math.inf:
                    print('no path was found.')
                    return False
                if top[1] == self.current_location:
                    break

                neighbors = self.get_neighbors(top[1])
                for n in neighbors:
                    if n == self.searchTree[top[1]].predecessor:
                        continue
                    g_cost = self.linkCostGrabber(n, top[1])+self.searchTree[top[1]].cost
                    f_cost = self.eudis5(self.current_location, n) + g_cost
                    if n in self.open_set_check and (open_check:=self.OpenQueueAndIndexCheck(n))[1] != None:
                        if self.open[open_check[1]][0] > f_cost:
                            self.searchTree[n].predecessor = top[1]
                            self.open[open_check[1]] = (f_cost , n)
                            heapq.heapify(self.open)

                    elif n not in self.open_set_check:
                        self.searchTree[n].cost = g_cost
                        self.searchTree[n].predecessor = top[1]
                        heapq.heappush(self.open, (f_cost, n))
                        self.open_set_check.add(n)

            while self.current_location != self.goal:
                self.castRays(self.current_location[0], self.current_location[1])
                self.path.append(self.current_location)
                if np.array_equal(self.segmentatedImage[self.searchTree[self.current_location].predecessor[1]][self.searchTree[self.current_location].predecessor[0]], [255, 0,0]) == False:
             #   if np.array_equal(self.predImage[self.searchTree[self.current_location].predecessor[1]][self.searchTree[self.current_location].predecessor[0]], [255, 255,255]) == True:
                    self.current_location = self.searchTree[self.current_location].predecessor
                else:
                    break

                if self.current_location == self.goal:
                    print('Goal has been traversed to.')
                    return self.path
            self.closed.discard(self.searchTree[self.current_location].predecessor)
            TEMP = [self.searchTree[self.current_location].predecessor]
            while TEMP: 
                t_top = TEMP.pop(random.randrange(len(TEMP)))
                neighbors = self.get_neighbors(t_top, True)
                for n in neighbors:
                    if self.searchTree[n].predecessor == t_top and n in self.open_set_check:
                        self.open_set_check.discard(n)
                        self.open.pop(self.OpenQueueAndIndexCheck(n)[1])
                        TEMP.append(n)

                    elif self.searchTree[n].predecessor == t_top and n in self.closed:
                        self.closed.discard(n)
                        TEMP.append(n)
                    
                    elif self.searchTree[n].predecessor != t_top and n in self.closed:
                        self.closed.discard(n)
                        self.open_set_check.add(n)
                        heapq.heappush(self.open, (self.searchTree[n].cost+self.eudis5(n, self.current_location), n))

                if len(self.open) > 0:
                    for a in range(len(self.open)):
                        self.open[a] = (self.eudis5(self.current_location, self.open[a][1])+self.searchTree[self.open[a][1]].cost , self.open[a][1])
                    heapq.heapify(self.open)
                else:
                    self.searchTree[self.goal].predecessor = None
                    self.searchTree[self.goal].cost = 0



