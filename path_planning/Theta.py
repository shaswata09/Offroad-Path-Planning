import math
import queue
import time
import numpy as np
import cv2
import heapq
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
        self.nodes_expanded = 0

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
        closed = []
        while fringe:
            s=heapq.heappop(fringe)
            if s[1] == self.endPos:
                print('goal was found.')
                break
            self.nodes_expanded += 1
            closed.append(s[1])
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
            print('Found a solution!')
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




