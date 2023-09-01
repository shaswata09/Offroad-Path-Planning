import math
import queue
import time
import numpy as np
import cv2

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

class AStar:
    def __init__(self,startPos,endPos,image,imageWindowName=None, predictionMatrix=None):
        #startPos and endPos are tuples
        #image is the numpy array from cv2
        #imageWindowName is the name of the image to update for the visual results
        self.predictionMatrix = predictionMatrix
        self.startPos = startPos
        self.endPos = endPos
        self.img = image
        self.imgWidth = len(image[0])
        self.imgHeight = len(image)
        self.nodes_expanded = 0
        self.imgWindow = imageWindowName
        self.closed = set()
        self.nodes_expanded = 0

    # This function returns the euclidean distance between two grid cells 
    def euclidean_distance_grid(self,a,b):
        d = math.hypot(b[0]-a[0],b[1]-a[1])
        return d

    def determineCost(self, state, occluded=False):
        if occluded:
            if self.predictionMatrix[state[1]][state[0]][1] >= 0.2:
                return 1.0
            else:
                return np.inf

        else:
            if np.array_equal(self.img[state[1]][state[0]], [0,0,0]):
                return np.inf
            else: 
                return 1

    



    def get_neighbors(self,pos,closed):

        neighbors = []

        #Calculate all the possible adjacent pixels
        adj = [(-1,0),(1,0),(0,-1),(0,1),
                (-1,1),(1,1),(-1,-1),(1,-1)]

        for move in adj:
            #(x,y format)
            nextCell = (pos[0]+move[0],pos[1]+move[1])

            #Make sure the pixel is a valid pixel inside the road
            if nextCell[0] >= self.imgWidth or nextCell[0] < 0 or nextCell[1] >= self.imgHeight or nextCell[1] < 0 or nextCell in closed:
                continue

            if np.array_equal(self.img[nextCell[1]][nextCell[0]],[0,0,0]):
                continue

            neighbors.append(nextCell)

        return neighbors

    def run(self):
        #Initializes and begins the a_star algorithm

        start_node = SearchNode(self.startPos,None,0)
        start_node.h = self.euclidean_distance_grid(self.startPos,self.endPos) 

        fringe = queue.PriorityQueue()

        priority = start_node.cost + start_node.h
        fringe.put((priority,start_node))


        goal_node = self.a_star(fringe)

        #Extract path from the goal_node and return it    
        path = []

        if goal_node is not None:
            print('Found a solution!')
            cur_node = goal_node
            while cur_node is not None:
                path.append(cur_node.state)
                cur_node = cur_node.parent
            path.reverse()

        else:
            '''
            min_cost_dist = np.inf
            cur_node = start_node

            for a in self.closed:
                if (cost_dist:=self.euclidean_distance_grid(a.state, self.endPos)) < min_cost_dist:
                    cur_node = a
                    min_cost_dist = cost_dist
            while cur_node is not None:
                path.append(cur_node.state)
                cur_node = cur_node.parent
            path.reverse()


            return path
            '''
            return None

        return path
    
    def a_star(self,fringe):
        #The A Star algorithm logic 
        #Fringe is the set of nodes that are open and can be evaluated
        closed = set() #The set of nodes already evaluated
        expansions = 0 #This keep track of the number of expansions
        self.closed = []
        while not fringe.empty():
            #Get the node out of the fringe. Format of fringe is: (priority,searchNode)
            expand_node = fringe.get()[1] # This gets the node to expand
            
            # Make sure we didn't already expand a node pointing to this state
            if expand_node.state not in closed:
                self.nodes_expanded += 1

                if expand_node.state == self.endPos:
                    #Found it!
                    print('Found the goal after ', self.nodes_expanded, ' node expansions')
                    return expand_node

                #Display the expanded node visually
             #   self.img[expand_node.state[1]][expand_node.state[0]] = [0,255,0]

                closed.add(expand_node.state)
                self.closed.append(expand_node)
                neighbors = self.get_neighbors(expand_node.state,closed)

                #For each neighbor (a grid cell tuple (gx,gy))
                for g in neighbors:

                    dist = self.euclidean_distance_grid(expand_node.state,g)
                    
                    newNode = SearchNode(g,expand_node,expand_node.cost + dist )
                    newNode.h = self.euclidean_distance_grid(g,self.endPos) +  (2 * max(abs(g[0]-self.endPos[0]) , abs(g[1]-self.endPos[1])))
                    
                    priority = newNode.cost + newNode.h
                    fringe.put((priority,newNode))

                
                #Update the visual image as we expand nodes
           #     if expansions % 100 == 0:
           #     	cv2.imshow(self.imgWindow,self.img)
           #     	cv2.waitKey(1)

