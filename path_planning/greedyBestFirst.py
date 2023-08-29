import numpy as np
import cv2
import heapq
import math


class greedyBestFirst:

    def __init__(self,startPos,endPos,image,imageWindowName):

        #startPos and endPos are tuples
        #image is the numpy array from cv2
        #imageWindowName is the name of the image to update for the visual results

        self.startPos = startPos
        self.endPos = endPos
        self.img = image
        self.imgWidth = len(image[0])
        self.imgHeight = len(image)
        self.imgWindow = imageWindowName
        self.nodes_expanded = 0

    def euclidean_distance_grid(self,a,b):
        d = math.hypot(b[0]-a[0],b[1]-a[1])
        return d

    def run(self):
        visited = set()
        visited.add(self.startPos)
        queue = []
        startDist = self.euclidean_distance_grid(self.startPos,self.endPos)
        heapq.heappush(queue,(startDist,self.startPos,[self.startPos]))

        while len(queue) > 0:

            #Get the node with min distance to target
            currNode = heapq.heappop(queue)
            currPos = currNode[1]
            currPath = currNode[2]

            self.nodes_expanded += 1
            self.img[currPos[1]][currPos[0]] = [0,255,0]
            #Loop through each neighbor of the current node
            adj = [(-1,0),(1,0),(0,-1),(0,1),
                    (-1,1),(1,1),(-1,-1),(1,-1)]

            for move in adj:
                nextCell = (currPos[0]+move[0],currPos[1]+move[1])

                if nextCell not in visited and not (nextCell[0] >= self.imgWidth or nextCell[0] < 0 or nextCell[1] >= self.imgHeight or nextCell[1] < 0):
                    if np.array_equal(self.img[nextCell[1]][nextCell[0]],[0,0,0]):
                        continue

                    if nextCell == self.endPos:
                        print('Found the goal after ', expansions, ' node expansions')
                        return currPath + [nextCell]
                    else:
                        visited.add(nextCell)
                        newDist = self.euclidean_distance_grid(nextCell,self.endPos)
                        heapq.heappush(queue,(newDist,nextCell,currPath + [nextCell]))
            
      #      if expansions % 10 == 0:
      #          cv2.imshow(self.imgWindow,self.img)
      #          cv2.waitKey(1)

        return None #Failure, did not find a path
