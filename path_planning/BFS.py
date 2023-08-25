import numpy as np
import cv2

class BFS:

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
        self.n_iterations = 0

        self.visited = set()

    def run(self):
        stack = []
        stack.append((self.startPos,[]))
        self.visited.add(self.startPos)

        while len(stack) > 0:
            self.n_iterations += 1

            nextItem = stack.pop(0)
            currPath = nextItem[1]
            currPos = nextItem[0]

            if currPos == self.endPos:
                print('Found the goal after ',expansions, ' node expansions')
                return currPath + [currPos]      



            #Attempt trying all of the adjacent pixels to see if there is a path that works
            adj = [(-1,0),(1,0),(0,-1),(0,1),
                    (-1,1),(1,1),(-1,-1),(1,-1)]
            
            for move in adj:
                
                nextCell = (currPos[0]+move[0],currPos[1]+move[1])

                if nextCell[0] >= self.imgWidth or nextCell[0] < 0 or nextCell[1] >= self.imgHeight or nextCell[1] < 0:
                    continue
                if nextCell in self.visited:
                    continue
                if np.array_equal(self.img[nextCell[1]][nextCell[0]],[0,0,0]):
                    continue
                
                self.visited.add(nextCell)
                stack.append((nextCell,currPath + [currPos]))

            self.img[currPos[1]][currPos[0]] = [0,255,0]
#            if expansions % 100 == 0:
#                cv2.imshow(self.imgWindow,self.img)
#                cv2.waitKey(1)

