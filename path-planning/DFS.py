import numpy as np

######################################
#                                    #
#  NOT RECOMMENDED FOR PATH PLANNING #
#                                    #
######################################

class DFS:

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

        self.visited = set()

    def run(self,pos,currResult):
        #Return a list of tuples of the path taken
        self.visited.add(pos)

        if pos == self.endPos:
            return currResult + [pos]

        #Attempt trying all of the adjacent pixels to see if there is a path that works
        adj = [(-1,0),(1,0),(0,-1),(0,1),
                (-1,1),(1,1),(-1,-1),(1,-1)]
        
        for move in adj:
            
            nextCell = (pos[0]+move[0],pos[1]+move[1])

            if nextCell[0] >= self.imgWidth or nextCell[0] < 0 or nextCell[1] >= self.imgHeight or nextCell[1] < 0:
                continue
            if nextCell in self.visited:
                continue
            if np.array_equal(self.img[nextCell[1]][nextCell[0]],[0,0,0]):
                continue
            
            res = self.run(nextCell,currResult + [pos])
            if res is not None:
                return res
        
        return None

    def runIterative(self):

        stack = []
        stack.append((self.startPos,[]))
        

        while len(stack) > 0:

            nextItem = stack.pop(-1)
            currPath = nextItem[1]
            currPos = nextItem[0]

            if currPos == self.endPos:
                return currPath + [currPos]      

            self.visited.add(self.startPos)

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

                stack.append((nextCell,currPath + [currPos]))

