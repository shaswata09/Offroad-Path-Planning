import cv2
import os
import numpy as np
import math
import heapq
import random

class Node:
	def __init__(self, cost=np.inf, rhs=np.inf, key= (np.inf, np.inf)):
		self.cost = cost
		self.rhs = rhs
		self.successors = set()
		self.predecessors = set()

class DLITESEARCH:
	def __init__(self, start, goal, segmentatedImage, GroundTruthImage):
		self.start = start
		self.goal = goal
		print('Starting search from', self.start, 'to', self.goal)
		self.img_width = len(segmentatedImage)
		self.img_height = len(segmentatedImage[0])
		self.segmentatedImage = segmentatedImage
		self.GroundTruthImage = GroundTruthImage
		self.randarray = np.linspace(0, np.pi, 50)
		self.pps = np.average(self.randarray)-self.randarray[0]
		self.nodeTree = {}
		self.path = []
		self.changedNodeList = []
		self.h_cost = self.eudis5(self.start, self.goal)
		#self.currentLocation = self.start
		self.kM = 0
		self.U = []
		self.open_Set_check = set()
		for a in range(self.img_width):
			for p in range(self.img_height):
				self.nodeTree[(a,p)] = Node()


	def Initialize(self):
		self.nodeTree[self.goal] = Node(cost=0, rhs=0)
		heapq.heappush(self.U, (self.CalculateKey(self.goal), self.goal))
		self.open_Set_check.add(self.goal)

	def eudis5(self, v1, v2):
		dist = [(a - b)**2 for a, b in zip(v1, v2)]
		dist = math.sqrt(sum(dist))
		return dist


	def CalculateKey(self, state):
		if state not in self.nodeTree:
			return (np.inf, np.inf)
		return (min(self.nodeTree[state].cost, self.nodeTree[state].rhs)+self.kM+self.h_cost, min(self.nodeTree[state].cost, self.nodeTree[state].rhs))


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



	def get_neighbors(self,pos):
		neighbors = set()

        #Calculate all the possible adjacent pixels
		adj = [(-1,0),(1,0),(0,-1),(0,1),
                (-1,1),(1,1),(-1,-1),(1,-1)]

		for move in adj:
			nextCell = (pos[0]+move[0],pos[1]+move[1])
			if nextCell[0] >= self.img_width or nextCell[0] < 0 or nextCell[1] >= self.img_height or nextCell[1] < 0:
				continue

			neighbors.add(nextCell)

		return neighbors

# start should only have successors and no predecessors. Goal should only have predecessors and no successors (initially). 

	def getPredecessorList(self, state):
		temp_neighbors = self.get_neighbors(state)
		tt = []
		for a in temp_neighbors:
			if a in self.nodeTree[state].successors:
				continue
			tt.append(a)

		return tt


	def linkCostGrabber(self, state2):
		if np.array_equal(self.segmentatedImage[state2[1]][state2[0]], [0,0,0]):
			return np.inf
		return 1
		

	def UpdateVertex(self, state):
		if state != self.goal:
			min_val = np.inf
			for p in self.nodeTree[state].successors:
				if min_val > (qrs:=self.linkCostGrabber(p)+self.nodeTree[p].cost):
					min_val = qrs
			
			self.nodeTree[state].rhs = min_val


		#if (opp:=self.OpenQueueAndIndexCheck(state))[0]:
		if state in self.open_Set_check:
			opp = self.OpenQueueAndIndexCheck(state)
			self.U.pop(opp[1])
			self.open_Set_check.discard(state)

		if self.nodeTree[state].cost != self.nodeTree[state].rhs:
			#heapq.heappush(self.U, (self.CalculateKey(state), state))
			self.open_Set_check.add(state)
			self.U.append((self.CalculateKey(state), state))

		heapq.heapify(self.U)

	def ComputeShortestPath(self):
		while self.nodeTree[self.start].rhs != self.nodeTree[self.start].cost or self.U[0][0] < self.CalculateKey(self.start):
			head = heapq.heappop(self.U)
			self.open_Set_check.discard(head[1])

			if head[0] < (pl:=self.CalculateKey(head[1])):
				heapq.heappush(self.U, (pl, head[1]))

			elif self.nodeTree[head[1]].cost > self.nodeTree[head[1]].rhs:
				self.nodeTree[head[1]].cost = self.nodeTree[head[1]].rhs
				top = self.getPredecessorList(head[1])
				for lp in top:
					self.nodeTree[lp].successors.add(head[1])
					self.nodeTree[head[1]].predecessors.add(lp)
					self.UpdateVertex(lp)

			else:
				self.nodeTree[head[1]].cost = np.inf
				top = self.getPredecessorList(head[1])
				top.append(head[1])
				for p in top:
					self.UpdateVertex(p)






	def castRays(self, x,y):
		final_dist = 100
		angle_gen=0
		angle_gen = math.atan2(y-self.start[1],x-self.start[0])
		for xr in self.randarray:
			x1 = int(x + final_dist * np.cos(xr+angle_gen-self.pps))
			y1 = int(y + final_dist * np.sin(xr+angle_gen-self.pps))
			self.bresenham((x,y), (x1,y1)) 
		return 

	def bresenham(self, start, end):
		if start[0] < 0 or start[1] < 0 or start[1] >= len(self.segmentatedImage) or start[0] >= len(self.segmentatedImage) or np.array_equal(self.GroundTruthImage[start[1]][start[0]], [0,0,0]):
			self.segmentatedImage[start[1]][start[0]] = self.GroundTruthImage[start[1]][start[0]]
			self.changedNodeList.append(start)
			return False
        
		(x0, y0) = start
		(x1, y1) = end
    
    
		m = None

    
		dx = abs(x1 - x0)
		dy = abs(y1 - y0)
		if dx == 0:
			m = 1.0
		else:
			m = dy/dx
        
		mm = False
        # step 3 perform test to check if pk < 0
		flag = True
    

    
		step = 1
		if x0>x1 or y0>y1:
			step = -1
        
		if m < 1:
			x0, x1 ,y0 ,y1 = y0, y1, x0, x1
			dx = abs(x1 - x0)
			dy = abs(y1 - y0)
			mm = True
        
		p0 = 2*dx - dy
		x = x0
		y = y0
		term = None
		for i in range(abs(y1-y0)):
			if flag:
				x_previous = x0
				p_previous = p0
				p = p0
				flag = False
			else:
				x_previous = x
				p_previous = p
        
    
			if p >= 0:
				x = x + step

			p = p_previous + 2*dx -2*dy*(abs(x-x_previous))
			y = y + step
        
			if mm:
				if x < 0 or y < 0 or y >= len(self.segmentatedImage) or x >= len(self.segmentatedImage):
					return False
				
				if np.array_equal(self.segmentatedImage[x][y], self.GroundTruthImage[x][y]) == False:
					self.changedNodeList.append((y,x))
					self.segmentatedImage[x][y] = self.GroundTruthImage[x][y]

				if np.array_equal(self.GroundTruthImage[x][y],[0,0,0]):
					return False


			else:
				if x < 0 or y < 0 or y >= len(self.segmentatedImage) or x >= len(self.segmentatedImage):
					return False


				if np.array_equal(self.segmentatedImage[y][x], self.GroundTruthImage[y][x]) == False:
					self.changedNodeList.append((x,y))
					self.segmentatedImage[y][x] = self.GroundTruthImage[y][x]

				if np.array_equal(self.GroundTruthImage[y][x],[0,0,0]):
					return False

            
		return True

	def DLITERUN(self):
		slast = self.start
		self.Initialize()
		self.ComputeShortestPath()
		while self.start != self.goal:
			if self.nodeTree[self.start].cost == np.inf or (len(self.path) != 0 and self.start == self.path[-1]):
				print('No path found.')
				break
			self.path.append(self.start)
			express = None
			min_val = np.inf

			succ_list = self.get_neighbors(self.start)

			for p in succ_list:
				if ((qg := self.nodeTree[p].cost + self.linkCostGrabber(p)) < min_val):
					min_val = qg
					express = p


			self.castRays(express[0],express[1])

			if len(self.changedNodeList) > 0:
				self.h_cost = self.eudis5(self.start, self.goal)
				self.kM = self.kM + self.eudis5(slast, self.start)
				for a in self.changedNodeList:
					self.UpdateVertex(a)
				self.ComputeShortestPath()
				self.changedNodeList = []



			min_val = np.inf
			express=None
			for p in succ_list:
				if ((qg := self.nodeTree[p].cost + self.linkCostGrabber(p)) < min_val):
					min_val = qg
					express = p

			self.start = express
			if self.start == None:
				print('No path from this position found.')
				break


		if self.path[-1] == self.goal:
			print('Path found.')
		return self.path