from platform import node
import cv2
import os
import numpy as np
import math
import heapq
from copy import deepcopy
import random
import sys

class Node: 
	def __init__(self, cost=sys.float_info.max, rhs = math.inf, key=(math.inf, math.inf)):
		self.cost = cost
		self.rhs = rhs
		self.key = key


class DLITESEARCH:
	def __init__(self, start, goal, segmentatedImage, GroundTruthImage):
		self.start = start
		self.goal = goal
		print('Starting search from', self.start, 'to', self.goal)
		self.img_width = len(segmentatedImage)
		self.img_height = len(segmentatedImage[0])
		self.path = []
		self.segmentatedImage = np.zeros([self.img_width,self.img_height,3],dtype=np.uint8)
		self.segmentatedImage.fill(255) 
		self.old_segmentatedImage = deepcopy(self.segmentatedImage)
		self.GroundTruthImage = GroundTruthImage
		self.nodeTree = {}
		self.changedNodeList = []
		self.kM = 0
		self.replans = 0
		self.U = []
		self.open_Set_check = set()
		for a in range(self.img_width):
			for p in range(self.img_height):
				self.nodeTree[(a,p)] = Node()



	def Initialize(self):
		self.U = []
		self.kM = 0
		self.nodeTree[self.goal] = Node(rhs=0)
		#heapq.heappush(self.U, (self.CalculateKey(self.goal), self.goal))
		heapq.heappush(self.U, ((self.eudis5(self.goal, self.start),0), self.goal))

		self.open_Set_check.add(self.goal)

	def get_diagonals(self, state):
		diagonals = [tuple((state[0]+1, state[1]+1)), tuple((state[0]-1, state[1]+1)), tuple((state[0]+1, state[1]-1)), tuple((state[0]-1, state[1]-1))]
		diagonals = [x for x in diagonals if x[0] >=0 and x[0] < self.img_height and x[1]>=0 and x[1] < self.img_width]		
		return diagonals



	def eudis5(self, v1, v2):
		dist = [(a - b)**2 for a, b in zip(v1, v2)]
		dist = math.sqrt(sum(dist))
		return dist

	def CalculateKey(self, state):
		return (min(self.nodeTree[state].cost, self.nodeTree[state].rhs)+self.kM+self.eudis5(state, self.start), min(self.nodeTree[state].cost, self.nodeTree[state].rhs))


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
			neighbors.append(nextCell)

		return neighbors




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





		'''




		if state != self.goal:
			min_val = np.inf
			neighbors = self.get_neighbors(state)
			for p in neighbors:
				if min_val > (qrs:=self.linkCostGrabber(p)+self.nodeTree[p].cost):
					min_val = qrs
			
			self.nodeTree[state].rhs = min_val


		#if (opp:=self.OpenQueueAndIndexCheck(state))[0]:
		if state in self.open_Set_check:
			opp = self.OpenQueueAndIndexCheck(state)
			self.U.pop(opp[1])
			self.open_Set_check.discard(state)
			heapq.heapify(self.U)


		if self.nodeTree[state].cost != self.nodeTree[state].rhs:
			heapq.heappush(self.U, (self.CalculateKey(state), state))
			self.open_Set_check.add(state)
			#self.U.append((self.CalculateKey(state), state))
			#heapq.heapify(self.U)
		'''
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
	def _fpart(self, x):
		return x - int(x)

	def _rfpart(self, x):
		return 1 - self._fpart(x)

	def draw_line(self, p1, p2):
		"""Draws an anti-aliased line in img from p1 to p2 with the given color."""
		x1, y1 = p1
		x2, y2 = p2
		dx, dy = x2-x1, y2-y1
		steep = abs(dx) < abs(dy)
		p = lambda px, py: ((px,py), (py,px))[steep]

		if steep:
			x1, y1, x2, y2, dx, dy = y1, x1, y2, x2, dy, dx
		if x2 < x1:
			x1, x2, y1, y2 = x2, x1, y2, y1

		grad = dy/dx
		intery = y1 + self._rfpart(x1) * grad
		def draw_endpoint(pt):
			x, y = pt
			xend = round(x)
			yend = y + grad * (xend - x)
			xgap = self._rfpart(x + 0.5)
			px, py = int(xend), int(yend)

			return px

		xstart = draw_endpoint(p(*p1)) + 1
		xend = draw_endpoint(p(*p2))

		for x in range(xstart, xend):
			y = int(intery)
			if np.array_equal(self.segmentatedImage[y][x], [255, 0,0]):
						return False, None

					#predicted obstacle that was correct
			elif np.array_equal(self.segmentatedImage[y][x], self.GroundTruthImage[y][x]) and np.array_equal(self.GroundTruthImage[y][x], [0,0,0]):
						self.changedNodeList.append( tuple((x,y)) )
						self.segmentatedImage[y][x] = [255,0,0]
						return False, None

					#predicted obstacle that was free space
			elif np.array_equal(self.segmentatedImage[y][x], [0,0,0]) and np.array_equal(self.GroundTruthImage[y][x], [255, 255, 255]):
						self.changedNodeList.append( tuple((x,y)) )
						self.segmentatedImage[y][x] = [255,255,255]

					#predicted free space that was an obstacle
			elif np.array_equal(self.segmentatedImage[y][x], [255, 255, 255]) and np.array_equal(self.GroundTruthImage[y][x], [0, 0, 0]):
						self.changedNodeList.append( tuple((x,y)) )
						self.segmentatedImage[y][x] = [255,0,0]
						return False, None
			intery += grad




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

	def castRays(self, x,y):
		final_dist = 100
		randarray = np.linspace(0, 2*np.pi,	300)
		for xr in randarray:
			x1 = int(x + final_dist * np.cos(xr))
			y1 = int(y + final_dist * np.sin(xr))
			self.bresenham((x,y), (x1,y1)) 
			#self.draw_line((x,y), (x1, y1))
		return 


	def DLITERUN(self):
		s_last = self.start
		self.Initialize()
		self.castRays(self.start[0], self.start[1])
		self.changedNodeList = []
		self.path.append(self.start)
		self.ComputeShortestPath()
		'''

		while self.start != self.goal:
			neighbors = self.get_neighbors(self.start)
			min_val = np.inf
			new_state = None


			for n in neighbors:
				print(self.linkCostGrabber(n))
				if (temp_val:=self.linkCostGrabber(n) + self.nodeTree[n].cost) < min_val:
					min_val = temp_val
					new_state = n

			if new_state is not None:
				self.start = new_state
			else:
				print('Start state is blocked.')
				return self.path
			self.path.append(self.start)
			self.castRays()
			#print(self.nodeTree[self.start].cost)
			if len(self.changedNodeList) != 0:
				for a in self.changedNodeList:
					self.UpdateVertex(a)
				s_last = self.start
				self.ComputeShortestPath()
				self.changedNodeList = []

			print(self.start)
			print(self.linkCostGrabber(self.goal))
		self.path.append(self.goal)
		'''
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
				for b in neighbors:
					print(self.nodeTree[b].rhs, self.nodeTree[b].cost)

			self.castRays(self.start[0], self.start[1])
			if len(self.changedNodeList) > 0:
				
					
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
				self.replans += 1
				self.ComputeShortestPath()
				self.old_segmentatedImage = deepcopy(self.segmentatedImage)

		return self.path



