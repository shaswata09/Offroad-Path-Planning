from http.client import INSUFFICIENT_STORAGE
from logging import root
import math
import queue
import time
import numpy as np
import cv2
import random
import tcod


class Node:
    def __init__(self, parent=None):
        self.parent = parent
        self.children = []
        self.cost = 0

    def change_parent(self, new_parent):
        self.parent = new_parent

    def remove_child(self, child):
        if child in self.children:
            self.children.remove(child)
        return None

    def add_Child(self, child):
        self.children.append(child)



class Tree:
    def __init__(self, root):
        self.root = root
        self.vertices_and_edges = {}
        self.vertices_and_edges[self.root] = Node()
    def add_to_tree(self, parent, new_child):
        self.vertices_and_edges[parent].add_Child(new_child)
        self.vertices_and_edges[new_child] = Node(parent)
        self.vertices_and_edges[new_child].cost = self.vertices_and_edges[parent].cost + self.T_euclidean_distance_grid(parent, new_child)

    def T_euclidean_distance_grid(self,a,b):
        d = math.hypot(b[0]-a[0],b[1]-a[1])
        return d



    def get_min_cost_val_and_index(self, temp_array):
        assert(len(temp_array) > 0)
        min_value = math.inf
        index = 0
        for a in range(len(temp_array)):
            if min_value > self.vertices_and_edges[temp_array[a]].cost:
                min_value = self.vertices_and_edges[temp_array[a]].cost
                index = a

        return min_value, index

    def update_parent_and_cost(self, new_parent, child):
        assert(new_parent in self.vertices_and_edges)
        tmp_parent = self.vertices_and_edges[child].parent
        self.vertices_and_edges[child].parent, self.vertices_and_edges[child].cost = (new_parent, self.vertices_and_edges[new_parent].cost+np.linalg.norm(np.subtract(new_parent,child)))
        self.vertices_and_edges[new_parent].add_Child(child)
        if tmp_parent != None:
            self.vertices_and_edges[tmp_parent].remove_child(child)


class Informed_RRT_star:
    def __init__(self, root, end_point, n_iterations, step_size, RRT_radius, goal_radius, image):
        self.tree = Tree(root)
        self.end_point = end_point
        self.n_iterations = n_iterations
        self.step_size = step_size
        self.goal_radius = goal_radius
        self.RRT_radius = RRT_radius
        self.img = image
        self.minimum_cost_distance = self.euclidean_distance_grid(self.tree.root, self.end_point)
        self.img_height = len(image)
        self.img_width = len(image[0])

    def generate_random_sample(self):
        random_point = (random.randint(0, self.img_width), random.randint(0, self.img_height))
        while random_point in self.tree.vertices_and_edges or (random_point[0] >= self.img_width or random_point[0] < 0 or random_point[1] >= self.img_height or random_point[1] < 0) or np.array_equal(self.img[random_point[1]][random_point[0]],[0,0,0]):
                random_point = (random.randint(0, self.img_width), random.randint(0, self.img_height))
        return random_point


    def k_nearest_neighbors(self, point, num_points=None, get_radius=False):
        if get_radius:
            array_to_return = []
            for jk in self.tree.vertices_and_edges.keys():
                if self.euclidean_distance_grid(jk, point) <= float(self.RRT_radius) and self.get_los(jk, point):
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

    def nearest_neighbor(self, random_sample):
        min_value = self.euclidean_distance_grid(self.tree.root, random_sample)
        return_vertex = self.tree.root
        for a in self.tree.vertices_and_edges:
            if a == self.tree.root:
                continue
            else:
                tmp_dist_holder = self.euclidean_distance_grid(random_sample, a)
                if tmp_dist_holder < min_value:
                    min_value = tmp_dist_holder
                    return_vertex = a


        return return_vertex



    def steer(self, begin, end_v, step_S):
        if self.euclidean_distance_grid(begin, end_v) <= float(step_S):
            return end_v
        start, end = np.array(begin), np.array(end_v)
        v = end - start
        u = v / (np.sqrt(np.sum(v ** 2)))
        steered_point = start + u * step_S
        return tuple(steered_point)

    def euclidean_distance_grid(self,a,b):
        d = math.hypot(b[0]-a[0],b[1]-a[1])
        return d
    


    #can be changed when a local planner is present. right now its just straight lines, so need to check if there exists a path in the goal region. 
    def InGoalRegion(self, point):
        if self.euclidean_distance_grid(point, self.end_point) <= self.goal_radius and self.get_los(point, self.end_point):
            return True
        return False

   
    def point_Check(self, point, center, major_axis, minor_axis):
        if ( (float(point[1]-center[0])) ** 2 / ((major_axis / 2) ** 2)) + ((float(point[0]-center[1])) ** 2 / ((minor_axis / 2) ** 2)) <= 1:
            return True

        return False

    def generate_point_within_ellipse(self, center, major_axis, minor_axis):
        random_point = (random.randint(0, self.img_width), random.randint(0, self.img_height))
        while random_point in self.tree.vertices_and_edges or (random_point[0] >= self.img_width or random_point[0] < 0 or random_point[1] >= self.img_height or random_point[1] < 0) or np.array_equal(self.img[random_point[1]][random_point[0]],[0,0,0]) or self.point_Check(random_point, center, major_axis, minor_axis) == False: 
            random_point = (random.randint(0, self.img_width), random.randint(0, self.img_height))
        return random_point

    def Sample(self, best_Cost):
        if best_Cost < math.inf:
            cost_Min = self.euclidean_distance_grid(self.tree.root, self.end_point)
            assert(best_Cost > cost_Min)
            x_Centre = (int((self.tree.root[0]+self.end_point[0])/2), int((self.tree.root[1]+self.end_point[1])/2))
            xop =  math.sqrt((best_Cost ** 2) - (cost_Min ** 2))
            return self.generate_point_within_ellipse(x_Centre, best_Cost, xop)

        else:
            return self.generate_random_sample()


    def run(self):
        print("Beginning Informed RRT* Search from ", self.tree.root, " to ", self.end_point)
        X_soln = []
        c_best = math.inf
        index_Hold = None
        #while len(self.tree.vertices_and_edges) < self.n_iterations:
        for i in range(self.n_iterations):
            if len(X_soln) != 0:
                c_best, index_Hold = self.tree.get_min_cost_val_and_index(X_soln)   
                c_best += self.euclidean_distance_grid(X_soln[index_Hold] , self.end_point)

            x_rand = self.Sample(c_best)
            x_nearest = self.nearest_neighbor(x_rand)
            x_new = self.steer(x_nearest, x_rand, self.step_size)
            x_new = (int(x_new[0]), int(x_new[1]))
            if self.get_los(x_nearest, x_new):
                self.tree.add_to_tree(x_nearest, x_new)
                X_near = self.k_nearest_neighbors(x_new, get_radius=True)
                x_min = x_nearest
                c_min = self.tree.vertices_and_edges[x_min].cost + self.euclidean_distance_grid(x_nearest, x_new)
                for x in X_near:
                    c_new = self.tree.vertices_and_edges[x].cost + self.euclidean_distance_grid(x, x_new)
                    if c_new < c_min and self.get_los(x, x_new):
                        x_min = x
                        c_min = c_new
                        self.tree.update_parent_and_cost(x, x_new)

                for x_p in X_near:
                    c_near = self.tree.vertices_and_edges[x_p].cost
                    c_new = self.tree.vertices_and_edges[x_new].cost + self.euclidean_distance_grid(x_new, x_p)
                    if c_new < c_near and self.get_los(x_new, x_p):
                        self.tree.update_parent_and_cost(x_new, x_p)

                if self.InGoalRegion(x_new):
                    X_soln.append(x_new)

        if self.end_point in self.tree.vertices_and_edges and len(X_soln) > 0 and self.tree.vertices_and_edges[X_soln[index_Hold]].cost+self.euclidean_distance_grid(X_soln[index_Hold], self.end_point) < self.tree.vertices_and_edges[self.end_point].cost:
            self.tree.update_parent_and_cost(X_soln[index_Hold], self.end_point)

        elif self.end_point not in self.tree.vertices_and_edges and len(X_soln) > 0:
            self.tree.add_to_tree(X_soln[index_Hold], self.end_point)

        elif self.end_point not in self.tree.vertices_and_edges and len(X_soln) == 0:
            print("Solution not found.")
            return []

        print("path found.")
        blind_path = []
        tmpo = self.end_point
        blind_path.append(tmpo)
        while tmpo is not None:
            tmpo = self.tree.vertices_and_edges[tmpo].parent
            blind_path.append(tmpo)
            if tmpo is None:
                blind_path.remove(tmpo)
        print(blind_path)
        blind_path.reverse()
        self.tree.vertices_and_edges.clear()
        return blind_path



                        










