
from logging import root
import math
import queue
import time
import numpy as np
import cv2
import random
import tcod

class ELLIPSE:
    def __init__(self, a, b, num_points, start, end):
        self.a = a
        self.b = b
        self.num_points = num_points
        self.start = start
        self.end = end
        self.angle_gen = math.atan2(self.end[1]-self.start[1], self.end[0]-self.start[0])
    
    def generate_theta(self, a, b):
            u = random.random() / 4.0
            theta = np.arctan(self.b/self.a * np.tan(2*np.pi*u))

            v = random.random()
            if v < 0.25:
                return theta
            elif v < 0.5:
                return np.pi - theta
            elif v < 0.75:
                return np.pi + theta
            else:
                return -theta

    def radius(self, a, b, theta):
            return self.a * self.b / np.sqrt((b*np.cos(theta))**2 + (a*np.sin(theta))**2)


    def random_point(self, major_axis, minor_axis, center, qa):
            random_theta = self.generate_theta(self.a, self.b)
            max_radius = self.radius(self.a, self.b, random_theta)
            random_radius = max_radius * np.sqrt(random.random())
            f = round(random_radius * np.cos(random_theta))
            s = round(random_radius * np.sin(random_theta))
            lio = self.rotate((0, 0), (f, s), self.angle_gen)
            return (int(lio[0]+center[0]), int(lio[1]+center[1]))


    def rotate(self, origin, point, angle):
            """
            Rotate a point counterclockwise by a given angle around a given origin.

            The angle should be given in radians.
            """
            ox, oy = origin
            px, py = point

            qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
            qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
            return qx, qy

    def midpoint(self, p1, p2):
            return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)

    def ret_list(self):
        points = [self.random_point(self.a, self.b, self.midpoint(self.start, self.end),  self.angle_gen) for _ in range(self.num_points)]
        return points

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
        if parent == new_child:
            print("parent is the child.")
        assert(parent != new_child)
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
        assert(new_parent in self.vertices_and_edges and new_parent != child)
        tmp_parent = self.vertices_and_edges[child].parent
        self.vertices_and_edges[child].parent, self.vertices_and_edges[child].cost = (new_parent, self.vertices_and_edges[new_parent].cost+np.linalg.norm(np.subtract(new_parent,child)))
        self.vertices_and_edges[new_parent].add_Child(child)
        if tmp_parent != None:
            self.vertices_and_edges[tmp_parent].remove_child(child)


class Informed_RRT_star:
    def __init__(self, root, end_point, n_iterations, step_size, RRT_radius, goal_radius, image):
        self.tree = Tree(root)
        self.start = root
        self.end_point = end_point
        self.nodes_expanded = n_iterations
        self.step_size = step_size
        self.goal_radius = goal_radius
        self.RRT_radius = RRT_radius
        self.img = image
        self.ellipse_values = ELLIPSE(math.inf, None, None, self.start, self.end_point)
        self.ellipse_points_holder = []
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
                if self.euclidean_distance_grid(jk, point) <= float(self.RRT_radius) and self.get_los(jk, point) and jk != point:
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

    def Sample(self, best_Cost):
        if best_Cost < math.inf:
            assert(best_Cost >= self.minimum_cost_distance)
            #x_Centre = (int((self.tree.root[0]+self.end_point[0])/2), int((self.tree.root[1]+self.end_point[1])/2))
           # xop =  math.sqrt((best_Cost ** 2) - (self.minimum_cost_distance ** 2))
            #return self.generate_point_within_ellipse(x_Centre, best_Cost, xop)
            if not self.ellipse_points_holder:
                return None
            random_point = random.sample(self.ellipse_points_holder, 1)[0]

            while (random_point in self.tree.vertices_and_edges or random_point[0] >= self.img_width or random_point[0] < 0 or random_point[1] >= self.img_height or random_point[1] < 0):
                self.ellipse_points_holder.remove(random_point)
                random_point = random.sample(self.ellipse_points_holder, 1)[0]

            return random_point

        else:
            return self.generate_random_sample()


    def run(self):
        print("Beginning Informed RRT* Search from ", self.tree.root, " to ", self.end_point)
        X_soln = []
        c_best = math.inf
        index_Hold = None
        #while len(self.tree.vertices_and_edges) < self.nodes_expanded:
        for i in range(self.nodes_expanded):
        
            if len(X_soln) != 0 or self.end_point in self.tree.vertices_and_edges:
                if self.end_point in self.tree.vertices_and_edges:
                   c_best = self.tree.vertices_and_edges[self.end_point].cost
                else:
                    c_best, index_Hold = self.tree.get_min_cost_val_and_index(X_soln)   
                    c_best += self.euclidean_distance_grid(X_soln[index_Hold] , self.end_point)
                if c_best < self.ellipse_values.a:
                    print('improving')
                    self.ellipse_values.a = c_best

                    assert(c_best >= self.minimum_cost_distance)
                    self.ellipse_values.b = math.sqrt((c_best ** 2) - (self.minimum_cost_distance ** 2))
                    print(c_best, self.minimum_cost_distance, self.ellipse_values.b)
                    self.ellipse_values.num_points = int(math.pi * self.ellipse_values.a * self.ellipse_values.b )
                    self.ellipse_points_holder = list(set(self.ellipse_values.ret_list()))
                    for tz in self.ellipse_points_holder[:]:
                        if tz[0] >= self.img_width or tz[0] < 0 or tz[1] >= self.img_height or tz[1] < 0 or np.array_equal(self.img[tz[1]][tz[0]] , [0, 0, 0]):
                            self.ellipse_points_holder.remove(tz)

            x_rand = self.Sample(c_best)
            if x_rand == None:
                break
            x_nearest = self.nearest_neighbor(x_rand)
            x_new = self.steer(x_nearest, x_rand, self.step_size)
            x_new = (int(x_new[0]), int(x_new[1]))
            if self.get_los(x_nearest, x_new):
                self.tree.add_to_tree(x_nearest, x_new)
                
                #if x_new == self.end_point:
                #    break
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

        if len(X_soln) > 0:
            c_best, index_Hold = self.tree.get_min_cost_val_and_index(X_soln)   
            if self.end_point in self.tree.vertices_and_edges:
                if self.tree.vertices_and_edges[self.end_point].cost > self.tree.vertices_and_edges[X_soln[index_Hold]].cost + self.euclidean_distance_grid(X_soln[index_Hold], self.end_point):
                    self.tree.update_parent_and_cost(X_soln[index_Hold], self.end_point)

            else:
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
            print(tmpo)
            blind_path.append(tmpo)
            if tmpo is None:
                blind_path.remove(tmpo)
            
        blind_path.reverse()
        self.tree.vertices_and_edges.clear()
        return blind_path



                        





