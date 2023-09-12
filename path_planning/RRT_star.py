from logging import root
import math
from platform import node
import queue
import time
import numpy as np
import cv2
import random
import tcod

#represent vertices and edges implicitly. 
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
        self.node_List = {}
        self.node_List[root] = Node()
        self.root = root

    def add_to_tree(self, parent, new_child):
        self.node_List[parent].add_Child(new_child)
        self.node_List[new_child] = Node(parent)
        self.node_List[new_child].cost= self.node_List[parent].cost + np.linalg.norm(np.subtract(parent,new_child))

    def update_parent_and_cost(self, new_parent, child):
        assert(new_parent in self.node_List)
        tmp_parent = self.node_List[child].parent
        self.node_List[child].parent, self.node_List[child].cost = (new_parent, self.node_List[new_parent].cost+np.linalg.norm(np.subtract(new_parent,child)))
        self.node_List[new_parent].add_Child(child)
        if tmp_parent != None:
            self.node_List[tmp_parent].remove_child(child)


class RRT_Star:
    def __init__(self, root, end_point, step_size, iteration, radius, image):
        self.root_tree = Tree(root)
        self.end_point = end_point
        self.step_size = step_size
        self.radius = radius
        self.nodes_expanded = iteration
        self.img = image
        self.img_height = len(image)
        self.img_width = len(image[0])
        random.seed(0)

    def generate_random_sample(self):
        random_point = (random.randint(0, self.img_width-1), random.randint(0, self.img_height-1))
        while random_point in self.root_tree.node_List or (random_point[0] >= self.img_width or random_point[0] < 0 or random_point[1] >= self.img_height or random_point[1] < 0) or np.array_equal(self.img[random_point[1]][random_point[0]],[0,0,0]):
                random_point = (random.randint(0, self.img_width-1), random.randint(0, self.img_height-1))
        return random_point

    def euclidean_distance_grid(self,a,b):
        d = math.hypot(b[0]-a[0],b[1]-a[1])
        return d

    def get_los(self, first_point, second_point):
        line_algo_points = tcod.los.bresenham(first_point, second_point).tolist()
        for a in line_algo_points:
            if np.array_equal(self.img[a[1]][a[0]], [0,0,0]):
                return False
    
        return True

    def nearest_neighbor(self, random_sample):
        min_value = self.euclidean_distance_grid(self.root_tree.root, random_sample)
        return_vertex = self.root_tree.root
        for a in self.root_tree.node_List:
            if a == self.root_tree.root:
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

    def STEER_INPUT(self, x_rand, x_near):
      x_point = None
      x_last = x_near
      sampled_Point = self.steer(x_near, x_rand, self.step_size)
      sampled_Point = (int(sampled_Point[0]), int(sampled_Point[1]))
      line_Check = tcod.los.bresenham(x_near, sampled_Point).tolist()
      controller = True
      for a in line_Check:
          if np.array_equal(self.img[a[1]][a[0]],[0, 0, 0]):
              controller = False
              break

      if controller:
          self.root_tree.add_to_tree(x_last, sampled_Point)
          x_point = sampled_Point

      return x_point

    def k_nearest_neighbors(self, point, num_points=None, get_radius=False):
        if get_radius:
            array_to_return = []
            for jk in self.root_tree.node_List.keys():
                if self.euclidean_distance_grid(jk, point) <= float(self.radius) and self.get_los(jk, point):
                    array_to_return.append(jk)
            return array_to_return

        temp_dict = {}
        return_Arr = []
        for k in self.root_tree.node_List.keys():
            temp_dict[k] = self.euclidean_distance_grid(k, point)
        if num_points > len(temp_dict):
            for a in range(len(temp_dict)):
                z = min(temp_dict, key=temp_dict.get)
                del temp_dict[z]
                return_Arr.append(z)
        else:
            for a in range(num_points):
                z = min(temp_dict, key=temp_dict.get)
                del temp_dict[z]
                return_Arr.append(z)

        return return_Arr

    def run(self):
        if np.all(self.img==0):
            return None
        for p in range(self.nodes_expanded):
            x_r = self.generate_random_sample()
            x_n = self.nearest_neighbor(x_r)
            x_new = self.STEER_INPUT(x_r, x_n)
            if x_new is not None:
                important_Neighbors = self.k_nearest_neighbors(x_new, get_radius=True)
                minimum = math.inf
                best_parent = None
                index = 0
                for y in important_Neighbors:
                    if (index := self.euclidean_distance_grid(y, x_new)+self.root_tree.node_List[y].cost < minimum):
                        minimum = index
                        best_parent = y

                if best_parent != self.root_tree.node_List[x_new].parent:
                    self.root_tree.update_parent_and_cost(best_parent, x_new)

                for xy in important_Neighbors:
                    if self.euclidean_distance_grid(x_new, xy)+self.root_tree.node_List[x_new].cost < self.root_tree.node_List[xy].cost:
                        self.root_tree.update_parent_and_cost(x_new, xy)


        conti = True
        blind_path = []
        if self.end_point in self.root_tree.node_List:
            tmpo = self.end_point
            while tmpo is not None:
                blind_path.append(tmpo)
                tmpo = self.root_tree.node_List[tmpo].parent
            return blind_path

        min_cost = math.inf
        key_list = list(self.root_tree.node_List.keys())
        for keys in key_list:
            if self.get_los(keys, self.end_point):
                if (best_goal_cost:=self.euclidean_distance_grid(self.end_point, keys) + self.root_tree.node_List[keys].cost) < min_cost:
                    min_cost = best_goal_cost
                    if self.end_point not in self.root_tree.node_List:
                        self.root_tree.add_to_tree(keys, self.end_point)
                    else:
                        self.root_tree.update_parent_and_cost(keys, self.end_point)

        print('Num nodes in tree:', len(self.root_tree.node_List))
        if self.end_point not in self.root_tree.node_List or min_cost == math.inf:
            return None

        tmpo = self.end_point
        while tmpo is not None:
            blind_path.append(tmpo)
            tmpo = self.root_tree.node_List[tmpo].parent
        blind_path.reverse()
        return blind_path





            




