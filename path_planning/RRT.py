from logging import root
import math
import queue
import time
import numpy as np
import cv2
import random
import tcod

class Node:
    def __init__(self, parent):
        self.parent = parent
    
    def change_parent(self, new_parent):
        self.parent = new_parent
class Tree:
    def __init__(self, root):
        self.root = root
        self.edges = []
        self.vertices = [self.root]

    def add_to_tree(self, parent, new_child):
        self.edges.append([parent, new_child])
        self.vertices.append(new_child)



class RRTtree:
    def __init__(self, root, end_point, step_size, k_vertices, image, imageWindow):
        self.root_tree = Tree(root)
        self.step_size = step_size
        self.end_point = end_point
        self.img_width = len(image[0])
        self.img = image
        self.img_height = len(image)
        self.k_vertices = k_vertices
        self.node_list = {}
        self.n_iterations = 0
        self.imageWindow = imageWindow

    def generate_random_sample(self):
        random_point = (random.randint(0, self.img_width), random.randint(0, self.img_height))
        while random_point in self.root_tree.vertices or (random_point[0] >= self.img_width or random_point[0] < 0 or random_point[1] >= self.img_height or random_point[1] < 0) or np.array_equal(self.img[random_point[1]][random_point[0]],[0,0,0]):
                random_point = (random.randint(0, self.img_width), random.randint(0, self.img_height))
        return random_point

    def euclidean_distance_grid(self,a,b):
        d = math.hypot(b[0]-a[0],b[1]-a[1])
        return d

    def nearest_neighbor(self, random_sample):
        min_value = self.euclidean_distance_grid(self.root_tree.root, random_sample)
        return_vertex = self.root_tree.root
        for a in self.root_tree.vertices:
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

    def es_points_along_line(self, start, end):
        d = self.euclidean_distance_grid(start, end)
        n_points = int(np.ceil(d / self.step_size))
        if n_points > 1:
            step = d / (n_points - 1)
            for i in range(n_points):
                next_point = self.steer(start, end, i * step)
                yield next_point

    def STEER_INPUT(self, x_rand, x_near):
     #   x_rand = self.generate_random_sample()
      #  x_near = self.nearest_neighbor(x_rand)
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
      return x_point




    def k_nearest_neighbors(self, point, num_points):
        temp_dict = {}
        return_Arr = []
        for k in self.root_tree.vertices:
            temp_dict[k] = self.euclidean_distance_grid(k, point)
        assert(len(self.root_tree.vertices) > num_points)
        for a in range(num_points):
            z = min(temp_dict, key=temp_dict.get)
            del temp_dict[z]
            return_Arr.append(z)

        return return_Arr



    def add_edges_to_dict(self):
        for p in self.root_tree.edges:
            if p[1] not in self.node_list:
                self.node_list[p[1]] = Node(p[0])
            print(len(self.node_list))

    def run(self):
        print('Starting RRT search from grid cell ', self.root_tree.root, ' to goal cell ', self.end_point)
    #    print('Starting f-value is ', start_node.h + start_node.cost)
        i = 1
        while len(self.root_tree.vertices) < self.k_vertices:
            self.n_iterations += 1
            if i % 5 == 0:
                cv2.imshow(self.imageWindow,self.img)
                cv2.waitKey(1)
            x_r = self.generate_random_sample()
            x_n = self.nearest_neighbor(x_r)
            x_new = self.STEER_INPUT(x_r, x_n)
            print(len(self.root_tree.vertices))




        conti = True
        x_nn = self.k_nearest_neighbors(self.end_point, 5)
        for a in x_nn:
            loip = tcod.los.bresenham(a, self.end_point).tolist()
            conti = True
            for p in loip:
                if np.array_equal(self.img[p[1]][p[0]],[0, 0, 0]):
                    conti = False
                    break
            if conti == True:
                self.root_tree.add_to_tree(a, self.end_point)
                break
        if conti:
            print("path found")
            self.add_edges_to_dict()
            temp_point = self.end_point
            blind_path = []
            blind_path.append(temp_point)
            if self.root_tree.root == self.end_point:
                return blind_path
            while True:
                temp_point = self.node_list[temp_point].parent
                blind_path.append(temp_point)
                if temp_point == self.root_tree.root:
                    break
            print(blind_path)
            blind_path.reverse()
            return blind_path
        else:
            print("path not found")
            return None





















