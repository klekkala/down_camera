"""
A crappy implementation of intelligent scissors/livewire algorithm
"""

from __future__ import division
import cv2
import numpy as np
import math

SQRT_0_5 = 0.70710678118654757

class Livewire():
    """
    A simple livewire implementation for verification using 
        1. Canny edge detector + gradient magnitude + gradient direction
        2. Dijkstra algorithm
    """
    
    def __init__(self, image):
        self.image = image
        self.x_lim = image.shape[0]
        self.y_lim = image.shape[1]
        # The values in cost matrix ranges from 0~1
        self.gradient = np.empty([self.x_lim, self.x_lim], dtype=object)
        
        self.flag = self._get_grad(self.image, self.gradient)
        self.n_pixs = self.x_lim * self.y_lim
        self.n_processed = 0
    

    def _get_neighbors(self, p):
        """s
        Return 8 neighbors around the pixel p
        """
        x, y = p
        x0 = 0 if x == 0 else x - 1
        x1 = self.x_lim if x == self.x_lim - 1 else x + 2
        y0 = 0 if y == 0 else y - 1
        y1 = self.y_lim if y == self.y_lim - 1 else y + 2
        
        return [(x, y) for x in xrange(x0, x1) for y in xrange(y0, y1) if (x, y) != p]

    def _get_grad(self, image, gradient):
        """
        Return the gradient magnitude of the image using heuristics
        """
        height, width = image.shape
        diagonal=0
        for i in range(0, height):
            for j in range(0, width):
                gradient[i,j] = {}
                maxval = 0
                neighbors = self._get_neighbors((i,j))
                print(type(neighbors))
                print(neighbors[1])
                for bors in neighbors:
                    #horizontal and vertical edges
                    diaganol = bors[0] == i or bors[1] == j

                    if diagonal:
                        gradient[i,j][bors] = math.fabs(image(i+1, j) - image(i, j-1))/math.sqrt(2)
                        maxval = max(maxval, gradient[i,j][bors])

                    else:
                        gradient[i,j][bors] = math.fabs(image(i, j-1) + image(i+1, j-1) - image(i, j+1) - image(i+1,j+1))/4
                        maxval = max(maxval, gradient[i,j][bors])
                
                for bors in neighbors:

                    if diagonal:
                        gradient[i,j][bors] = (math.sqrt(maxval) - gradient[i,j][bors])*1
                    else:
                        gradient[i,j][bors] = (math.sqrt(maxval) - gradient[i,j][bors])*1
        
        

        return 1

    def _local_cost(self, p, q):
        """
        Assumption: p & q are neighbors
        """
        
        # dlink is the cost of the link
        # dmax is the maximum link cost among all the links in the picture
        # clink is the final cost function

        return self.gradient[p[0], p[1]][q]

    def get_path_matrix(self, seed):
        """
        Get the back tracking matrix of the whole image from the cost matrix
        """
        neighbors = []          # 8 neighbors of the pixel being processed
        processed = set()       # Processed point
        cost = {seed: 0.0}      # Accumulated cost, initialized with seed to itself
        paths = {}

        self.n_processed = 0
        
        while cost:
            # Expand the minimum cost point
            p = min(cost, key=cost.get)
            neighbors = self._get_neighbors(p)
            processed.add(p)

            # Record accumulated costs and back tracking point for newly expanded points
            for q in [x for x in neighbors if x not in processed]:
                temp_cost = cost[p] + self._local_cost(p, q)
                if q in cost:
                    if temp_cost < cost[q]:
                        cost.pop(q)
                else:
                    cost[q] = temp_cost
                    processed.add(q)
                    paths[q] = p
            
            # Pop traversed points
            cost.pop(p)
            
            self.n_processed += 1
        
        return paths
