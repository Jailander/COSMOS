import yaml
import rospy
import random

import numpy as np

from kriging_exploration.srv import GetTsp
import matplotlib.pyplot as plt





class PoissonExplorationPlan(object):
    
    def __init__(self, topo_map):
        self.max_iters=1000        
        self.targets=[]
        self.explored_wp=[]
        self.route=[]
        self.route_nodes=[]
        self.tmap = topo_map
        
        
    def get_random_target(self):
        ntries=0
        rwo= random.choice(self.tmap.waypoints)

        while rwo.name in self.explored_wp:
            rwo= random.choice(self.tmap.waypoints)
            ntries+=1
            if ntries > len(self.explored_wp)*2:
                break

        if rwo in self.explored_wp:
            self.targets.append(None)

        else:
            self.targets.append(rwo)



    def get_next_target(self):
        return self.targets[0]
        
    def set_wp_as_explored(self, wp_name):
        self.explored_wp.append(wp_name)
        tnames=[x.name for x in self.targets]
        self.targets.pop(tnames.index(wp_name))