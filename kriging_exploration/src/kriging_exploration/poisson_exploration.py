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
#        self.strategy=strategy        
        
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


    def _find_in_route(self,wp_name):
        found =False
#        for i in self.route:
#            if i.name == wp_name:
#                found=True
#                return found
        
        for i in self.explored_wp:
            if i == wp_name:
                found=True
                return found

        for i in self.targets:
            if i.name == wp_name:
                found=True
                return found
        
        return found


    def get_greedy_target(self, model_variance):
        print "getting Greedy..."
        #x ,y =np.where(model_variance==np.max(model_variance))
        #print x,y
        
        found = False
        maxes1= np.copy(model_variance)
        maxes = np.reshape(maxes1, -1)
        #print "-------"
        maxes.sort()
        maxes = maxes[::-1]
        #print maxes
        maxind=0
        
        while not found:
            x ,y = np.where(model_variance==maxes[maxind])
            #print "looking for: ", maxes[maxind], " (", x[0],",",y[0],")"
            for j in self.tmap.waypoints:
                if (x[0],y[0]) == j.ind:
                    if not self._find_in_route(j.name):
                        to_append=j                        
                        found=True
                        break
            maxind+=1
        print "Going To:"+to_append.name
        self.targets.append(to_append)        
    

    def get_next_target(self):
        return self.targets[0]
        
        
        
    def set_wp_as_explored(self, wp_name):
        self.explored_wp.append(wp_name)
        tnames=[x.name for x in self.targets]
        self.targets.pop(tnames.index(wp_name))