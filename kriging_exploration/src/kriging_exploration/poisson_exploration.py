import yaml
import rospy
import random
import time

import numpy as np

from kriging_exploration.srv import GetTsp
import matplotlib.pyplot as plt





class PoissonExplorationPlan(object):
    
    def __init__(self, topo_map, cell_size=5):
        self.max_iters=1000
        self.targets=[]
        self.explored_wp=[]
        self.explo_list=[]
        self.route=[]
        self.route_nodes=[]
        self.tmap = topo_map
        self.cell_size=cell_size
#        self.strategy=strategy        
        
    def get_random_target(self):
        ntries=0
        rwo= random.choice(self.tmap.waypoints)

        while rwo.name in self.explored_wp or self.distance_to_explored(rwo) <= (self.cell_size*4):
            rwo= random.choice(self.tmap.waypoints)
            ntries+=1
            if ntries > len(self.explored_wp)*2:
                break

        if rwo in self.explored_wp:
            self.targets.append(None)

        else:
            print "Going To:"+rwo.name
            print "Distance to explored ", self.distance_to_explored(rwo)  
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

    def distance_to_explored(self, wp):
        distance=1000
        for i in self.explo_list :
            d = wp-i
            if d[0]<distance:
                distance=d[0]
        return distance

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
        
        while not found and maxind<len(maxes):
            x ,y = np.where(model_variance==maxes[maxind])
            #print "looking for: ", maxes[maxind], " (", x[0],",",y[0],")"
            for j in self.tmap.waypoints:
                if (x[0],y[0]) == j.ind:
                    if not self._find_in_route(j.name):
                        if self.distance_to_explored(j) >= (self.cell_size*4):
                            to_append=j
                            found=True
                            break
            maxind+=1
        if found:
            print "Going To:"+to_append.name
            print "Distance to explored ", self.distance_to_explored(to_append)
            self.targets.append(to_append)  
        else:
            self.get_random_target()


    def add_montecarlo_goal(self, model_variance):
        print "getting Montecarlo ..."
        st = time.time()
        found = False
        maxes1= np.copy(model_variance)
        maxes = np.reshape(maxes1, -1)
        #print "-------"
        maxes.sort()
        maxes = maxes[::-1]
        #print maxes
        maxind=0
        
        var_average= np.mean(model_variance)
        
        var_cut=var_average*0.9
        varis=[]
        wps=[]
        
        while not found:
            x ,y = np.where(model_variance==maxes[maxind])

            for j in self.tmap.waypoints:
                if (x[0],y[0]) == j.ind:
                    #dist = j.coord - current_coord
                    if not self._find_in_route(j.name): #and np.abs(dist[0]) :
                        wps.append(j)
                        varis.append(maxes[maxind])
            
            if maxes[maxind] > var_cut:
                maxind+=1                
            else:
                found=True
                break
        
        suma = np.sum(varis)
        scores = []
        for i in varis:
            kkl = np.ceil((i*1000)/suma)
            scores.append(int(kkl))
#        print "Going To:"
        #print varis,
        #print scores, var_cut
        mctargets=[]
        
        for i in range(len(varis)):
            for j in range(scores[i]):
                mctargets.append(wps[i])
                
        print mctargets, len(mctargets)
        np.random.shuffle(mctargets)
        inds=np.random.randint(len(mctargets))

        while self.distance_to_explored(mctargets[inds]) <= (self.cell_size*4):
            np.random.shuffle(mctargets)
            inds=np.random.randint(len(mctargets))
            
        
        
        at = time.time()-st
        print "search time: ", at
#        return mctargets[inds]
        print "Going To:"+mctargets[inds].name
        print "Distance to explored ", self.distance_to_explored(mctargets[inds])
        self.targets.append(mctargets[inds])
        #self.route.append(mctargets[inds])        
            
            

    def get_next_target(self):
        return self.targets[0]
        
        
        
    def set_wp_as_explored(self, wp_name):
        self.explored_wp.append(wp_name)
        tnames=[x.name for x in self.targets]
        bb = self.targets.pop(tnames.index(wp_name))
        self.explo_list.append(bb)