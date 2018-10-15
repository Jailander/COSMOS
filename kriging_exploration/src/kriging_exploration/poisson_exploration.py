import yaml
import rospy
import random
import time

import numpy as np

from kriging_exploration.srv import GetTsp
import matplotlib.pyplot as plt


def line_intersection(l1, l2):    
    line1 = [[l1[0].ind[0], l1[0].ind[1]], [l1[1].ind[0], l1[1].ind[1]]]
    line2 = [[l2[0].ind[0], l2[0].ind[1]], [l2[1].ind[0], l2[1].ind[1]]]    
    
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return False, (-1, -1)

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    if x>= min(line1[0][0], line1[1][0]) and x <= max(line1[0][0], line1[1][0]):
        if x>= min(line2[0][0], line2[1][0]) and x <= max(line2[0][0], line2[1][0]):
            if  y>= min(line1[0][1], line1[1][1]) and y <= max(line1[0][1], line1[1][1]):
                if  y>= min(line2[0][1], line2[1][1]) and y <= max(line2[0][1], line2[1][1]):
                    return True, (x, y)
    
    return False, (-1, -1)



class PoissonExplorationPlan(object):
    
    def __init__(self, topo_map, cell_size=5):
        self.as_targets=[]
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



    def add_mc_to_as(self, model_variance):
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

        print "Going To:"+mctargets[inds].name
        print "Distance to explored ", self.distance_to_explored(mctargets[inds])
        return mctargets[inds]


    def add_random_to_as(self):
        ntries=0
        rwo= random.choice(self.tmap.waypoints)
        tnames=[x.name for x in self.targets]

        while rwo.name in self.explored_wp or self.distance_to_explored(rwo) <= (self.cell_size*4) or rwo.name in tnames:
            rwo= random.choice(self.tmap.waypoints)
            ntries+=1
            if ntries > len(self.explored_wp)*2:
                break

#        if rwo in self.explored_wp:
#            self.targets.append(None)

#        else:
        print "Going To:"+rwo.name
        print "Distance to explored ", self.distance_to_explored(rwo)  
        return rwo


    def plan_as_goals(self, model_variance, remaining_time, initial_waypoint, min_scans_per_hour=5):
        print "AS planning ..."
        print "Remaining Time: "+str(remaining_time)
        min_targets_in_queue=int((remaining_time*min_scans_per_hour)/(3600))+1
        print "Minimun targets in queue: "+str(min_targets_in_queue)
        print "Current targets in queue: "+str(len(self.targets))
        st = time.time()
        
        if model_variance != None:
            self.remove_low_var_targets(model_variance)
            while len(self.targets) < min_targets_in_queue:
                mc_goal=self.add_mc_to_as(model_variance)
                self.targets.append(mc_goal)
        else:
            while len(self.targets) < min_targets_in_queue:
                mc_goal=self.add_random_to_as()       
                self.targets.append(mc_goal)


        self._get_plan(initial_waypoint)


        at = time.time()-st
        print "search time: ", at

        #self.route.append(mctargets[inds])        

    def remove_low_var_targets(self, model_variance):
        var_average= np.mean(model_variance)
        var_std= np.std(model_variance)
        var_cut=var_average-(0.5*var_std)
        #removed=0
        to_pop=[]
        for i in range(len(self.targets)):
            print i, model_variance.shape, len(self.targets), self.targets[i].ind[0],self.targets[i].ind[1]
            if model_variance[self.targets[i].ind[0]][self.targets[i].ind[1]]<var_cut:
                to_pop.append(i)
                #removed=removed+1

        to_pop.reverse()
        print "removing ",len(to_pop), "goals from original ",len(self.targets), "nodes"
        for i in to_pop:
            self.targets.pop(i)
            
            

    def get_next_target(self):
        return self.targets[0]
        
        
    def set_wp_as_explored(self, wp_name):
        self.explored_wp.append(wp_name)
#       print "targets: ", self.targets
        tnames=[x.name for x in self.targets]
        bb = self.targets.pop(tnames.index(wp_name))
#       print "targets: ", self.targets
        self.explo_list.append(bb)
        
        
        
        
    def _get_plan(self, initial_waypoint):
#        route, dist = self._create_greedy_plan(initial_waypoint)
#        route.pop(0)
        route, dist = self._get_tsp_plan(initial_waypoint)
        self.targets=route
                   
        


    def get_route_dist(self, route):
        dist = 0
#        print range(1, len(route)-1)
        for i in range(1, len(route)):
            #print i, len(route)
            d = route[i].coord - route[i-1].coord
            dist+=d[0]
        return dist
    
    
            
    def _get_tsp_plan(self, initial_waypoint):
        route=[]
        coordsx=[]
        coordsy=[]

        for i in self.targets:
            if i.name != initial_waypoint.name:
                coordsx.append(float(i.ind[0]))
                coordsy.append(float(i.ind[1]))
            else:
                coordsx.insert(0,float(i.ind[0]))
                coordsy.insert(0,float(i.ind[1]))

        #print coordsx
        #print coordsy
        
        print "Waiting for Service!!!!!"
        rospy.wait_for_service('get_tsp')
        try:
            get_tsp = rospy.ServiceProxy('/get_tsp', GetTsp)
            resp1 = get_tsp(coordsx, coordsy)
            #print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        plt.plot(resp1.x, resp1.y, '-o')
        plt.savefig('tsp-opt.png')
        
        #print "Remember initial wP: ", initial_waypoint
        for i in range(len(resp1.x)):
            for j in self.targets:
                if (resp1.x[i],resp1.y[i]) == j.ind:
                    route.append(j)
                    break

        return route, self.get_route_dist(route)
        
 