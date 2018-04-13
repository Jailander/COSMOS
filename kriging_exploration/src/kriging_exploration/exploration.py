import utm
import yaml
import rospy
import random

import numpy as np

from map_coords import MapCoords
from kriging_exploration.srv import GetTsp
import matplotlib.pyplot as plt

def line_intersection(l1, l2):
#    line1 = [l1[0].coord.northing, l1[0].coord.easting, l1[1].coord.northing, l1[1].coord.easting]
#    line2 = [l2[0].coord.northing, l2[0].coord.easting, l2[1].coord.northing, l2[1].coord.easting]
    
    line1 = [[l1[0].ind[0], l1[0].ind[1]], [l1[1].ind[0], l1[1].ind[1]]]
    line2 = [[l2[0].ind[0], l2[0].ind[1]], [l2[1].ind[0], l2[1].ind[1]]]    
    
#    xdiff = (line1[0].coord.northing - line1[1].coord.northing, line2[0].coord.northing - line2[1].coord.northing)
#    ydiff = (line1[0].coord.easting - line1[1].coord.easting, line2[0].coord.easting - line2[1].coord.easting) #Typo was here
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

    #print x,y
    #check interval

    if x>= min(line1[0][0], line1[1][0]) and x <= max(line1[0][0], line1[1][0]):
        if x>= min(line2[0][0], line2[1][0]) and x <= max(line2[0][0], line2[1][0]):
            if  y>= min(line1[0][1], line1[1][1]) and y <= max(line1[0][1], line1[1][1]):
                if  y>= min(line2[0][1], line2[1][1]) and y <= max(line2[0][1], line2[1][1]):
                    return True, (x, y)
    
    return False, (-1, -1)




class ExplorationPlan(object):
    def __init__(self, topo_map, initial_waypoint, percentage=0.05, exploration_type='ac', ac_model='random', ac_coords=[]):
        self.max_iters=1000        
        self.targets=[]
        self.explored_wp=[]
        self.route=[]
        self.route_nodes=[]
        self.tmap = topo_map
        
        if exploration_type == 'ac':
            if ac_model=='random':
                self._generate_targets(topo_map, initial_waypoint, percentage)
            else:
                self._get_ac_targets(topo_map, initial_waypoint, ac_coords)
        else:
            self.get_greedy_initial_goals(initial_waypoint)


            
        #print self.targets
        self._get_plan(initial_waypoint)
        
    def _get_wpt(self, wp):
        for i in self.tmap.waypoints:
            if i.name == wp:
                return i

        
    def _get_wp(self, wp):
        for i in self.targets:
            if i.name == wp:
                return i
    
    def _find_wp(self, wpname):
        for i in self.tmap.waypoints:
            if i.name == wpname:
                return i
    
    def _find_in_route(self,wp_name):
        found =False
        for i in self.route:
            if i.name == wp_name:
                found=True
                return found
        
        for i in self.explored_wp:
            if i.name == wp_name:
                found=True
                return found

        for i in self.targets:
            if i.name == wp_name:
                found=True
                return found
        
        return found
    
    def add_greedy_goal(self, model_variance):
        print "getting Greedy..."
        #x ,y =np.where(model_variance==np.max(model_variance))
        #print x,y
        
        found = False
        maxes1= np.copy(model_variance)
        #print maxes1
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
        print "Going To:"
        print to_append
        return to_append
        #self.route.append(to_append) 


    def add_limited_greedy_goal(self, model_variance, current_coord):
        print "getting Greedy..."
        #x ,y =np.where(model_variance==np.max(model_variance))
        #print x,y
        
        found = False
        maxes1= np.copy(model_variance)
        #print maxes1
        maxes = np.reshape(maxes1, -1)
        #print "-------"
        maxes.sort()
        maxes = maxes[::-1]
        #print maxes
        maxind=0
        while not found:
            x ,y = np.where(model_variance==maxes[maxind])
            print "looking for: ", maxes[maxind], " (", x[0],",",y[0],")"
            for j in self.tmap.waypoints:
                if (x[0],y[0]) == j.ind:
                    dist = j.coord - current_coord
                    if not self._find_in_route(j.name) and np.abs(dist[0]) < 48 :
                        to_append=j                        
                        found=True
                        break
            maxind+=1
        print "Going To:"
        print to_append

        return to_append
        #self.route.append(to_append) 



    def adaptive_goals(self, model_variance, mode='greedy'):
        ini_p=self.route[0].name
        print("current node", ini_p)
        self.targets=[]
        orig_len=len(self.route)
        for i in self.route:
            self.targets.append(i)
        print("clearing route")
        self.route =[]
        print("removing low variance goals")
        var_average= np.mean(model_variance)
        var_std= np.std(model_variance)
        var_cut=var_average-(0.2*var_std)
        #removed=0
        to_pop=[]
        for i in range(len(self.targets)):
            print i, model_variance.shape, len(self.targets), self.targets[i].ind[0],self.targets[i].ind[1]
            if model_variance[self.targets[i].ind[0]][self.targets[i].ind[1]]<var_cut:
                to_pop.append(i)
                #removed=removed+1

        to_pop.reverse()
        for i in to_pop:
            self.targets.pop(i)

        preadd=len(self.targets)
        to_add=orig_len-preadd
        for i in range(to_add+1):
            #if mode=='greedy':
            #self.targets.append(self.add_greedy_goal(model_variance))
            #else:
            self.targets.append(self.add_montecarlo_goal(model_variance))

        print "Val Cut: ", var_cut, " av: ", var_average, " std ", var_std
        print "TARGETS: ",len(self.targets), " Removed ", len(to_pop), " from ", orig_len, " there were ", preadd
        
        self._get_plan(ini_p)


    def add_montecarlo_goal(self, model_variance):
        print "getting Greedy..."
        
        found = False
        maxes1= np.copy(model_variance)
        #print maxes1
        maxes = np.reshape(maxes1, -1)
        #print "-------"
        maxes.sort()
        maxes = maxes[::-1]
        #print maxes
        maxind=0
        
        var_average= np.mean(model_variance)
        #var_std= np.std(model_variance)
        
        var_cut=var_average#-(var_std)
        varis=[]
        wps=[]
        
        while not found:
            x ,y = np.where(model_variance==maxes[maxind])
            #print "looking for: ", maxes[maxind], " (", x[0],",",y[0],")"
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
        print scores, var_cut
        mctargets=[]
        
        for i in range(len(varis)):
            for j in range(scores[i]):
                mctargets.append(wps[i])
                
        #print mctargets, len(mctargets)
        np.random.shuffle(mctargets)
        inds=np.random.randint(len(mctargets))
        
        return mctargets[inds]
        #self.route.append(mctargets[inds])



    def add_limited_montecarlo_goal(self, model_variance, current_coord):
        print "getting Greedy..."
        
        found = False
        maxes1= np.copy(model_variance)
        #print maxes1
        maxes = np.reshape(maxes1, -1)
        #print "-------"
        maxes.sort()
        maxes = maxes[::-1]
        #print maxes
        maxind=0
        
        var_average= np.mean(model_variance)
        #var_std= np.std(model_variance)
        
        var_cut=var_average#-(var_std)
        varis=[]
        wps=[]
        
        while not found:
            x ,y = np.where(model_variance==maxes[maxind])
            #print "looking for: ", maxes[maxind], " (", x[0],",",y[0],")"
            for j in self.tmap.waypoints:
                if (x[0],y[0]) == j.ind:
                    dist = j.coord - current_coord
                    if not self._find_in_route(j.name) and np.abs(dist[0]) < 50 :
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
        print varis, scores, var_cut
        mctargets=[]
        
        for i in range(len(varis)):
            for j in range(scores[i]):
                mctargets.append(wps[i])
                
        #print mctargets, len(mctargets)
        np.random.shuffle(mctargets)
        inds=np.random.randint(len(mctargets))
        
        return mctargets[inds]
        #self.route.append(mctargets[inds])




    def get_greedy_initial_goals(self, initial_waypoint):
        names=[]
        print "getting greedy init"
        iwp = self._find_wp(initial_waypoint)
        print iwp
        self.targets.append(iwp)
        names.append(iwp.name)
        print("finding 4 candidates in range")
        drange= np.sqrt(50)*5
        ntargs=1
        while ntargs<4:
            nind=np.random.randint(0, len(self.tmap.waypoints))
            pwp=self.tmap.waypoints[nind]
            dist= pwp.coord - iwp.coord
            if np.abs(dist[0]) < drange and pwp.name not in names:
                ntargs+=1
                self.targets.append(pwp)
                names.append(pwp.name)
                
    
    
    def _get_ac_targets(self, topo_map, initial_waypoint, ac_coords):
        
        for i in ac_coords:
            for j in topo_map.waypoints:
                if i == j.ind:
                    self.targets.append(j)
                    #break
                    
        found = False            
        for i in self.targets:
            if i.name == initial_waypoint:
                found=True
                break
        
        if not found:
            for i in topo_map.waypoints:
                if i.name == initial_waypoint:
                    self.targets.append(i)
                    break
    
    def _generate_targets(self, topo_map, initial_waypoint, percentage):
        ntargets = int(np.ceil(len(topo_map.waypoints))*percentage)

        N = 0
        for item in topo_map.waypoints:
            N += 1
            if len( self.targets ) < ntargets:
                self.targets.append(item)
            else:
                s = int(random.random() * N)
                if s < ntargets:
                    self.targets[ s ] = item

        found = False            
        for i in self.targets:
            if i.name == initial_waypoint:
                found=True
                break
        
        if not found:
            for i in topo_map.waypoints:
                if i.name == initial_waypoint:
                    self.targets.append(i)
                    break
        
    def _get_plan(self, initial_waypoint):
#        self.route.append(self._get_wp(initial_waypoint))
#        self.route_nodes.append(initial_waypoint)
        #print initial_waypoint
        groute, gdist = self._create_greedy_plan(initial_waypoint)
        rroute, rdist = self._create_random_plan(initial_waypoint)
        
        if gdist < rdist:
            route = groute
            dist = gdist
        else:
            route = rroute
            dist = rdist
        
        
        
        #route, dist = self.optimise_route(route)
        self.route = route
        self.route.pop(0)

        troute, tdist = self._get_tsp_plan(initial_waypoint)
        
        if tdist<dist:
            self.route = troute
        
        print "Greedy: " + str(gdist) + " Random: " + str(rdist) + " Optimised: " + str(dist) + " Tom: " + str (tdist)
        #print self.route

        
    
    def _create_greedy_plan(self, initial_waypoint):
        route=[]
        route.append(self._get_wpt(initial_waypoint))
        
        local_targs = self.targets[:]
        
        print "-----------"
        #print route[-1]
        while len(local_targs) > 0:
            cn = route[-1]
            min_dist =10000.0
            min_ind = -1
            for i in range(0, len(local_targs)):
                print local_targs[i]
                print cn
                dist, ang = local_targs[i].coord - cn.coord
                if dist <= min_dist:
                   min_dist = dist
                   min_ind = i
            route.append(local_targs[min_ind])
            #self.route_nodes.append(local_targs[min_ind].name)
            local_targs.pop(min_ind)
        
        #print [x.name for x in route]
        dist = self.get_route_dist(route)
        
        #print "TOTAL DIST: " + str(dist)
        return route, dist
            
    def get_route_dist(self, route):
        dist = 0
        for i in range(1, len(route)):
            d = route[i].coord - route[i-1].coord
            dist+=d[0]
        return dist
    
    
    def optimise_route(self, route):
        iters=0
        min_route=route[:]
        min_dist = self.get_route_dist(min_route)
        while iters < self.max_iters:
            swp = self.find_intersections(route)
            if swp:
                #route[swp[0]], route[swp[1]] = route[swp[1]], route[swp[0]]
                i=0
                while i<len(swp):
                    route[swp[i]], route[swp[i+1]] = route[swp[i+1]], route[swp[i]]
                    i+=2
                dist = self.get_route_dist(route)
                if dist < min_dist:
                    min_route=route[:]
                    min_dist=dist
            else:
                dist = self.get_route_dist(route)
                print "no more crossings", dist
                break
            iters+=1
            print "optimising iter: ",iters
        return min_route, min_dist
            
        
    
    def find_intersections(self, route):
        swp=[]
        ind = 1
        jnd = 0
        while jnd < (len(route)-1):
            line1=(route[jnd],route[jnd+1])
            ind = jnd +2
            while ind < (len(route)-1):
                line2=(route[ind], route[ind+1])
                res = line_intersection(line1, line2)
                #print len(route), ": ", jnd, "->",jnd+1, " : ",ind,ind+1
                if res[0]:
                    #print "intersecion between: ", line1[0].name,"->",line1[1].name, " and ", line2[0].name,"->",line2[1].name
                    #print "swap: ", jnd+1, "with:", ind
                    swp.append(jnd+1)
                    swp.append(ind)                    
                    #return jnd+1, ind
                ind+=1
            jnd+=1
        return swp
            
            
    def _get_tsp_plan(self, initial_waypoint):
        route=[]
        coordsx=[]
        coordsy=[]

        for i in self.targets:
            if i.name != initial_waypoint:
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
            get_tsp = rospy.ServiceProxy('get_tsp', GetTsp)
            resp1 = get_tsp(coordsx, coordsy)
            #print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        plt.plot(resp1.x, resp1.y, '-o')
        plt.savefig('poo.png')
        
        #print "Remember initial wP: ", initial_waypoint
        for i in range(len(resp1.x)):
            for j in self.targets:
                if (resp1.x[i],resp1.y[i]) == j.ind:
                    route.append(j)
                    #print j.name
                    break
        return route, self.get_route_dist(route)
        
    
    def _create_random_plan(self, initial_waypoint):
        route=[]
        route.append(self._get_wpt(initial_waypoint))
        
        for i in self.targets:
            if i.name != initial_waypoint:
                route.append(i)
        
        dist = self.get_route_dist(route)
        
        #print "TOTAL DIST: " + str(dist)
        return route, dist