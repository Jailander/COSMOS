#!/usr/bin/env python


import cv2
import sys
import yaml

import signal
import numpy as np
#import utm


import matplotlib as mpl
import matplotlib.cm as cm

import rospy

import argparse

import actionlib


from cosmos_msgs.msg import KrigInfo
from cosmos_msgs.srv import CompareModels
import kriging_exploration.map_coords
import std_msgs.msg

import open_nav.msg

from kriging_exploration.data_grid import DataGrid
from kriging_exploration.map_coords import MapCoords
from kriging_exploration.visualiser import KrigingVisualiser
from kriging_exploration.canvas import ViewerCanvas
from kriging_exploration.topological_map import TopoMap
from kriging_exploration.exploration import ExplorationPlan

from sensor_msgs.msg import NavSatFix


def overlay_image_alpha(img, img_overlay):
    """Overlay img_overlay on top of img at the position specified by
    pos and blend using alpha_mask.
    """
    show_image = img.copy()
    alpha =  img_overlay[:, :, 3] / 255.0   # Alpha mask must contain values 
                                            # within the range [0, 1] 
                                            # and be the same size as img_overlay.
    # Image ranges
    y1, y2 = 0, img.shape[0]
    x1, x2 = 0, img.shape[1]

    channels = img.shape[2]

    alpha_inv = 1.0 - alpha

    for c in range(channels):
        show_image[y1:y2, x1:x2, c] = (alpha * img_overlay[y1:y2, x1:x2, c] + alpha_inv * img[y1:y2, x1:x2, c])
    return show_image


class Explorator(KrigingVisualiser):

    #_w_shape=[(0, 16), (1, 17), (3, 17), (5, 16), (8, 15), (10, 15), (12, 14), (14, 13), (12, 12), (10, 11), (8, 11), (5, 10), (8, 9), (10, 9), (12, 8), (14, 7), (12, 6), (10, 5), (8, 5), (6, 4), (4, 3), (3, 2), (4, 1), (5, 0), (7, 0)]
    #_w_shape=[(17, 0), (17, 1), (17, 3), (16, 5), (15, 8), (15, 10), (14, 12), (13, 14), (12, 12), (11, 10), (11, 8), (10, 5), (9, 8), (9, 10), (8, 12), (7, 14), (6, 12), (5, 10), (5, 8), (4, 6), (3, 4), (2, 3), (1, 4), (0, 5), (0, 7)]
    #_w_shape=[(17, 0), (17,1), (17, 2), (17, 4), (16, 4), (16, 6), (16, 8), (15, 8), (15, 10), (14, 10), (14, 12), (13, 12), (13, 14), (12, 14), (12, 12), (11, 12), (11, 10), (10, 10), (10, 8), (10, 6), (10, 4), (9, 4), (9, 6), (9, 8), (9, 10), (8, 10), (8, 12), (7, 12), (7, 14), (6, 14), (6, 12), (5, 12), (5, 10), (4, 10), (4, 8), (4, 6), (4, 4), (3, 4), (3, 3), (2, 3), (2, 4), (1,4), (1, 6), (0,6), (1, 8), (0,8), (1, 10), (0, 10), (0, 12), (0, 14)]
    #_w_shape=[(17, 0), (16, 1), (14, 6), (12, 11), (10, 14), (8, 9), (5, 14), (3, 11), (2, 6), (0, 3)]
    
    #30    
    #_w_shape=[( 34 , 0),( 33 , 2 ),( 32 , 4 ),( 31 , 6 ),( 30 , 8 ),( 29 , 10 ),( 28 , 12 ),( 27 , 14 ),( 25 , 16 ),( 24 , 19 ),( 23 , 22 ),( 22 , 25 ),( 21 , 27 ),( 20 , 29 ),( 19 , 27 ),( 18 , 25 ),( 17 , 22 ),( 16 , 19 ),( 15 , 16 ),( 14 , 19 ),( 13 , 22 ),( 12 , 25 ),( 11 , 27 ),( 10 , 29 ),( 9 , 27 ),( 8 , 25 ),( 7 , 22 ),( 6 , 18 ),( 5 , 15 ),( 4 , 13 ),( 3 , 11 )]
    #20
    #_w_shape=[( 34 , 0),( 32 , 4 ),( 30 , 8 ),( 28 , 12 ),( 25 , 16 ),( 23 , 22 ),( 22 , 25 ),( 21 , 27 ),( 20 , 29 ),( 19 , 27 ),( 18 , 25 ),( 17 , 22 ),( 16 , 19 ),( 15 , 16 ),( 14 , 19 ),( 13 , 22 ),( 12 , 25 ),( 11 , 27 ),( 9 , 27 ),( 7 , 22 ),( 5 , 15 ),( 3 , 11 )]
    _w_shape=[( 34 , 0),( 32 , 4 ),( 30 , 8 ),( 28 , 12 ),( 25 , 16 ),( 22 , 25 ),( 20 , 29 ),( 18 , 25 ),( 16 , 19 ),( 15 , 16 ),( 14 , 19 ),( 13 , 22 ),( 12 , 25 ),( 11 , 27 ),( 9 , 27 ),( 7 , 22 ),( 3 , 11 )]
    def __init__(self, lat_deg, lon_deg, zoom, size, args):
        self.targets = []
        self.results =[]
        self.results2 =[]
        self.result_counter=0
        self.explodist=0
        self.running = True
        self.last_coord=None
        signal.signal(signal.SIGINT, self.signal_handler)
        self.expid=args.experiment_name
        print "Creating visualiser object"
        super(Explorator, self).__init__(lat_deg, lon_deg, zoom, size)

        cv2.namedWindow('explorator')
        cv2.setMouseCallback('explorator', self.click_callback)

        self.current_model=-1
        self.draw_mode = 'none'
        self.grid = DataGrid(args.limits_file, args.cell_size)
        #self.gtgrid = DataGrid(args.limits_file, args.cell_size)
        
        self.topo_map= TopoMap(self.grid)
        self.visited_wp=[]

        explo_type = args.area_coverage_type
        self.define_exploration_type(explo_type)
        
            
        self.navigating = False
        self.pause_exp = False
        self.exploring = 0
        self.n_inputs = 0
        
        print "NUMBER OF TARGETS:"
        print len(self.explo_plan.targets)        
        
        self.limits_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.grid_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.exploration_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.gps_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)

        self.limits_canvas.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
        self.grid_canvas.draw_grid(self.grid.cells, args.cell_size, (128,128,128,2), thickness=1)
        
        self.redraw()

        self.redraw_kriged=True
        self.redraw_var=True
        self.redraw_devi=True
        
        self.model_canvas=[]
        self.model_legend=[]
        self.kriging_canvas=[]
        self.klegend_canvas=[]
        self.klegend2_canvas=[]
        self.klegend3_canvas=[]
        self.sigma_canvas=[]
        self.sigma2_canvas=[]
        self.model_canvas_names=[]
        
        self.mean_out_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.mean_out_legend_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.mean_var_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.mean_var_legend_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.mean_dev_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.mean_dev_legend_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)        

        rospy.loginfo("Subscribing to Krig Info")
        rospy.Subscriber("/kriging_data", KrigInfo, self.data_callback)
        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber('/penetrometer_scan', std_msgs.msg.String, self.scan_callback)
        self.req_data_pub = rospy.Publisher('/request_scan', std_msgs.msg.String, latch=False, queue_size=1)

        rospy.loginfo(" ... Connecting to Open_nav")
        
        self.open_nav_client = actionlib.SimpleActionClient('/open_nav', open_nav.msg.OpenNavAction)
        self.open_nav_client.wait_for_server()

        rospy.loginfo(" ... done")


        tim1 = rospy.Timer(rospy.Duration(0.2), self.drawing_timer_callback)
        tim2 = rospy.Timer(rospy.Duration(0.1), self.control_timer_callback)
        self.refresh()

        while(self.running):
            cv2.imshow('explorator', self.show_image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)

        tim1.shutdown()
        tim2.shutdown()
        cv2.destroyAllWindows()       
        sys.exit(0)


    # EXPLORATION PARAMS HERE!!!!
    def define_exploration_type(self, explo_type):
        self.exploration_strategy=explo_type    
        self.n_goals=50   
        
        if explo_type=='area_split' or explo_type=='as_greedy':
            #self.grid._split_area(7,7)
            self.grid._split_area(3,3)
            sb=[]
            for i in self.grid.area_splits_coords:
                (y, x) = self.grid.get_cell_inds_from_coords(i)
                sb.append((x,y))
            self.explo_plan = ExplorationPlan(self.topo_map, args.initial_waypoint, args.initial_percent, ac_model=explo_type, ac_coords=sb)
        elif explo_type=='random' or explo_type=='random_greedy':
            #print "no Exploplan for now"            
            self.explo_plan = ExplorationPlan(self.topo_map, args.initial_waypoint, args.initial_percent)
        elif explo_type=='w_shape':
            self.explo_plan = ExplorationPlan(self.topo_map, args.initial_waypoint, args.initial_percent, ac_model=explo_type, ac_coords=self._w_shape)
        else: #greedy
            self.explo_plan = ExplorationPlan(self.topo_map, args.initial_waypoint, args.initial_percent, exploration_type='greedy', ac_model=explo_type)




    def drawing_timer_callback(self, event):
        self.refresh()

    def control_timer_callback(self, event):
        if self.navigating:
            if self.open_nav_client.simple_state ==2:
                print "DONE NAVIGATING"
                self.navigating = False
                if self.exploring==1:
                    self.exploring=2
        
        elif self.exploring==2:
            if not self.pause_exp:
                self.explo_plan.explored_wp.append(self.explo_plan.route.pop(0))
                info_str='Do_reading'
                self.req_data_pub.publish(info_str)
                self.exploring=3
        
        elif self.exploring==4:
            if not self.pause_exp:
                if len(self.explo_plan.route) >0:
                    gg=self.explo_plan.route[0]
                    self.open_nav_client.cancel_goal()
                    targ = open_nav.msg.OpenNavActionGoal()
        
                    targ.goal.coords.header.stamp=rospy.Time.now()
                    targ.goal.coords.latitude=gg.coord.lat
                    targ.goal.coords.longitude=gg.coord.lon
        
                    print "Going TO: ", gg.name
                    self.exploring=1
                    self.navigating=True
                    self.open_nav_client.send_goal(targ.goal)
                else:
                    print "Done Exploring"
                    self.exploring = 0
#        else:
#            if self.exploring:
#                print "waiting for new goal"
            
    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            self.gps_canvas.clear_image()
            gps_coord = MapCoords(data.latitude,data.longitude)            
            self.gps_canvas.draw_coordinate(gps_coord,'black',size=2, thickness=2, alpha=255)
            if self.last_coord:
                dist = gps_coord - self.last_coord
                self.explodist+= dist[0]
            self.last_coord=gps_coord



    def data_callback(self, msg):
        point_coord = kriging_exploration.map_coords.coord_from_satnav_fix(msg.coordinates)
        for i in msg.data:
            self.grid.add_data_point(i.model_name, point_coord, i.measurement)

        self.vmin, self.vmax = self.grid.get_max_min_vals()
        self.n_models=len(self.grid.models)
        
        for i in self.grid.models:
            if i.name not in self.model_canvas_names:       
                #print i.name
                self.model_canvas_names.append(i.name)
                self.model_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.model_legend.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.kriging_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.klegend_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.klegend2_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.klegend3_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))               
                self.sigma_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.sigma2_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
            self.draw_inputs(self.model_canvas_names.index(i.name))

        self.n_inputs+=1
        if self.exploring==3:
            if self.n_inputs>3:
                self.krieg_all_mmodels()
                rospy.sleep(0.1)
                self.grid.calculate_mean_grid()
                rospy.sleep(0.1)
                self.draw_means()
                self.draw_mode="means"
                
                resp = self.get_errors()
                self.result_counter+=1
                              
                
                d={}
                d['step']=self.result_counter
                d['id']=self.expid
                d['ns']=len(self.explo_plan.targets)
                d['coord']={}
                d['coord']['lat']=self.last_coord.lat
                d['coord']['lon']=self.last_coord.lon
                d['dist']=float(self.explodist)
                d['results']={}
                d['results']['groundtruth']=resp
                d['results']['var']={}
                d['results']['var']['mean']={}
                d['results']['var']['mean']['mean']= float(np.mean(self.grid.mean_variance))
                d['results']['var']['mean']['max']= float(np.max(self.grid.mean_variance))
                d['results']['var']['mean']['min']= float(np.min(self.grid.mean_variance))
                
#                d['results']['var']['std']['mean']= np.mean(self.grid.mean_deviation)
#                d['results']['var']['std']['max']= np.max(self.grid.mean_deviation)
#                d['results']['var']['std']['min']= np.min(self.grid.mean_deviation)

                means=[]
                maxs=[]
                mins=[]

                e={}
                e['step']=self.result_counter
                e['results']=[]
                for i in range(self.n_models):
                    f={}
                    means.append(float(np.mean(self.grid.models[i].variance)))
                    maxs.append(float(np.max(self.grid.models[i].variance)))
                    mins.append(float(np.min(self.grid.models[i].variance)))
                    f['name']=self.grid.models[i].name
                    f['params']={}
                    f['params']['Model']= self.grid.models[i].variogram_model
                    if self.grid.models[i].variogram_model != 'linear':
                        f['params']['Sill']= float(self.grid.models[i].Sill)
                        f['params']['Range']= float(self.grid.models[i].Range)
                        f['params']['Nugget']= float(self.grid.models[i].Nugget)
                    else:
                        print "Sill: ", float(self.grid.models[i].Sill)
                        print "Range: ", float(self.grid.models[i].Range)
                        f['params']['Sill']= float(self.grid.models[i].sim_sill)
                        f['params']['Nugget']= float(self.grid.models[i].Sill)
                        if self.grid.models[i].Range <= 0.1:
                            print "RANGE 0"
                            f['params']['Range']= 0
                        else:
                            f['params']['Range']= (f['params']["Sill"]-f['params']["Nugget"])/float(self.grid.models[i].Range)
                    f['results']={}
                    f['results']['kriging_variance']=float(np.mean(self.grid.models[i].variance))
                    e['results'].append(f)
                    
                
                d['results']['models']={}
                d['results']['models']['means']=means
                d['results']['models']['maxs']=maxs
                d['results']['models']['mins']=mins






                rospy.sleep(0.1)
                self.results.append(d)
                self.results2.append(e)
                if self.exploration_strategy == 'greedy':
                    nwp = len(self.explo_plan.route) + len(self.explo_plan.explored_wp)
                    print nwp, " nodes in plan"
                    if nwp <= self.n_goals:
                        #THIS IS the ONE
                        #ng = self.explo_plan.add_limited_greedy_goal(self.grid.mean_variance, self.last_coord)
                        #self.explo_plan.route.append(ng)
                                        
                        #ng = self.explo_plan.add_greedy_goal(self.grid.mean_variance)
                        #self.explo_plan.route.append(ng)
                                               
                        ng=self.explo_plan.add_montecarlo_goal(self.grid.mean_variance)
                        #ng=self.explo_plan.add_limited_montecarlo_goal(self.grid.mean_variance, self.last_coord)
                        
                        self.explo_plan.route.append(ng)
                        
                elif self.exploration_strategy == 'random_greedy' or  self.exploration_strategy == 'as_greedy':
                    print "ADAPTIVE!!!"
                    nwp = len(self.explo_plan.route) + len(self.explo_plan.explored_wp)
                    print nwp, " nodes in plan"
                    
                    #if nwp <= self.n_goals:
                        #THIS IS the ONE
                        #self.explo_plan.add_limited_greedy_goal(self.grid.mean_variance, self.last_coord) 
                    
                    self.explo_plan.adaptive_goals(self.grid.mean_variance)
                    self.exploration_canvas.clear_image()
                    self.exploration_canvas.draw_waypoints(self.explo_plan.targets, (255,200,128,255), thickness=3)
                    self.exploration_canvas.draw_plan(self.explo_plan.route, 'cyan', thickness=1)
                    self.redraw()
                    
                    nwp = len(self.explo_plan.route) + len(self.explo_plan.explored_wp)
                    print nwp, " nodes in plan"
                #self.draw_mode="deviation"
#                self.current_model=0
#                if self.redraw_devi:
#                    self.draw_all_devs()
                self.redraw()
                rospy.sleep(0.1)
                
            if len(self.explo_plan.explored_wp) > self.n_goals:
                print "Done Exploring?"
                self.exploring = 4
            else:
                self.exploring=4



    def scan_callback(self, msg):
        if msg.data == 'Reading':
            #print "GOT READING!!!"
            cx, cy = self.grid.get_cell_inds_from_coords(self.last_coord)
            if cx <0 or cy<0:
                print "Reading outside the grid"
            else:
                #print 'Reading at: ', cx, cy
                for i in self.topo_map.waypoints:
                    if (cy,cx) == i.ind:
                        #print 'Setting: ', i.name, i.coord, "as Visited"
                        i.visited= True
                        self.visited_wp.append(i)
                        self.grid_canvas.draw_waypoints(self.topo_map.waypoints, (0,255,0,2), thickness=1)
                        self.grid_canvas.draw_waypoints(self.visited_wp, (0,0,255,2), thickness=1)
                        self.redraw()


    def refresh(self):
        #self.show_image = self.image.copy()
        #self.show_image = cv2.addWeighted(self.gps_canvas.image, 0.7, self.image, 1.0, 0)
        #self.show_image = transparentOverlay(self.image, self.gps_canvas.image)
        self.show_image = overlay_image_alpha(self.image,self.gps_canvas.image)

    def redraw(self):
        self.image = cv2.addWeighted(self.grid_canvas.image, 0.5, self.base_image, 1.0, 0)
        self.image = cv2.addWeighted(self.limits_canvas.image, 0.75, self.image, 1.0, 0)
        self.image = cv2.addWeighted(self.exploration_canvas.image, 0.75, self.image, 1.0, 0)
        if self.draw_mode == "inputs" and self.current_model>=0 :
            self.image = cv2.addWeighted(self.model_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)
            self.image = overlay_image_alpha(self.image, self.model_legend[self.current_model].image)

        if self.draw_mode == "kriging":# and self.current_model>=0 :
            self.image = cv2.addWeighted(self.kriging_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)
            #self.image = cv2.addWeighted(self.klegend_canvas[self.current_model].image, 1.0, self.image, 1.0, 0)
            self.image = overlay_image_alpha(self.image, self.klegend_canvas[self.current_model].image)

        if self.draw_mode == "deviation":# and self.current_model>=0 :
            self.image = cv2.addWeighted(self.sigma_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)
            #self.image = cv2.addWeighted(self.klegend3_canvas[self.current_model].image, 1.0, self.image, 1.0, 0)
            self.image = overlay_image_alpha(self.image, self.klegend3_canvas[self.current_model].image)
            
        if self.draw_mode == "variance":# and self.current_model>=0 :    
            self.image = cv2.addWeighted(self.sigma2_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)
            #self.image = cv2.addWeighted(self.klegend2_canvas[self.current_model].image, 1.0, self.image, 1.0, 0)
            self.image = overlay_image_alpha(self.image, self.klegend2_canvas[self.current_model].image)
        
        if self.draw_mode == "means":
            self.image = cv2.addWeighted(self.mean_dev_canvas.image, 0.75, self.image, 1.0, 0)
            #self.image = cv2.addWeighted(self.klegend2_canvas[self.current_model].image, 1.0, self.image, 1.0, 0)
            self.image = overlay_image_alpha(self.image, self.mean_dev_legend_canvas.image)
            
        
        self.show_image = self.image.copy()

            

    def click_callback(self, event, x, y, flags, param):
        
        if event == cv2.EVENT_RBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            cx, cy = self.grid.get_cell_inds_from_coords(click_coord)

            if cx <0 or cy<0:
                print "click outside the grid"
            else:
                print "(",cy,",", cx,")"
                
            for i in self.topo_map.waypoints:
                if (cy,cx) == i.ind:
                    print i.name, i.coord.easting, i.coord.northing
                    i.visited= True
                    self.visited_wp.append(i)
                    self.grid_canvas.draw_waypoints(self.topo_map.waypoints, (0,255,0,2), thickness=1)
                    self.grid_canvas.draw_waypoints(self.visited_wp, (0,0,255,2), thickness=1)
                    self.redraw()

        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            cx, cy = self.grid.get_cell_inds_from_coords(click_coord)

            if cx <0 or cy<0:
                print "click outside the grid"
            else:
                print cx, cy

            for i in self.topo_map.waypoints:
                if (cy,cx) == i.ind:
                    self.open_nav_client.cancel_goal()
                    targ = open_nav.msg.OpenNavActionGoal()

                    #goal.goal.goal.header.
                    targ.goal.coords.header.stamp=rospy.Time.now()
                    targ.goal.coords.latitude=i.coord.lat
                    targ.goal.coords.longitude=i.coord.lon

                    print targ
                    self.navigating=True
                    self.open_nav_client.send_goal(targ.goal)
                    #self.client.wait_for_result()
                    # Prints out the result of executing the action
                    #ps = self.client.get_result()
                    #print ps
                


    def draw_inputs(self, nm):
        
        minv =  self.grid.models[nm].lims[0]
        maxv =  self.grid.models[nm].lims[1]

        if (maxv-minv) <=1:
            maxv = maxv + 50
            minv = minv - 50
            
        norm = mpl.colors.Normalize(vmin=minv, vmax=maxv)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.model_canvas[nm].clear_image()
        self.model_legend[nm].clear_image()
        
        for i in self.grid.models[nm].orig_data:
            cell = self.grid.cells[i.y][i.x]
            a= colmap.to_rgba(int(i.value))                
            b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
            self.model_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)
            self.model_canvas[nm].put_text(self.grid.models[nm].name)
        
        self.model_legend[nm].put_text(self.grid.models[nm].name)
        self.model_legend[nm].draw_legend(minv, maxv, colmap, title="Kriging")
        



    def draw_krigged(self, nm):
        print "drawing kriging" + str(nm)

        minv =  self.grid.models[nm].min_val
        maxv =  self.grid.models[nm].max_val

        if (maxv-minv) <=1:
            maxv = maxv + 50
            minv = minv - 50

        norm = mpl.colors.Normalize(vmin=minv, vmax=maxv)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)


        self.kriging_canvas[nm].clear_image()
        self.klegend_canvas[nm].clear_image()
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].output[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))    
                self.kriging_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)
        
        self.klegend_canvas[nm].put_text(self.grid.models[nm].name)
        self.klegend_canvas[nm].draw_legend(minv, maxv, colmap, title="Kriging")
        
        self.redraw()


    def draw_variance(self, nm):
        print "drawing variance" + str(nm)
        
        minv =  self.grid.models[nm].min_var
        maxv =  self.grid.models[nm].max_var
        
        if (maxv-minv) <=1:
            maxv = maxv + 50
            minv = minv - 50
        
        norm = mpl.colors.Normalize(vmin=minv, vmax= maxv)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.sigma_canvas[nm].clear_image()
        self.klegend2_canvas[nm].clear_image()
        
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].variance[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.sigma2_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)


        self.klegend2_canvas[nm].put_text(self.grid.models[nm].name)
        self.klegend2_canvas[nm].draw_legend(minv, maxv, colmap, title="Variance")
        self.redraw()




    def draw_means(self):
        self.draw_mean_vari()


    def draw_mean_out(self):
        print "drawing mean output ..."
        
        minv =  self.grid.min_mean_output
        maxv =  self.grid.max_mean_output

        if (maxv-minv) <=1:
            maxv = maxv + 50
            minv = minv - 50        
        
        
        norm = mpl.colors.Normalize(vmin=minv, vmax=maxv)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.mean_dev_canvas.clear_image()
        self.mean_dev_legend_canvas.clear_image()
        
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.mean_output[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.mean_dev_canvas.draw_cell(cell, self.grid.cell_size, b, thickness=-1)


        #self.mean_dev_legend_canvas.put_text(self.grid.models[nm].name)
        self.mean_dev_legend_canvas.draw_legend(minv, maxv, colmap, title="Mean Output")
        
        #self.draw_mode="means"
        self.redraw()        
        
        

    def draw_mean_vari(self):
        print "drawing mean variance ..."
        
        minv =  self.grid.min_mean_variance
        maxv =  self.grid.max_mean_variance

        if (maxv-minv) <=1:
            maxv = maxv + 50
            minv = minv - 50        
        
        
        norm = mpl.colors.Normalize(vmin=minv, vmax=maxv)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.mean_dev_canvas.clear_image()
        self.mean_dev_legend_canvas.clear_image()
        
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.mean_variance[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.mean_dev_canvas.draw_cell(cell, self.grid.cell_size, b, thickness=-1)


        #self.mean_dev_legend_canvas.put_text(self.grid.models[nm].name)
        self.mean_dev_legend_canvas.draw_legend(minv, maxv, colmap, title="Mean Variance")
        
        #self.draw_mode="means"
        self.redraw()
        
    def draw_mean_devi(self):
        print "drawing mean deviation ..."
        
        minv =  self.grid.min_mean_deviation
        maxv =  self.grid.max_mean_deviation

        if (maxv-minv) <=1:
            maxv = maxv + 50
            minv = minv - 50        
        
        
        norm = mpl.colors.Normalize(vmin=minv, vmax=maxv)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.mean_dev_canvas.clear_image()
        self.mean_dev_legend_canvas.clear_image()
        
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.mean_deviation[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.mean_dev_canvas.draw_cell(cell, self.grid.cell_size, b, thickness=-1)


        #self.mean_dev_legend_canvas.put_text(self.grid.models[nm].name)
        self.mean_dev_legend_canvas.draw_legend(minv, maxv, colmap, title="Mean Deviation")
        
        #self.draw_mode="means"
        self.redraw()



    def draw_deviation(self, nm):
        print "drawing deviation" + str(nm)
        
        minv =  self.grid.models[nm].min_dev
        maxv =  self.grid.models[nm].max_dev

        if (maxv-minv) <=1:
            maxv = maxv + 50
            minv = minv - 50        
        
        
        
        norm = mpl.colors.Normalize(vmin=minv, vmax=maxv)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.sigma_canvas[nm].clear_image()
        self.klegend3_canvas[nm].clear_image()
        
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].deviation[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.sigma_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)


        self.klegend3_canvas[nm].put_text(self.grid.models[nm].name)
        self.klegend3_canvas[nm].draw_legend(minv, maxv, colmap, title="Deviation")
        self.redraw()


    def krieg_all_mmodels(self):
        for i in self.grid.models:
            i.do_krigging()
            self.redraw_kriged=True
            self.redraw_var=True
            self.redraw_devi=True

    def draw_all_outputs(self):
        for i in self.grid.models:
            self.draw_krigged(self.model_canvas_names.index(i.name))
            self.redraw_kriged=False


    def draw_all_vars(self):
        for i in self.grid.models:
            self.draw_variance(self.model_canvas_names.index(i.name))
            self.redraw_var=False
    
    def draw_all_devs(self):
        for i in self.grid.models:
            self.draw_deviation(self.model_canvas_names.index(i.name))
            self.redraw_devi=False
        
    
    def _change_mode(self, k):
        if k == 27:
            self.running = False
        elif k == ord('q'):
            self.running = False
        elif k == ord('n'):
            print len(self.grid.models)
        elif k == ord('i'):
            if self.n_models > 0:
                self.draw_mode="inputs"
                self.current_model=0
                self.redraw()
        elif k == ord('d'):
            if self.n_models > 0:
                self.draw_mode="deviation"
                self.current_model=0
                if self.redraw_devi:
                    self.draw_all_devs()
                self.redraw()                
        elif k == ord('v'):
            if self.n_models > 0:
                self.draw_mode="variance"
                self.current_model=0
                if self.redraw_var:
                    self.draw_all_vars()
                self.redraw()
        elif k == ord('t'):
            self.krieg_all_mmodels()
            self.grid.calculate_mean_grid()
            if self.n_models > 0:
                self.draw_all_outputs()
                self.draw_mode="kriging"
                self.current_model=0
                self.redraw()

        elif k == ord('k'):
            if self.n_models > 0:
                self.draw_mode="kriging"
                self.current_model=0
                if self.redraw_kriged:
                    self.draw_all_outputs()
                self.redraw()

        elif k == ord('>'):
            self.current_model+=1
            if self.current_model >= self.n_models:
                self.current_model=0
            self.redraw()
        elif k == ord('<'):
            self.current_model-=1
            if self.current_model < 0:
                self.current_model=self.n_models-1
            self.redraw()
        elif k == ord('w'):
            self.grid_canvas.draw_waypoints(self.topo_map.waypoints, (0,255,0,2), thickness=1)
            self.grid_canvas.draw_waypoints(self.visited_wp, (0,0,255,2), thickness=1)
            self.redraw()
        elif k == ord('e'):
            self.exploration_canvas.draw_waypoints(self.explo_plan.targets, (255,200,128,255), thickness=3)
            self.exploration_canvas.draw_plan(self.explo_plan.route, 'cyan', thickness=1)
            self.redraw()
            #xnames = [x.name for x in self.explo_plan.route]
            #print xnames
        elif k == ord('g'):
            if len(self.explo_plan.route) >0:
                gg=self.explo_plan.route[0]
                self.open_nav_client.cancel_goal()
                targ = open_nav.msg.OpenNavActionGoal()
    
                targ.goal.coords.header.stamp=rospy.Time.now()
                targ.goal.coords.latitude=gg.coord.lat
                targ.goal.coords.longitude=gg.coord.lon
    
                print "Going TO: ", gg
                self.exploring=1
                self.navigating=True
                self.open_nav_client.send_goal(targ.goal)
                self.result_counter=0
                self.explodist=0
            else:
                print "Done Exploring"
                self.exploring = 0
        elif k == ord('y'):
            vwp = []
            for i in self.visited_wp:
                vwp.append(i.name)
            yml = yaml.safe_dump(vwp, default_flow_style=False)
            fh = open("visited.yaml", "w")
            s_output = str(yml)
            fh.write(s_output)
            fh.close          
        elif k == ord('l'):
            print "loading visited"
            
            with open("visited.yaml", 'r') as f:
                visited = yaml.load(f)
                for i in visited:
                    for l in self.topo_map.waypoints:
                        if i == l.name:
                            self.visited_wp.append(l)
                            break

        elif k == ord('a'):
            self.grid.calculate_mean_grid()
            self.draw_means()
            self.draw_mode="means"

        elif k == ord('p'):    
            self.pause_exp= not self.pause_exp
            
        elif k == ord('c'):
            print self.grid.limits
            print "Area: ", self.grid.calculate_area(self.grid.limits)
            print "Area of Area: ", self.grid.area.area_size
            colours=['magenta','cyan', 'grey','white','red','yellow','green','blue']
            
            nc=0
            for j in self.grid.area_splits:
                print j.area_size, j.centre
                self.limits_canvas.draw_coordinate(j.centre, 'crimson', size=3, thickness=2)
                for i in j.limit_lines:
                    #self.limits_canvas.draw_line(i, colours[nc], thickness=1)
                    self.limits_canvas.draw_line(i, 'black', thickness=1)
                if nc < len(colours)-1:
                    nc+=1
                else:
                    nc=0

            self.redraw()
            
        elif k== ord('r'):
            #diff = (self.grid.models[1].output - self.grid.models[0].output)
            #print np.mean(diff), np.std(diff), diff.dtype
            print self.get_errors()            

        elif k== ord('o'):
            #print self.results
            outfile = self.expid + '.yaml'
            #print self.data_out
            yml = yaml.safe_dump(self.results, default_flow_style=False)
            fh = open(outfile, "w")
            s_output = str(yml)
            #print s_output
            fh.write(s_output)
            fh.close()


            print self.results
            outfile = self.expid + '-parameters.yaml'
            #print self.data_out
            yml = yaml.safe_dump(self.results2, default_flow_style=False)
            fh = open(outfile, "w")
            s_output = str(yml)
            #print s_output
            fh.write(s_output)
            fh.close()



        elif k== ord('2'):
            self.draw_mean_out()
            self.draw_mode= "means"
            self.redraw()

        elif k== ord('1'):
            self.draw_mean_vari()
            self.draw_mode= "means"
            self.redraw()

        elif k== ord('0'):
            self.exploration_canvas.clear_image()
            self.redraw()
            
        elif k== ord('9'):            
            self.exploration_canvas.draw_waypoints(self.explo_plan.explored_wp, (255,200,255,255), thickness=3)
            self.exploration_canvas.draw_plan(self.explo_plan.explored_wp, 'white', thickness=2)
            self.redraw()

    def get_errors(self):
        error_chain=[]
        shapeo = self.grid.models[0].output.shape
        
        #print vals
        print "Waiting for Service"
        rospy.wait_for_service('/compare_model')
        compare_serv = rospy.ServiceProxy('/compare_model', CompareModels)
        
        for i in range(self.n_models):
            try:
                d={}
                #print "going for it ", i
                vals = np.reshape(self.grid.models[i].output, -1)
                resp1 = compare_serv('kriging', i, shapeo[0], shapeo[1], vals.tolist())
                d['name']= self.grid.models[i].name
                d['type']= 'kriging'
                d['errors']={}
                d['errors']['error']=resp1.error
                d['errors']['mse']=resp1.mse
                d['errors']['std']=resp1.std
                d['errors']['var']=resp1.var
                #print resp1
                error_chain.append(d)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e    
                
        try:
            d={}
            #print "Mean "
            vals = np.reshape(self.grid.mean_output, -1)
            resp1 = compare_serv('mean', 0, shapeo[0], shapeo[1], vals.tolist())
            #print self.grid.mean_output
            d['name']= 'mean'
            d['type']= 'mean'
            d['errors']={}
            d['errors']['error']=resp1.error
            d['errors']['mse']=resp1.mse
            d['errors']['std']=resp1.std
            d['errors']['var']=resp1.var

            #print resp1
            error_chain.append(d)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e        
        
        
        return error_chain
    
    def signal_handler(self, signal, frame):
        self.running = False
        print('You pressed Ctrl+C!')





if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    parser.add_argument("--initial_percent", type=float, default=0.05,
                        help="Percentage of cells to be explored on the initial plan") 
    parser.add_argument("--limits_file", type=str, default='limits.coords',
                        help="Percentage of cells to be explored on the initial plan")
    parser.add_argument("--initial_waypoint", type=str, default='WayPoint498',
                        help="Percentage of cells to be explored on the initial plan")
    parser.add_argument("--area_coverage_type", type=str, default='area_split',
                        help="Type of area coverage, random or area_split")
    parser.add_argument("--experiment_name", type=str, default='exp1',
                        help="Experiment ID")
    args = parser.parse_args()
    
    rospy.init_node('kriging_exploration')
    #Explorator(53.261685, -0.527158, 16, 640, args.cell_size)
    
    #Explorator(53.267213, -0.533420, 17, 640, args)  #Football Field
    Explorator(53.261576, -0.526648, 17, 640, args)  #Half cosmos field
    #Explorator(53.261685, -0.525158, 17, 640, args) #COSMOS Field

    
