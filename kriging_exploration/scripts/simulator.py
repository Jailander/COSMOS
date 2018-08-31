#!/usr/bin/env python

import signal
import yaml
import numpy as np

import cv2
import sys

import argparse

import matplotlib as mpl
import matplotlib.cm as cm

import rospy

import std_msgs

from cosmos_msgs.msg import KrigInfo
from cosmos_msgs.msg import KrigMsg
from cosmos_msgs.srv import CompareModels

#import satellite
from kriging_exploration.satellite import SatelliteImage
from kriging_exploration.map_coords import MapCoords
from kriging_exploration.data_grid import DataGrid
#from krigging_data import KriggingData
from sensor_msgs.msg import NavSatFix


def data_to_yaml(data_fn):
    mods = {}
#    mods['names']=['0.0 cm', '2.5 cm' ,'5.0 cm', '7.5 cm', '10.0 cm', '12.5 cm', '15.0 cm', '17.5 cm', '20.0 cm', '22.5 cm' ,'25.0 cm', '27.5 cm', '30.0 cm', '32.5 cm', '35.0 cm', '37.5 cm', '40.0 cm', '42.5 cm', '45.0 cm']
    mods['data']=[]
#    vals=[]
    print "open: " + data_fn
    f = open(data_fn, 'r')
    for line in f:
        line=line.strip('\n')
        a=line.split(';')
        val={}
        val['position']={'lat':float(a[0]), 'lon':float(a[1])}
        val['data']= [float(i) for i in a[2:]]
        mods['data'].append(val)
    
    print mods['data']
    yml = yaml.safe_dump(mods, default_flow_style=False)
    print yml
    filename = data_fn+'.yaml'
    fh = open(filename, "w")
    s_output = str(yml)
    fh.write(s_output)
    fh.close


class simulator(object):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }

    def __init__(self, lat_deg, lon_deg, zoom, size, cell_size):
        self.draw_mode = 'none'
        self.running = True
        self.current_model=-1
        self.n_models=0
        
        cv2.namedWindow('simulator')
        cv2.setMouseCallback('simulator', self.click_callback)
        
        signal.signal(signal.SIGINT, self.signal_handler)
        self.data_pub = rospy.Publisher('/kriging_data', KrigInfo, latch=False, queue_size=1)
        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber('/request_scan', std_msgs.msg.String, self.scan_callback)

        rospy.Service('/compare_model', CompareModels, self.model_comparison_cb)
        

        print "Loading Satellite Image"
        self.satellite = SatelliteImage(lat_deg, lon_deg, zoom, size)
#        self.grid = DataGrid('football-old.coords', cell_size)
#        self.grid = DataGrid('greece.coords', cell_size)
        self.grid = DataGrid('airfield.coords', cell_size)
#        self.grid = DataGrid('cosmos.coords', cell_size)

        #self.load_groundtruth('Iains2.yaml')
        #self.load_groundtruth('bottom_testing.yaml')
        #self.load_groundtruth('upper_testing.yaml')
#        self.load_groundtruth('testing-5cm-intervals.yaml')
#        self.load_groundtruth('football-corrected.yaml')        
#        self.load_groundtruth('cosmos-corrected.yaml')        
#        self.load_groundtruth('greece_gt.yaml')        
        self.load_groundtruth('airfield-sim.data')
                
                
        self.krieg_all_mmodels()
        self.grid.calculate_mean_grid()
        
        mpar=[]
                
        for i in range(self.n_models):
            ddd={}
            print "---------------"
            print "Layer: ", self.grid.models[i].name
            print "Sill: ", float(self.grid.models[i].Sill)
            print "Range: ", float(self.grid.models[i].Range)
            print self.grid.models[i].variogram_model
            if self.grid.models[i].variogram_model != 'linear':
                print "Nugget: ", float(self.grid.models[i].Nugget)           
                ddd["Layer"]= self.grid.models[i].name
                ddd["Sill"]= float(self.grid.models[i].Sill)
                ddd["Range"]= float(self.grid.models[i].Range)
                ddd["Nugget"]= float(self.grid.models[i].Nugget)
                ddd["model"]= self.grid.models[i].variogram_model
                ddd["var"]= float(self.grid.models[i].avg_var)
            else:
                ddd["Layer"]= self.grid.models[i].name
                ddd["Sill"]= float(self.grid.models[i].sim_sill)          
                ddd["Nugget"]= float(self.grid.models[i].Sill)
                ddd["Range"]= (ddd["Sill"]-ddd["Nugget"])/float(self.grid.models[i].Range)
                ddd["model"]= self.grid.models[i].variogram_model
                ddd["var"]= float(self.grid.models[i].avg_var)

            mpar.append(ddd)
            
            
        outfiled = 'cosmos-groundtruth-params-1m-spherical.yaml'
        yml = yaml.safe_dump(mpar, default_flow_style=False)
        fhd = open(outfiled, "w")
        s_output = str(yml)
        print s_output
        fhd.write(s_output)
        fhd.close()


        self.image = self.satellite.base_image.copy()
        while(self.running):
            cv2.imshow('simulator', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)



    def model_comparison_cb(self, req):
        print req.model_name, req.model_index, req.height, req.width
        compm = np.reshape(np.asarray(req.vals), (req.height, req.width))
        if req.model_name == 'kriging':
            diff = self.grid.models[req.model_index].output-compm
        else :
            diff = self.grid.mean_output-compm
        return True, np.mean(diff), np.mean(diff**2), np.std(diff), np.var(diff)


    def load_groundtruth(self, filename):
        self.grid.load_data_from_yaml(filename)
        self.vmin, self.vmax = self.grid.get_max_min_vals()
        print "LIMS: " + str(self.vmin) + " " + str(self.vmax)
        self.n_models=len(self.grid.models)
        self.current_model=0


    def krieg_all_mmodels(self):
        for i in self.grid.models:
            i.do_krigging()

            
    def click_callback(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            cx, cy = self.grid.get_cell_inds_from_coords(click_coord)

            if cx <0 or cy<0:
                print "click outside the grid"
            else:
                hmm = KrigInfo()               
                hmm.header = std_msgs.msg.Header()
                hmm.header.stamp = rospy.Time.now() 
                hmm.coordinates.latitude = click_coord.lat
                hmm.coordinates.longitude = click_coord.lon
                
                for i in self.grid.models:
                    mmh = KrigMsg()
                    mmh.model_name = i.name
                    mmh.measurement = i.output[cy][cx]
                    hmm.data.append(mmh)
                
                self.data_pub.publish(hmm)


    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            self.last_coord=data

    def scan_callback(self, msg):
        if msg.data == 'Do_reading':
            print "generating reading"
            gps_coord = MapCoords(self.last_coord.latitude,self.last_coord.longitude) 
            cx, cy = self.grid.get_cell_inds_from_coords(gps_coord)
            hmm = KrigInfo()               
            hmm.header = std_msgs.msg.Header()
            hmm.header.stamp = rospy.Time.now() 
            hmm.coordinates = self.last_coord
#            hmm.coordinates.longitude = click_coord.lon
            
            for i in self.grid.models:
                mmh = KrigMsg()
                mmh.model_name = i.name
                mmh.measurement = i.output[cy][cx]
                hmm.data.append(mmh)
            
            self.data_pub.publish(hmm)
            print hmm    

    def _change_mode(self, k):
        if k == 27:           
            self.running = False
            
        elif k == ord('q'):
            self.running = False
            
        elif k == ord('c'):
            print "Draw corners"
            self.satellite.draw_coordinate(self.grid.swc,(0,255,0,128))
            self.satellite.draw_coordinate(self.grid.sec,(0,0,255,128))
            self.satellite.draw_coordinate(self.grid.nwc,(255,0,0,128))
            self.satellite.draw_coordinate(self.grid.nec,(255,255,255,128))
            self.image = self.satellite.base_image.copy()

        elif k == ord('i'):
            self.draw_mode='inputs'
            self.draw_inputs(self.current_model)

        elif k == ord('t'):
            self.draw_mode='krigging'
            self.draw_krigged(self.current_model)

        elif k == ord('v'):
            self.draw_mode='sigma'
            self.draw_deviation(self.current_model)

        elif k == ord('>'):
            self.current_model+=1
            if self.current_model >= self.n_models:
                self.current_model=0

            if self.draw_mode=='inputs':
                self.draw_inputs(self.current_model)
            if  self.draw_mode=='krigging':
                self.draw_krigged(self.current_model)
            if  self.draw_mode=='sigma':
                self.draw_deviation(self.current_model)          

        elif k == ord('<'):
            self.current_model-=1
            if self.current_model < 0:
                self.current_model=self.n_models-1

            if self.draw_mode=='inputs':
                self.draw_inputs(self.current_model)
            if  self.draw_mode=='krigging':
                self.draw_krigged(self.current_model)
            if  self.draw_mode=='sigma':
                self.draw_deviation(self.current_model)

        elif k == ord('l'):
            print "Draw Limits"
            #self.satellite.draw_list_of_coords(self.kriged.limits, (0,0,255,128))
            self.satellite.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
            self.image = self.satellite.base_image.copy()
        elif k == ord('g'):
            print "Draw Grid"
            print "Grid Shape: ", self.grid.shape
            self.satellite.draw_grid(self.grid.cells, self.grid.cell_size, (64,64,64,64), thickness=1)            
#            print len(self.kriged.waypoints), len(self.kriged.waypoints[0])
#            for i in range(0, len(self.kriged.waypoints)):
#                self.satellite.draw_list_of_coords(self.kriged.waypoints[i], (128,0,128,64), size=2, thickness=-1)
            self.image = self.satellite.base_image.copy()

            

#
#            vmin = np.floor(self.grid.data[0].lims[0])
#            vmax = np.ceil(self.grid.data[0].lims[1])
#            print vmin, vmax            
#            
#            norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
#            cmap = cm.jet
#            colmap = cm.ScalarMappable(norm=norm, cmap=cmap)            
#                       
#            for i in range(self.grid.data[0].shape[0]):
#                for j in range(self.grid.data[0].shape[1]):
#                    cell = self.grid.cells[i][j]
#                    a= colmap.to_rgba(int(self.grid.data[0].output[i][j]))
#                    b= (int(a[0]*255), int(a[1]*255), int(a[2]*255), int(a[3]*50))
#                    self.satellite.draw_cell(cell, self.grid.cell_size, b, thickness=-1)
#
#            self.satellite.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
#            #self.satellite.draw_grid(self.grid.cells, self.grid.cell_size, (64,64,64,64), thickness=1)
#            self.image = self.satellite.base_image.copy()






    def draw_deviation(self, nm):
        font = cv2.FONT_HERSHEY_SIMPLEX
        norm = mpl.colors.Normalize(vmin=0, vmax=100)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

#        vmin = np.floor(self.grid.data[0].lims[0])
#        vmax = np.ceil(self.grid.data[0].lims[1])
#        norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
#        cmap = cm.jet
#        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)            
                   
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].sigmapercent[i][j]*100))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.satellite.draw_cell(cell, self.grid.cell_size, b, thickness=-1)

        self.satellite.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
        self.image = self.satellite.base_image.copy()        
        cv2.putText(self.image, self.grid.models[nm].name, (int(520), int(20)), font, 0.8, (200, 200, 200), 2)
        self.draw_legend(self.vmin, self.vmax)




    def draw_krigged(self, nm):
        font = cv2.FONT_HERSHEY_SIMPLEX
        norm = mpl.colors.Normalize(vmin=self.vmin, vmax=self.vmax)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

#        vmin = np.floor(self.grid.data[0].lims[0])
#        vmax = np.ceil(self.grid.data[0].lims[1])
#        norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
#        cmap = cm.jet
#        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)            
                   
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].output[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.satellite.draw_cell(cell, self.grid.cell_size, b, thickness=-1)

        self.satellite.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
        self.image = self.satellite.base_image.copy()        
        cv2.putText(self.image, self.grid.models[nm].name, (int(520), int(20)), font, 0.8, (200, 200, 200), 2)
        self.draw_legend(self.vmin, self.vmax)



    def draw_inputs(self, nm):
#        vmin = np.floor(self.grid.models[nm].lims[0])
#        vmax = np.ceil(self.grid.models[nm].lims[1])
#        print vmin, vmax            
#
        font = cv2.FONT_HERSHEY_SIMPLEX
        norm = mpl.colors.Normalize(vmin=self.vmin, vmax=self.vmax)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)
        #print "N Origin data: ", len(self.grid.models[nm].orig_data)
#
        for i in self.grid.models[nm].orig_data:
            cell = self.grid.cells[i.y][i.x]
#                #print i.value
            a= colmap.to_rgba(int(i.value))                
            b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
#                #print a, b
            self.satellite.draw_cell(cell, self.grid.cell_size, b, thickness=-1)

        self.image = self.satellite.base_image.copy()
        cv2.putText(self.image, self.grid.models[nm].name, (int(520), int(20)), font, 0.8, (200, 200, 200), 2)
        self.draw_legend(self.vmin, self.vmax)
    
    
    def draw_legend(self, vmin, vmax):
        font = cv2.FONT_HERSHEY_SIMPLEX
        step = (vmax - vmin)/(600-40)
        norm = mpl.colors.Normalize(vmin=self.vmin, vmax=self.vmax)
        cmap = cm.jet
#        cmap = cm.plasma
#        cmap = cm.gnuplot
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        vp = range(int(np.floor(vmin)),int(np.ceil(vmax)), int(np.ceil(step)))
        print len(vp)        

        if step>1.0:
            ind = 0
            while ind < 560:
#                print int(vmin+(ind*step))
                a= colmap.to_rgba(int(vmin+(ind*step)))                
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))                
                cv2.rectangle(self.image, (int(ind+40), int(580)), (int(ind+1+40), int(600)), b , thickness=-1)
                ind+=1
#            cv2.rectangle(self.image, (int(40), int(580)), (int(600), int(600)), (200,0,0,0), thickness=-1)

        a= colmap.to_rgba(int(vmin))
        b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
        cv2.putText(self.image, str(self.vmin) + " Kpa", (int(5), int(575)), font, 0.6, b, 2)
        a= colmap.to_rgba(int(vmax))
        b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
        cv2.putText(self.image, str(self.vmax) + " Kpa", (int(520), int(575)), font, 0.6, b, 2)


    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    args = parser.parse_args()
    
    rospy.init_node('kriging_simulator')
    
#    simulator(53.267213, -0.533420, 17, 640, args.cell_size)  #Football Field
#    simulator(39.432305, 22.79269, 17, 640, args.cell_size)    #Greece
    simulator(53.138604, 0.004182, 19, 640, args.cell_size)    #Greece    
#    simulator(53.261576, -0.526648, 17, 640, args.cell_size)  #Half cosmos field
#    simulator(53.261685, -0.525158, 17, 640, args.cell_size)  #Full cosmos field

