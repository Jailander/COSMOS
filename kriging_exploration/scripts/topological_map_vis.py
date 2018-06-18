#!/usr/bin/env python

import signal
import numpy as np
import rospy
import time
import math

import cv2
import sys
import yaml
import argparse

import matplotlib as mpl
import matplotlib.cm as cm

from sensor_msgs.msg import NavSatFix

from kriging_exploration.map_coords import MapCoords

from kriging_exploration.visualiser import KrigingVisualiser
from kriging_exploration.data_grid import DataGrid
from kriging_exploration.canvas import ViewerCanvas



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



class topo_map_node(object):
    def __init__(self, name, coord):
        self.name = name
        self.coord = coord


    def __repr__(self):
        a = dir(self)
        b = []
        s = ''
        for i in a:
            if not i.startswith('_'):
                b.append(str(i))
        for i in b:
                s = s + str(i) + ': ' + str(self.__getattribute__(i)) + '\n'
        return s


class SimpleDataVisualiser(KrigingVisualiser):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }

    coords=[[53.13986, 0.00572], [53.13943, 0.00462], [53.13936, 0.00454], 
            [53.13917, 0.004], [53.13917, 0.00394], [53.13919, 0.00393], 
            [53.13919, 0.00391], [53.13916, 0.00383], [53.13893, 0.00324], 
            [53.1389, 0.00343], [53.1391, 0.00462], [53.13907, 0.00419], [53.13701, 0.00499]]

    def __init__(self, field_file, size, cell_size):
        self.running = True
        self.topo_map=[]
        signal.signal(signal.SIGINT, self.signal_handler)


        with open(field_file, 'r') as f:
            a = yaml.load(f)

        print a
        lat_deg = a['field']['lat']
        lon_deg = a['field']['lon']
        zoom = a['field']['zoom']

        print "Creating visualiser object"
        super(SimpleDataVisualiser, self).__init__(lat_deg, lon_deg, zoom, size)

        cv2.namedWindow('SimpleDataVisualiser')


#        self.grid = DataGrid(a['field']['limits_file'], 10)
#        self.grid2 = DataGrid(a['field']['limits_file'], 10)
#        self.load_groundtruth()



        self.map_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.gps_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)        
        
        self.create_map(self.centre, 30.0)
        self.draw_coords()
        
        rospy.loginfo("Subscribing to GPS Data")
        rospy.Subscriber("/navsat_fix", NavSatFix, self.gps_callback)
        
        
        while(self.running):
            self.refresh()
            cv2.imshow('SimpleDataVisualiser', self.show_image)
            k = cv2.waitKey(20) & 0xFF
#            self._change_mode(k)
            if k == 27:
                self.running = False
            time.sleep(0.02)
            
        cv2.destroyAllWindows()       
        sys.exit(0)


    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            gps_coord = MapCoords(data.latitude,data.longitude)
            self.closest_node = self.get_closest_node(gps_coord)
            self.gps_canvas.clear_image()
            self.gps_canvas.draw_coordinate(self.closest_node.coord,'red',size=6, thickness=2, alpha=255)
            self.gps_canvas.draw_coordinate(gps_coord,'white',size=2, thickness=2, alpha=255)



    def get_closest_node(self, gps_coord):
        dist, node = self.check_topo_map_dist(gps_coord)
        print node.name, dist, 'E: ', node.coord.easting-gps_coord.easting, 'N: ', node.coord.northing-gps_coord.northing
        return node

    def check_topo_map_dist(self, coord):
        mindist=10000
        closest='none'
        for i in self.topo_map:
            dist, ori = coord-i.coord
            if dist < mindist:
                mindist=dist
                closest=i
                if mindist < 0.5:
                    break
        
        return mindist, closest
        

    def create_cal_map(self, centre, degang):
#        ang=math.radians(degang)        
        self.calib_map=[]
        name_index=0


        # Calibration Points
        for i in range(-120,121,120):
            ang2=math.radians(degang+90.0)
            dy=i*math.cos(ang2)
            dx=i*math.sin(ang2)
            new_coord = self.centre._get_rel_point(dy,dx)
            
            dist, clnode = self.check_topo_map_dist(new_coord)
            if dist > 0.5:
                name='C_'+str("%03d" %name_index)
                name_index+=1
                map_node = topo_map_node(name, new_coord)
                self.calib_map.append(map_node)        
                self.topo_map.append(map_node)
            else:
                print "node too close to: ", clnode.name


    def create_map(self, centre, degang):
        ang=math.radians(degang)
        self.lowr_map=[]
        self.highr_map=[]
        self.pin_map=[]
        self.ceh_map=[]
        
        self.create_cal_map(centre, degang)


        #High res map

        name_index=0
        #East-West
        for i in range(-60,61,15):
            dy=i*math.sin(ang)
            dx=i*math.cos(ang)
            new_coord=centre._get_rel_point(dx,dy)
            dist, clnode = self.check_topo_map_dist(new_coord)
            if dist > 0.5:
                name='HR_'+str("%03d" %name_index)
                name_index+=1
                map_node = topo_map_node(name, new_coord)
                self.highr_map.append(map_node)
                self.topo_map.append(map_node)
            else:
                print "node too close to: ", clnode.name
            
            #North - South
            for j in range(-90,91,15):
                ang2=math.radians(degang+90.0)
                dy=j*math.cos(ang2)
                dx=j*math.sin(ang2)
                new_coord2=new_coord._get_rel_point(dy,dx)
                dist, clnode = self.check_topo_map_dist(new_coord2)
                if dist > 0.5:
                    name='HR_'+str("%03d" %name_index)
                    name_index+=1
                    map_node = topo_map_node(name, new_coord2)
                    self.highr_map.append(map_node)
                    self.topo_map.append(map_node)
                else:
                    print "node too close to: ", clnode.name                
                
      

        
        #Low res map

        name_index=0
        #East-West
        for i in range(-90,91,30):
            dy=i*math.sin(ang)
            dx=i*math.cos(ang)
            new_coord=centre._get_rel_point(dx,dy)
            dist, clnode = self.check_topo_map_dist(new_coord)
            if dist > 0.5:
                name='LR_'+str("%03d" %name_index)
                name_index+=1
                map_node = topo_map_node(name, new_coord)
                self.lowr_map.append(map_node)
                self.topo_map.append(map_node)
            else:
                print "node too close to: ", clnode.name
       
            
            
            #North - South
            for j in range(-210,211,30):
                ang2=math.radians(degang+90.0)
                dy=j*math.cos(ang2)
                dx=j*math.sin(ang2)
                new_coord2=new_coord._get_rel_point(dy,dx)
                dist, clnode = self.check_topo_map_dist(new_coord2)
                if dist > 0.5:
                    name='LR_'+str("%03d" %name_index)
                    name_index+=1
                    map_node = topo_map_node(name, new_coord2)
                    self.lowr_map.append(map_node)
                    self.topo_map.append(map_node)
                else:
                    print "node too close to: ", clnode.name                
                



        # Pinboard points
        # North - South Line
        for i in range(-210,211,30):
            ang2=math.radians(degang+90.0)
            dy=i*math.cos(ang2)
            dx=i*math.sin(ang2)
            new_coord2=centre._get_rel_point(dy,dx)
            self.pin_map.append(new_coord2)


        # Southern East - West Line        
        ang2=math.radians(degang+90.0)
        dy=-120*math.cos(ang2)
        dx=-120*math.sin(ang2)
        new_coord=centre._get_rel_point(dy,dx)

        for j in range(-90,91,30):
            ang2=math.radians(degang)
            dy=j*math.cos(ang2)
            dx=j*math.sin(ang2)
            new_coord2=new_coord._get_rel_point(dy,dx)
            self.pin_map.append(new_coord2)
        
        # Northern East - West Line
        ang2=math.radians(degang+90.0)
        dy=120*math.cos(ang2)
        dx=120*math.sin(ang2)
        new_coord=centre._get_rel_point(dy,dx)

        for j in range(-90,91,30):
            ang2=math.radians(degang)
            dy=j*math.cos(ang2)
            dx=j*math.sin(ang2)
            new_coord2=new_coord._get_rel_point(dy,dx)
            self.pin_map.append(new_coord2)


        # CEH points
        for i in self.calib_map:
            self.ceh_map.append(i.coord)
            for j in range(0,360,60):
                for h in [2,5,25,75]:
                    ang2=math.radians(degang+j+90)
                    dy=h*math.cos(ang2)
                    dx=h*math.sin(ang2)
                    new_coord2=i.coord._get_rel_point(dy,dx)
                    self.ceh_map.append(new_coord2)
        

#2 5 25 75


    def draw_coords(self):
        for i in self.calib_map:  
            print i
            self.map_canvas.draw_coordinate(i.coord,'red',size=5, thickness=1, alpha=255)

        for i in self.lowr_map:
            self.map_canvas.draw_coordinate(i.coord,'white',size=5, thickness=1, alpha=255)

        for i in self.highr_map:
            self.map_canvas.draw_coordinate(i.coord,'yellow',size=5, thickness=1, alpha=255)#,shape='rect')

        
        for i in self.pin_map:  
            self.map_canvas.draw_coordinate(i,'blue',size=2, thickness=2, alpha=255, shape='rect')
        
        for i in self.ceh_map:  
            self.map_canvas.draw_coordinate(i,'cyan',size=2, thickness=1, alpha=255)#, shape='rect')

        print len(self.topo_map)
#        for i in self.topo_map:
#            self.map_canvas.draw_coordinate(i.coord,'white',size=2, thickness=1, alpha=255)#, shape='rect')
            
        self.redraw()

    def redraw(self):
        #self.image = cv2.addWeighted(self.gps_canvas.image, 0.3, self.image, 1.0, 0)
        self.image = overlay_image_alpha(self.image,self.map_canvas.image)


    def refresh(self):
        #self.show_image = cv2.addWeighted(self.gps_canvas.image, 0.3, self.image, 1.0, 0)
        self.show_image = overlay_image_alpha( self.image, self.gps_canvas.image)

    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--field_file", type=str, default='footbal.field',
                        help="cell size in meters")
    parser.add_argument("--cell_size", type=int, default=10,
                        help="field definition file")
    args = parser.parse_args()

    rospy.init_node('topomap_vis')
    SimpleDataVisualiser(args.field_file, 640, args.cell_size)
#    SimpleDataVisualiser(53.261685, -0.525158, 17, 640, args.cell_size)

