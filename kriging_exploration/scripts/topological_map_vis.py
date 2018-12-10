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

import matplotlib.pyplot as plt

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
    
    excluded=['HR_001','HR_003','HR_005','HR_006','HR_008','HR_010',
              'HR_012','HR_013','HR_014','HR_015','HR_016','HR_017','HR_018','HR_019','HR_020','HR_021','HR_022','HR_023','HR_024',
              'HR_027','HR_029','HR_031','HR_032','HR_034','HR_036',
              'HR_038','HR_039','HR_040','HR_041','HR_042','HR_043','HR_044','HR_045','HR_046','HR_047','HR_048','HR_049','HR_050',
              'HR_053','HR_055','HR_057','HR_058','HR_060','HR_062',
              'HR_064','HR_065','HR_066','HR_067','HR_068','HR_069','HR_070','HR_071','HR_072','HR_073','HR_074','HR_075','HR_076',
              'HR_079','HR_081','HR_083','HR_084','HR_086','HR_088',
              'HR_090','HR_091','HR_092','HR_093','HR_094','HR_095','HR_096','HR_097','HR_098','HR_099','HR_100','HR_101','HR_102',
              'HR_104','HR_106','HR_108','HR_109','HR_111','HR_113']

    def __init__(self, field_file, size, cell_size):
        self.running = True
        self.topo_map=[]
        signal.signal(signal.SIGINT, self.signal_handler)
        map_angle=30.0

        with open(field_file, 'r') as f:
            a = yaml.load(f)

        print a
        lat_deg = a['field']['lat']
        lon_deg = a['field']['lon']
        zoom = a['field']['zoom']

        print "Creating visualiser object"
        super(SimpleDataVisualiser, self).__init__(lat_deg, lon_deg, zoom, size)

        cv2.namedWindow('SimpleDataVisualiser')
        cv2.setMouseCallback('SimpleDataVisualiser', self.click_callback)

#        self.grid = DataGrid(a['field']['limits_file'], 10)
#        self.grid2 = DataGrid(a['field']['limits_file'], 10)
#        self.load_groundtruth()



        self.map_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.gps_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)        
        
        
        
        self.create_map(self.centre, map_angle)
        self.draw_coords(self.centre,map_angle)
        
        rospy.loginfo("Subscribing to GPS Data")
        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        
        
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


    def click_callback(self, event, x, y, flags, param):
        
        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            closest_click = self.get_closest_node(click_coord)
            print "Closest CLICK: ", closest_click



    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            gps_coord = MapCoords(data.latitude,data.longitude)
            self.closest_node = self.get_closest_node(gps_coord)
            self.gps_canvas.clear_image()
            self.gps_canvas.draw_coordinate(self.closest_node.coord,'red',size=6, thickness=2, alpha=255)
            self.gps_canvas.draw_coordinate(gps_coord,'white',size=2, thickness=2, alpha=255)



    def get_closest_node(self, gps_coord):
        dist, node = self.check_topo_map_dist(gps_coord)
        dd=round(dist,2)
        de=round(node.coord.easting-gps_coord.easting,2)
        dn=round(node.coord.northing-gps_coord.northing,2)
        print node.name, dd, 'E: ', de, 'N: ', dn 
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
        


    # Function to add calibration points to map
    def create_cal_map(self, centre, degang):
#        ang=math.radians(degang)        
        self.calib_map=[]
        name_index=0


        # Calibration Points
        for i in range(-60,61,120):
#            ang2=math.radians(degang+90.0)
            ang2=math.radians(degang)
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


    # Function to add high res points to map
    def create_hr_map(self, centre, degang):
        ang=math.radians(degang)
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


    # Function to add low res points to map
    def create_lr_map(self, centre, degang):
        ang=math.radians(degang)
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
            for j in range(-150,151,30):
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


    def create_map(self, centre, degang):
#       ang=math.radians(degang)
       self.lowr_map=[]
       self.highr_map=[]
       self.pin_map=[]
       self.ceh_map=[]
       self.final_map=[]
       
       self.create_cal_map(centre, degang)
       self.create_hr_map(centre, degang)
       self.create_lr_map(centre, degang)


       # Pinboard points
       # North - South Line
       for i in range(-150,151,30):
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
               for h in [2,3,5]:
                   ang2=math.radians(degang+j+90)
                   dy=h*math.cos(ang2)
                   dx=h*math.sin(ang2)
                   new_coord2=i.coord._get_rel_point(dy,dx)
                   self.ceh_map.append(new_coord2)
       
       
       for i in self.topo_map:
           if i.name not in self.excluded:
               self.final_map.append(i)
        
        
        

#2 5 25 75


    def draw_coords(self, centre, degang):
#        for i in self.calib_map:  
#            self.map_canvas.draw_coordinate(i.coord,'red',size=5, thickness=1, alpha=255)
#
#        for i in self.lowr_map:
#            self.map_canvas.draw_coordinate(i.coord,'white',size=5, thickness=1, alpha=255)
#
#        for i in self.highr_map:
#            self.map_canvas.draw_coordinate(i.coord,'yellow',size=5, thickness=1, alpha=255)#,shape='rect')
#
#        
#        for i in self.pin_map:  
#            self.map_canvas.draw_coordinate(i,'blue',size=2, thickness=2, alpha=255, shape='rect')
#        
#        for i in self.ceh_map:  
#            self.map_canvas.draw_coordinate(i,'cyan',size=2, thickness=1, alpha=255)#, shape='rect')



#        line=[]
#        
#        ang=math.radians(degang)
#        dy=-22.5*math.cos(ang)
#        dx=-22.5*math.sin(ang)
#        irr_centre=centre._get_rel_point(dy,dx)
#
#
#        ang2=math.radians(degang+90.0)
#        dy=220*math.cos(ang2)
#        dx=220*math.sin(ang2)
#        new_coord=irr_centre._get_rel_point(dy,dx)
#        line.append(new_coord)
#
#        dy=-220*math.cos(ang2)
#        dx=-220*math.sin(ang2)
#        new_coord2=irr_centre._get_rel_point(dy,dx)
#        line.append(new_coord2)
#
#        self.map_canvas.draw_line(line, 'white')

        line=[]
        
        ang=math.radians(degang)
        dy=0*math.cos(ang)
        dx=0*math.sin(ang)
        irr_centre=centre._get_rel_point(dy,dx)


        ang2=math.radians(degang+90.0)
        dy=220*math.cos(ang2)
        dx=220*math.sin(ang2)
        new_coord=irr_centre._get_rel_point(dy,dx)
        line.append(new_coord)

        dy=-220*math.cos(ang2)
        dx=-220*math.sin(ang2)
        new_coord2=irr_centre._get_rel_point(dy,dx)
        line.append(new_coord2)

        self.map_canvas.draw_line(line, 'white')
        
        
#        line =[]
#        ang=math.radians(degang)
#        dy=-77.5*math.cos(ang)
#        dx=-77.5*math.sin(ang)
#        irr_centre=centre._get_rel_point(dy,dx)
#
#
#        ang2=math.radians(degang+90.0)
#        dy=220*math.cos(ang2)
#        dx=220*math.sin(ang2)
#        new_coord=irr_centre._get_rel_point(dy,dx)
#        line.append(new_coord)
#
#        dy=-220*math.cos(ang2)
#        dx=-220*math.sin(ang2)
#        new_coord2=irr_centre._get_rel_point(dy,dx)
#        line.append(new_coord2)


        self.map_canvas.draw_line(line, 'white')
        print len(self.topo_map)

        for i in self.final_map:
            self.map_canvas.draw_coordinate(i.coord,'white',size=2, thickness=2, alpha=255)


        labels=[]
        pxs=[]
        pys=[]
        filelines=[]
        filelines.append('Name; lat; lon')
        for i in self.final_map:
#            self.map_canvas.draw_coordinate(i.coord,'white',size=2, thickness=1, alpha=255)
            px, py = self.map_canvas._coord2pix(i.coord)
            labels.append(i.name)
            pxs.append(px)
            pys.append(640-py)
            #print i.name, px, py, i.coord
            node_str= i.name + '; ' + str(i.coord.lat) + '; ' + str(i.coord.lon)
            filelines.append(node_str)
        
        print min(pxs), max(pxs)
        print min(pys), max(pys)
        
        for j in filelines:
            print j
        
        

        plt.subplots_adjust(bottom = 0.1)
        plt.scatter(pxs, pys, marker='o', c='blue', s= 100)
        
        for lab, x, y in zip(labels, pxs, pys):
            plt.annotate(lab, xy=(x, y), xytext=(-25, -20), textcoords='offset points')
            #, ha='right', va='bottom',
            #    bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5))
                #,arrowprops=dict(arrowstyle = '->', connectionstyle='arc3,rad=0'))
        
        #plt.show()

        
        
        
        
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

