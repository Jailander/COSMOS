#!/usr/bin/env python

import signal
import numpy as np

import cv2
import sys
import yaml
import argparse
import math

import matplotlib as mpl
import matplotlib.cm as cm



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


class SimpleDataVisualiser(KrigingVisualiser):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }

    def __init__(self, lat, lon, zoom, size, cell_size):
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

#        with open(field_file, 'r') as f:
#            a = yaml.load(f)


        lat_deg = lat
        lon_deg = lon
        #zoom =a['field']['zoom']

        print "Creating visualiser object"
        super(SimpleDataVisualiser, self).__init__(lat_deg, lon_deg, zoom, size)
#
        cv2.namedWindow('SimpleDataVisualiser')
#
#        self.grid = DataGrid('limits.coords', cell_size)
#        
        self.image = self.satellite.base_image.copy()
        self.map_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
#
        self.get_corners(-15.5)
        cell_size=5.0
        self.grid = DataGrid(None, cell_size, limit_list=self.corners)        
        
        
        self.drawing_grid(self.centre, -15.5)
        self.generate_files()
        while(self.running):
            cv2.imshow('SimpleDataVisualiser', self.image)
            k = cv2.waitKey(20) & 0xFF
#            self._change_mode(k)
            if k == 27:
                self.running = False
        cv2.destroyAllWindows()       
        sys.exit(0)

    def get_corners(self, degang):
        self.corners=[]
        angie=math.pi/4
        ang=math.radians(degang)
        radi=30.0/math.cos(angie)
        
        dx=-radi*math.cos(ang+angie)
        dy=-radi*math.sin(ang+angie)
        x0=self.centre._get_rel_point(dy,dx)
#        self.map_canvas.draw_coordinate(x0,'white', alpha=200)
        
        dx=-radi*math.cos(ang-angie)
        dy=-radi*math.sin(ang-angie)
        x1=self.centre._get_rel_point(dy,dx)
#        self.map_canvas.draw_coordinate(x1,'black', alpha=200)        
        
        dx=radi*math.cos(ang+angie)
        dy=radi*math.sin(ang+angie)
        x2=self.centre._get_rel_point(dy,dx)
#        self.map_canvas.draw_coordinate(x2,'black', alpha=200)

        dx=radi*math.cos(ang-angie)
        dy=radi*math.sin(ang-angie)
        x3=self.centre._get_rel_point(dy,dx)
#        self.map_canvas.draw_coordinate(x3,'black', alpha=200)
        self.corners.append(x0)
        self.corners.append(x1)
        self.corners.append(x2)
        self.corners.append(x3)        


    def drawing_grid(self, centre, degang):
        self.map_canvas.draw_coordinate(self.centre,'black',size=5, thickness=1, alpha=255)

        print "Draw Grid"
        print "Grid Shape: ", self.grid.shape
        self.map_canvas.draw_grid(self.grid.cells, self.grid.cell_size, (64,64,64,64), thickness=1)

        ang=math.radians(degang)
        cwp=[]
        for i in range(0, 12):
            dx=(-(i-5.5)*5.0)*math.cos(ang)
            dy=(-(i-5.5)*5.0)*math.sin(ang)
            x0=self.centre._get_rel_point(dy,dx)            
            cwp.append(x0)
        
        ang=math.radians(degang+90)
        self.dry_data_points=[]
        self.wet_data_points=[]
        for i in cwp:
            for j in range(0, 12):
                dx=(-(j-5.5)*5.0)*math.cos(ang)
                dy=(-(j-5.5)*5.0)*math.sin(ang)
                x0=i._get_rel_point(dy,dx)
                if (j-5.5) < 0:
                    self.dry_data_points.append(x0)
                else:
                    self.wet_data_points.append(x0)
        
        self.map_canvas.draw_list_of_coords(self.dry_data_points, 'red', size=6, thickness=2)
        self.map_canvas.draw_list_of_coords(self.wet_data_points, 'blue', size=6, thickness=2)
        self.map_canvas.draw_list_of_coords(self.corners, 'black', size=6, thickness=2)
       
        self.redraw()

    def redraw(self):
        self.image = overlay_image_alpha(self.image,self.map_canvas.image)


    def generate_files(self):
        with open('airfield.coords', 'w') as f:
            for i in self.corners:
                line = str(i.lat) +', '+ str(i.lon) + '\n'
                f.write(line)
            f.close()
        
        fd={}
        fd['field']={}
        fd['field']['name']='airfield'
        fd['field']['zoom']=19
        fd['field']['lat']=self.centre.lat
        fd['field']['lon']=self.centre.lon
        fd['field']['limits_file']='airfield.coords'
        
        fdstr=yaml.safe_dump(fd, default_flow_style=False)
        with open('airfield.field', 'w') as f:
            f.write(fdstr)
            f.close()
        
        
        gd={}
        gd['data']=[]
        gd['names']=['count_rate']        
        
        for i in self.dry_data_points:
            d={}
            d['data']=[]
            d['data'].append(50.0)
            d['position']={}
            d['position']['lat']=i.lat
            d['position']['lon']=i.lon
            gd['data'].append(d)
            
        for i in self.wet_data_points:
            d={}
            d['data']=[]
            d['data'].append(25.0)
            d['position']={}
            d['position']['lat']=i.lat
            d['position']['lon']=i.lon
            gd['data'].append(d)

        gdstr=yaml.safe_dump(gd, default_flow_style=False)
        with open('airfield-sim.data', 'w') as f:
            f.write(gdstr)
            f.close()
            
#    def refresh(self):
#        self.show_image = overlay_image_alpha(self.image, self.gps_canvas.image)



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
    
#    SimpleDataVisualiser(args.field_file, 640, args.cell_size)
    SimpleDataVisualiser(53.138604, 0.004182, 19, 640, args.cell_size)

