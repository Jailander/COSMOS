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
        self.drawing_grid(self.centre, 15.5)

        while(self.running):
            cv2.imshow('SimpleDataVisualiser', self.image)
            k = cv2.waitKey(20) & 0xFF
#            self._change_mode(k)
            if k == 27:
                self.running = False
        cv2.destroyAllWindows()       
        sys.exit(0)


    def drawing_grid(self, centre, degang):
        self.map_canvas.draw_coordinate(self.centre,'black',size=5, thickness=1, alpha=255)

        #zero lines
        ang=math.radians(-degang)
        line=[]
        dx=-30*math.cos(ang)
        dy=-30.0*math.sin(ang)
        x0=centre._get_rel_point(dy,dx)
        dx=30.0*math.cos(ang)
        dy=30.0*math.sin(ang)
        x1=centre._get_rel_point(dy,dx)
        line.append(x0)        
        line.append(x1)
        self.map_canvas.draw_line(line, 'red')


        ang=math.radians(90-degang)
        lineb=[]
        dx=-30*math.cos(ang)
        dy=-30.0*math.sin(ang)
        xb0=x0._get_rel_point(dy,dx)
        dx=30.0*math.cos(ang)
        dy=30.0*math.sin(ang)
        xb1=x0._get_rel_point(dy,dx)
        lineb.append(xb0)        
        lineb.append(xb1)
        self.map_canvas.draw_line(lineb, 'red')                
        
        
        ang=math.radians(90-degang)
        line=[]
        dx=-30*math.cos(ang)
        dy=-30.0*math.sin(ang)
        x0=centre._get_rel_point(dy,dx)
        dx=30.0*math.cos(ang)
        dy=30.0*math.sin(ang)
        x1=centre._get_rel_point(dy,dx)
        line.append(x0)        
        line.append(x1)
        self.map_canvas.draw_line(line, 'red')        
        
        #corners        
        
        corners=[]
        angie=math.pi/4
        ang=math.radians(-degang)
        radi=37.50/math.cos(angie)
        
        dx=-radi*math.cos(ang+angie)
        dy=-radi*math.sin(ang+angie)
        x0=centre._get_rel_point(dy,dx)
        self.map_canvas.draw_coordinate(x0,'white', alpha=200)
        
        dx=-radi*math.cos(ang-angie)
        dy=-radi*math.sin(ang-angie)
        x1=centre._get_rel_point(dy,dx)
        self.map_canvas.draw_coordinate(x1,'black', alpha=200)        
        
        dx=radi*math.cos(ang+angie)
        dy=radi*math.sin(ang+angie)
        x2=centre._get_rel_point(dy,dx)
        self.map_canvas.draw_coordinate(x2,'black', alpha=200)

        dx=radi*math.cos(ang-angie)
        dy=radi*math.sin(ang-angie)
        x1=centre._get_rel_point(dy,dx)
        self.map_canvas.draw_coordinate(x1,'black', alpha=200)
        
        self.redraw()

    def redraw(self):
        self.image = overlay_image_alpha(self.image,self.map_canvas.image)


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

