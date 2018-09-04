#!/usr/bin/env python

import signal
import yaml
import sys
import argparse
import rospy

import cv2

import kriging_exploration.canvas

from kriging_exploration.map_coords import MapCoords
#from kriging_exploration.canvas import ViewerCanvas
from kriging_exploration.visualiser import KrigingVisualiser



import numpy as np




import std_msgs

from cosmos_msgs.msg import KrigInfo
from cosmos_msgs.msg import KrigMsg
from cosmos_msgs.srv import CompareModels

#import satellite
from kriging_exploration.satellite import SatelliteImage


#from krigging_data import KriggingData
from sensor_msgs.msg import NavSatFix




def load_field_defs(filename):
    with open(filename, 'r') as f:
        a = yaml.load(f)
    return a

class PoissonSimulation(KrigingVisualiser):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }
    draw_mode="none"
    current_model=0
    running = True
    publish = False

    def __init__(self, field, cell_size):

        signal.signal(signal.SIGINT, self.signal_handler)
        self.data_pub = rospy.Publisher('/kriging_data', KrigInfo, latch=False, queue_size=1)

        print "Creating visualiser object"
        super(PoissonSimulation, self).__init__(field['field']['lat'], field['field']['lon'], field['field']['zoom'], 640)

        limits=[]
        for i in field['field']['limits']:
            b=MapCoords(i[0],i[1])
            limits.append(b)

        cv2.namedWindow('Kriging_simulator')
        cv2.setMouseCallback('Kriging_simulator', self.click_callback)
        
        self.image = self.satellite.base_image.copy()

        self.create_grid(cell_size, limits)
        self.draw_grid()

        self.load_groundtruth_data('airfield-sim.data')

        self.draw_inputs(0, alpha=200)


        pubtim = rospy.Timer(rospy.Duration(0.5), self.pub_timer_callback)

        self.grid.krieg_all_mmodels()
        self.draw_krigged(0, alpha=200)

        self.redraw()

        while(self.running):
            cv2.imshow('Kriging_simulator', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)
            if k == 27:
                self.running = False

        pubtim.shutdown()
        cv2.destroyAllWindows()       
        sys.exit(0)


    def redraw(self):
        self.base_image = self.satellite.base_image.copy()
        if self.draw_mode=='inputs':
            self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.model_canvas[self.current_model].image)
        if self.draw_mode=='kriging':
            self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.kriging_canvas[self.current_model].image)
        self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.grid_canvas.image)
        
        
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
        elif k == ord('k'):
            if self.n_models > 0:
                self.draw_mode="kriging"
                self.current_model=0
                self.redraw()

    def pub_timer_callback(self, event):
        if self.publish:
            hmm = KrigInfo()               
            hmm.header = std_msgs.msg.Header()
            hmm.header.stamp = rospy.Time.now() 
            hmm.coordinates.latitude = self.scan_coord.lat
            hmm.coordinates.longitude = self.scan_coord.lon
            
            cx, cy = self.grid.get_cell_inds_from_coords(self.scan_coord)            
            
            for i in self.grid.models:
                mmh = KrigMsg()
                mmh.model_name = i.name
                mmh.measurement = i.output[cy][cx]
                hmm.data.append(mmh)
            
            self.data_pub.publish(hmm)            


    def click_callback(self, event, x, y, flags, param):
        #print "click"
        
        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            cx, cy = self.grid.get_cell_inds_from_coords(click_coord)

            if cx <0 or cy<0:
                print "click outside the grid"
            else:
                self.scan_coord=click_coord
                self.publish=True
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.publish=False



    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    parser.add_argument("--field_def_file", type=str, default="field.field",
                        help="cell size in meters")
    args = parser.parse_args()
    
    field=load_field_defs(args.field_def_file)
    rospy.init_node('kriging_simulator')
    PoissonSimulation(field, args.cell_size)

   
#    simulator(53.267213, -0.533420, 17, 640, args.cell_size)  #Football Field
#    simulator(39.432305, 22.79269, 17, 640, args.cell_size)    #Greece
#    simulator(53.138604, 0.004182, 19, 640, args.cell_size)    #Greece    
#    simulator(53.261576, -0.526648, 17, 640, args.cell_size)  #Half cosmos field
#    simulator(53.261685, -0.525158, 17, 640, args.cell_size)  #Full cosmos field

