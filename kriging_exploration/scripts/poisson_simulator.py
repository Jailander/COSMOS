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
#from kriging_exploration.satellite import SatelliteImage


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
        rospy.Subscriber('/request_scan', std_msgs.msg.String, self.scan_callback)
        rospy.Service('/compare_model', CompareModels, self.model_comparison_cb)


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

        rospy.Subscriber("/navsat_fix", NavSatFix, self.gps_callback)
        self.add_gps_canvas()

        pubtim = rospy.Timer(rospy.Duration(0.5), self.pub_timer_callback)
        drawtim = rospy.Timer(rospy.Duration(0.03), self.draw_timer_callback)

        self.grid.krieg_all_mmodels()

        print "Area Size: ", self.grid.area.area_size        
        self.draw_krigged(0, alpha=200)
        self.draw_variance(0, alpha=200)



        self.redraw()
        self.refresh()
        while(self.running):
            cv2.imshow('Kriging_simulator', self.show_image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)
            if k == 27:
                self.running = False
#            rospy.sleep(0.05)

        pubtim.shutdown()
        drawtim.shutdown()
        cv2.destroyAllWindows()       
        sys.exit(0)


    def redraw(self):
        self.image = self.satellite.base_image.copy()

        if self.draw_mode=='inputs':
            self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.model_canvas[self.current_model].image)
        if self.draw_mode=='kriging':
            self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.kriging_canvas[self.current_model].image)
        if self.draw_mode=='variance':
            self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.sigma_canvas[self.current_model].image)
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
        elif k == ord('v'):
            if self.n_models > 0:
                self.draw_mode="variance"
                self.current_model=0
                self.redraw()

    def draw_timer_callback(self, event):
        self.refresh()

    def refresh(self):
        self.show_image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.gps_canvas.image)


    def pub_timer_callback(self, event):
        if self.publish:
            hmm = KrigInfo()               
            hmm.header = std_msgs.msg.Header()
            hmm.header.stamp = rospy.Time.now() 
            hmm.coordinates.latitude = self.gps_coord.lat
            hmm.coordinates.longitude = self.gps_coord.lon
            
            cx, cy = self.grid.get_cell_inds_from_coords(self.gps_coord)            
            
            for i in self.grid.models:
                mmh = KrigMsg()
                mmh.model_name = i.name
                #val=np.random.normal(i.output[cy][cx], np.sqrt(i.output[cy][cx])/2.0,1)
                val=np.random.poisson(i.output[cy][cx])
                mmh.measurement = val
                hmm.data.append(mmh)
            
            self.data_pub.publish(hmm)            


    def click_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            cx, cy = self.grid.get_cell_inds_from_coords(click_coord)

            if cx <0 or cy<0:
                print "click outside the grid"


    def scan_callback(self, msg):
        if msg.data == 'start_reading':
            print "starting reading"
            self.publish=True
        elif msg.data == 'stop_reading':
            print "stoping reading"
            self.publish=False


    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            self.gps_canvas.clear_image()
            self.gps_coord = MapCoords(data.latitude,data.longitude)            
            self.gps_canvas.draw_coordinate(self.gps_coord,'black',size=2, thickness=2, alpha=255)
            

    def model_comparison_cb(self, req):
        print req.model_name, req.model_index, req.height, req.width
        compm = np.reshape(np.asarray(req.vals), (req.height, req.width))
        if req.model_name == 'kriging':
            diff = self.grid.models[req.model_index].output-compm
        else :
            diff = self.grid.mean_output-compm
        return True, np.mean(diff), np.mean(diff**2), np.std(diff), np.var(diff)


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
