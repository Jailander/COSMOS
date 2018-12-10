import numpy as np
import cv2
import rospy

import matplotlib as mpl
import matplotlib.cm as cm
#from matplotlib import colors as mcolors

from sensor_msgs.msg import NavSatFix

from kriging_exploration.satellite import SatelliteImage
from kriging_exploration.map_coords import MapCoords
from kriging_exploration.canvas import ViewerCanvas

from kriging_exploration.data_grid import DataGrid


class SimpleKrigingVisualiser(object):

    def __init__(self, lat_deg, lon_deg, zoom, size):
        self.satellite = SatelliteImage(lat_deg, lon_deg, zoom, size)
        self.base_image = self.satellite.base_image.copy()
        self.canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.centre = self.satellite.centre
        self.refresh_image()

        
    def refresh_image(self):
        self.image = cv2.addWeighted(self.canvas.image, 0.5, self.base_image, 0.9, 0)

        
    def draw_coordinate(self, lat, lon, colour='white', size=6, thickness=2, alpha=128):
        a = MapCoords(lat, lon)
        self.canvas.draw_coordinate(a,colour,size=size, thickness=thickness)
        self.refresh_image()


class KrigingVisualiser(object):

    
    def __init__(self, lat_deg, lon_deg, zoom, size):
        self.satellite = SatelliteImage(lat_deg, lon_deg, zoom, size)
        self.base_image = self.satellite.base_image.copy()
        self.canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.centre = self.satellite.centre
        
        self.model_canvas=[]
        self.kriging_canvas=[]
        self.sigma_canvas=[]
        self.model_canvas_names=[]
        
        self.refresh_image()


    def create_grid(self, cell_size, limit_list):
        self.grid = DataGrid(None, cell_size, limit_list=limit_list)
        self.cell_size=cell_size
        self.grid_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)


    def draw_topo_map(self, waypoints, colour='white', size=5, thickness=1):
        self.topo_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        for i in waypoints:
            self.topo_canvas.draw_coordinate(i.coord, colour, size=size, thickness=thickness)


    def draw_grid(self):
        #self.grid_canvas.draw_grid(self.grid.cells, self.cell_size, (72,72,72,128), thickness=1)
        self.grid_canvas.draw_polygon(self.grid.limits, (0,0,255,208), thickness=2)


    def refresh_image(self):
        self.image = cv2.addWeighted(self.canvas.image, 0.5, self.base_image, 0.9, 0)


    def load_groundtruth_data(self, filename):
        self.grid.load_data_from_yaml(filename)
        self.vmin, self.vmax = self.grid.get_max_min_vals()
        print "LIMS: " + str(self.vmin) + " " + str(self.vmax)
        self.n_models=len(self.grid.models)
        self.current_model=0

        for i in range(len(self.grid.models)):
            self.model_canvas_names.append(self.grid.models[i].name)
            self.model_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
            self.sigma_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
            self.kriging_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))

        print "Models: "
        print self.model_canvas_names
        

    def add_canvases(self):
        self.n_models=len(self.grid.models)
        
        for i in self.grid.models:
            if i.name not in self.model_canvas_names:       
                self.model_canvas_names.append(i.name)
                self.model_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.sigma_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.kriging_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
 #            self.draw_inputs(self.model_canvas_names.index(i.name))
        
        
    def draw_coordinate(self, lat, lon, colour='white', size=6, thickness=2, alpha=128):
        a = MapCoords(lat, lon)
        self.canvas.draw_coordinate(a,colour,size=size, thickness=thickness)
        self.refresh_image()


    def draw_inputs(self, nm, alpha=50):
        minv =  self.grid.models[nm].lims[0]
        maxv =  self.grid.models[nm].lims[1]

        if (maxv-minv) <=1:
            maxv = maxv + 50
            minv = minv - 50
            
        norm = mpl.colors.Normalize(vmin=minv, vmax=maxv)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.model_canvas[nm].clear_image()
        #self.model_legend[nm].clear_image()
        
        for i in self.grid.models[nm].orig_data:
            cell = self.grid.cells[i.y][i.x]
            a= colmap.to_rgba(int(i.value))                
            b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*alpha))
            self.model_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)
            self.model_canvas[nm].put_text(self.grid.models[nm].name, colour=(255,255,255,255), x_or=220, y_or=20)
        
        self.model_canvas[nm].draw_legend(minv, maxv, colmap, title="Inputs")


    def draw_krigged(self, nm, alpha=50):
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
        #self.klegend_canvas[nm].clear_image()
        
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].output[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*alpha))    
                self.kriging_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)
        
        self.kriging_canvas[nm].draw_legend(minv, maxv, colmap, title="Kriging")
#        self.redraw()


    def draw_variance(self, nm, alpha=50):
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
#        self.klegend2_canvas[nm].clear_image()
        
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].variance[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*alpha))
                self.sigma_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)

#        self.klegend2_canvas[nm].put_text(self.grid.models[nm].name)
        self.sigma_canvas[nm].draw_legend(minv, maxv, colmap, title="Variance")
#        self.redraw()


    def add_traj_canvas(self):
        self.traj_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)


    def add_gps_canvas(self):
        self.gps_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)

