#import numpy as np
import cv2
from matplotlib import colors as mcolors

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
        self.refresh_image()


    def create_grid(self, cell_size, limit_list):
        self.grid = DataGrid(None, cell_size, limit_list=limit_list)
        self.cell_size=cell_size
        self.grid_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)

    def draw_grid(self):
        self.grid_canvas.draw_grid(self.grid.cells, self.cell_size, (72,72,72,128), thickness=1)
        self.grid_canvas.draw_polygon(self.grid.limits, (0,0,255,208), thickness=2)

    def refresh_image(self):
        self.image = cv2.addWeighted(self.canvas.image, 0.5, self.base_image, 0.9, 0)


    def load_groundtruth_data(self, filename):
        self.grid.load_data_from_yaml(filename)
        self.vmin, self.vmax = self.grid.get_max_min_vals()
        print "LIMS: " + str(self.vmin) + " " + str(self.vmax)
        self.n_models=len(self.grid.models)
        self.current_model=0


    def draw_coordinate(self, lat, lon, colour='white', size=6, thickness=2, alpha=128):
        a = MapCoords(lat, lon)
        self.canvas.draw_coordinate(a,colour,size=size, thickness=thickness)
        self.refresh_image()

