import utm
import yaml

import numpy as np

from map_coords import MapCoords
from krigging_data import KriggingDataPoint
from krigging_data import KriggingData

class DataGrid(object):
    def __init__(self, limits_file, cell_size):
        self.limits=[]
        self.corners=[]
        self.cells=[]

        self._load_limits(limits_file)
        self.create_grid(cell_size)

        self.models=[]


    def _load_limits(self, limits_fn):
        limits=[]
        f = open(limits_fn, 'r')
        for line in f:
            line=line.strip('\n')
            a=line.split(',')
            limits.append(MapCoords(float(a[0]),float(a[1])))
        self.set_limits(limits)
    
    
    def load_data_from_yaml(self, filename):
        with open(filename, 'r') as f:
            a = yaml.load(f)
            
        for i in range(len(a['names'])):
            print "---------------------------------------"
            print "creating model " + str(i) + " of " + str(len(a['names'])) + " " + a['names'][i]
            kd = KriggingData(self.shape, a['names'][i])
            dt = []
            for j in a['data']:
                if j['data'][i] >0:
                    #print j['position']
                    b = MapCoords(j['position']['lat'],j['position']['lon'])
                    cx, cy = self.get_cell_inds_from_coords(b)
                    dt.append(KriggingDataPoint(b,(cx,cy),j['data'][i]))
                    print cx, cy
                    #print b
                    print j['data'][i]
            kd.add_data(dt)
            self.models.append(kd)
                    
    
    def _load_model_from_file(self, data_fn, name='default'):
        data=[]
        vals=[]
        print "open: " + data_fn
        f = open(data_fn, 'r')
        for line in f:
            line=line.strip('\n')
            a=line.split(';')
            b=MapCoords(float(a[0]),float(a[1]))
#            cx = int(np.floor((b.easting - self.swc.easting)/self.cell_size))
#            cy = int(np.floor(((b.northing - self.swc.northing)/self.cell_size)))
            cx, cy = self.get_cell_inds_from_coords(b)
            data.append(KriggingDataPoint(b,(cx,cy),float(a[2])))
            vals.append(float(a[2]))

        
        #lims= [np.min(vals), np.max(vals)]
        #a = KriggingData(self.shape, lims, name)
        a = KriggingData(self.shape, name)
        a.add_data(data)
        self.models.append(a)
        #print len(self.data_coords), self.data_coords[0]
        
    def get_cell_inds_from_coords(self, point):
        cx = int(np.floor((point.easting - self.swc.easting)/self.cell_size))
        cy = int(np.floor(((point.northing - self.swc.northing)/self.cell_size)))
        return cx, cy
    

#    def get_cell_inds_from_pix(self, x, y):
#        return x, y
    
    
    def set_limits(self, limits):
        self.limits = limits


    def get_max_min_vals(self):
        self.valmin = np.floor(np.min([x.lims[0] for x in self.models]))
        self.valmax = np.ceil(np.max([x.lims[1] for x in self.models]))
        return self.valmin, self.valmax

    
    def create_grid(self, cell_size):
        self.cell_size = cell_size
        deasting, dnorthing = self._get_grid_corners()
        dnorthing = int(np.ceil(dnorthing/cell_size))
        deasting = int(np.ceil(deasting/cell_size))
        for i in range(0, dnorthing):
            ab=[]
            for j in range(0, deasting):
                ab.append(self.swc._get_rel_point((j*cell_size)+(cell_size/2),(i*cell_size)+(cell_size/2)))
            self.cells.append(ab)
        np_cells = np.asarray(self.cells)
        self.shape= np_cells.shape

    def _get_grid_corners(self):
        self.grid=[]
        mineast = self.limits[0].easting
        minnorth = self.limits[0].northing
        maxeast = self.limits[0].easting
        maxnorth = self.limits[0].northing
        zone_number = self.limits[0].zone_number
        zone_letter = self.limits[0].zone_letter
        for i in self.limits:
            if i.easting < mineast:
                mineast = i.easting
            if i.northing < minnorth:
                minnorth = i.northing
            if i.easting > maxeast:
                maxeast = i.easting
            if i.northing > maxnorth:
                maxnorth = i.northing
        
        corner0 = utm.to_latlon(maxeast, minnorth, zone_number, zone_letter=zone_letter)
        self.sec = MapCoords(float(corner0[0]),float(corner0[1]))
        corner1 = utm.to_latlon(mineast, maxnorth, zone_number, zone_letter=zone_letter)
        self.nwc = MapCoords(float(corner1[0]),float(corner1[1]))
        corner2 = utm.to_latlon(mineast, minnorth, zone_number, zone_letter=zone_letter)
        self.swc = MapCoords(float(corner2[0]),float(corner2[1]))
        corner3 = utm.to_latlon(maxeast, maxnorth, zone_number, zone_letter=zone_letter)
        self.nec = MapCoords(float(corner3[0]),float(corner3[1]))

        self.corners.append(corner0)
        self.corners.append(corner1)
        self.corners.append(corner2)
        self.corners.append(corner3)        

        return np.ceil(maxeast-mineast), np.ceil(maxnorth-minnorth)