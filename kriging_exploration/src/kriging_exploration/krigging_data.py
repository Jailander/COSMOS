#import utm
import numpy as np

from pykrige.ok import OrdinaryKriging
import pykrige.kriging_tools as kt
#from map_coords import MapCoords


class KriggingDataPoint(object):
    def __init__(self, coords, cell_coord, value):
        self.coord = coords
        self.x = cell_coord[0]
        self.y = cell_coord[1]
        self.value = value
        self.kriged=False



class KriggingData(object):
    def __init__(self, shape, name='default'):
        self.name= name
        self.shape = shape
        #self.lims = lims
        
        self.orig_data=[]
        self.gridx = np.arange(0.0, shape[1], 1)
        self.gridy = np.arange(0.0, shape[0], 1)
            
    def add_data(self, data):
        if hasattr(data, '__iter__'):
            for i in data:
                self.orig_data.append(i)
        else:
            self.orig_data.append(data)
        
        vals = [x.value for x in self.orig_data]
        self.lims = [np.min(vals), np.max(vals)]
        print self.lims

    def do_krigging(self):
        datablah=[]
        for i in self.orig_data:
            datablah.append([i.x, i.y, i.value])
        
        datablah = np.asarray(datablah)
                       
        print "OK"
        OK = OrdinaryKriging(datablah[:, 0], datablah[:, 1], datablah[:, 2], variogram_model='linear', verbose=False, enable_plotting=False)
        print "OK Done"

        try:
            # Creates the kriged grid and the variance grid. Allows for kriging on a rectangular
            # grid of points, on a masked rectangular grid of points, or with arbitrary points.
            # (See OrdinaryKriging.__doc__ for more information.)
            z, ss = OK.execute('grid', self.gridx, self.gridy)
            self.kriged=True
        except:
            print "this failed at: ", self.name
            ss=np.full(self.shape,-1,dtype=np.float64)
            z=np.full(self.shape,-1,dtype=np.float64)
            self.kriged=False
        
        self.output = z
        
        self.variance = ss

        for i in self.orig_data:
            self.variance[i.y][i.x]= abs(self.variance[i.y][i.x])


        self.deviation=np.sqrt(self.variance)
        #self.sigmapercent =  self.deviation/self.output

        
        #self.min_var = np.min([np.nonzero(self.variance)])
        self.min_var = np.min(self.variance)
        self.max_var = np.max(self.variance)

        self.min_val = np.min(self.output)
        self.max_val = np.max(self.output)
        
        self.min_dev = np.min(self.deviation)
        self.max_dev = np.max(self.deviation)
        

