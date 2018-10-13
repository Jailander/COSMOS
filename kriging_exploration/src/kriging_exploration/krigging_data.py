#import utm
import numpy as np
import time
from pykrige.ok import OrdinaryKriging
from kriging_exploration.poisson_kriging import PoissonKriging
#from map_coords import MapCoords


class KriggingDataPoint(object):
    def __init__(self, coords, cell_coord, value, rate=0):
        self.coord = coords
        self.x = cell_coord[0]
        self.y = cell_coord[1]
        self.value = value
        self.rate = rate
        self.kriged=False



class KriggingData(object):
    def __init__(self, shape, name='default', poisson_rate=60.0):
        self.name= name
        self.shape = shape
        #self.lims = lims
        self.rate = poisson_rate
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


    def add_poisson_data(self, data, rate):
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
        self.variogram_model='linear'
        #self.variogram_model='gaussian'
        #print datablah
        #OK = OrdinaryKriging(datablah[:, 0], datablah[:, 1], datablah[:, 2], variogram_model='spherical', verbose=False, enable_plotting=False)
        #OK = OrdinaryKriging(datablah[:, 0], datablah[:, 1], datablah[:, 2], variogram_model='linear', verbose=True, enable_plotting=False)
        OK = OrdinaryKriging(datablah[:, 0], datablah[:, 1], datablah[:, 2], variogram_model=self.variogram_model, verbose=True, enable_plotting=False)        
        
        print "OK Done"
        print OK.variogram_model_parameters
        if self.variogram_model == 'linear':
            self.Sill= OK.variogram_model_parameters[0]
            self.Range= OK.variogram_model_parameters[1]
            if self.Range <= 0.001:
                self.Range = 0.001
            #self.Nugget= OK.variogram_model_parameters[2]
        else:
            self.Sill= OK.variogram_model_parameters[0]
            self.Range= OK.variogram_model_parameters[1]
            self.Nugget= OK.variogram_model_parameters[2]
            
        self.sim_sill=np.mean(OK.semivariance[2:])
        #OK.display_variogram_model()

        try:
            # Creates the kriged grid and the variance grid. Allows for kriging on a rectangular
            # grid of points, on a masked rectangular grid of points, or with arbitrary points.
            # (See OrdinaryKriging.__doc__ for more information.)
            z, ss = OK.execute('grid', self.gridx, self.gridy)

#            print "Check HERE:"        
#            print self.shape            
#            print self.gridx
#            
#            print "----------------"


            self.kriged=True
        except:
            print "this failed at: ", self.name
            ss=np.full(self.shape,0,dtype=np.float64)
            z=np.full(self.shape,np.average(datablah[:, 2]),dtype=np.float64)
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
        self.avg_var = np.average(self.variance)


        self.min_val = np.min(self.output)
        self.max_val = np.max(self.output)
        
        self.min_dev = np.min(self.deviation)
        self.max_dev = np.max(self.deviation)
        

        
    def do_poisson_krigging(self):
        datablah=[]
        for i in self.orig_data:
#            datablah.append([i.x, i.y, (i.value*self.rate), 100*np.sqrt(i.value*self.rate)/(i.value*self.rate)])
            datablah.append([i.x, i.y, i.rate, np.sqrt(i.value/i.rate)])
        
        datablah = np.asarray(datablah)
                       
        print "OK"
#        self.variogram_model='linear'
        self.variogram_model='gaussian'
        #print datablah
        #OK = OrdinaryKriging(datablah[:, 0], datablah[:, 1], datablah[:, 2], variogram_model='spherical', verbose=False, enable_plotting=False)
        #OK = OrdinaryKriging(datablah[:, 0], datablah[:, 1], datablah[:, 2], variogram_model='linear', verbose=True, enable_plotting=False)
        OK = OrdinaryKriging(datablah[:, 0], datablah[:, 1], datablah[:, 2], variogram_model=self.variogram_model, verbose=True, enable_plotting=False)        
        
        print "OK Done"
        print datablah[:, 2]
        print OK.variogram_model_parameters
        if self.variogram_model == 'linear':
            self.Sill= OK.variogram_model_parameters[0]
            self.Range= OK.variogram_model_parameters[1]
            if self.Range <= 0.001:
                self.Range = 0.001
            #self.Nugget= OK.variogram_model_parameters[2]
        else:
            self.Sill= OK.variogram_model_parameters[0]
            self.Range= OK.variogram_model_parameters[1]
            self.Nugget= OK.variogram_model_parameters[2]
            
        self.sim_sill=np.mean(OK.semivariance[2:])
        #OK.display_variogram_model()

        try:
            z, ss = OK.execute('grid', self.gridx, self.gridy)
            self.kriged=True
        except:
            print "1. this failed at: ", self.name
            ss=np.full(self.shape,0,dtype=np.float64)
            z=np.full(self.shape,np.average(datablah[:, 2]),dtype=np.float64)
            self.kriged=False
        
                
        self.output = z#/self.rate
        #base_var = np.sqrt(np.abs(ss))
        base_var = ss


        #print base_var
        
            
        measure_var = self._get_poisson_var(datablah)
        
        #print measure_var
        #print measure_var
        self.variance = 2*np.abs(base_var)  + np.abs(measure_var)

        for i in self.orig_data:
            self.variance[i.y][i.x]= abs(self.variance[i.y][i.x])


        self.deviation=np.sqrt(self.variance)
        #self.sigmapercent =  self.deviation/self.output

        
        #self.min_var = np.min([np.nonzero(self.variance)])
        self.min_var = np.min(self.variance)
        self.max_var = np.max(self.variance)
        self.avg_var = np.average(self.variance)


        self.min_val = np.min(self.output)
        self.max_val = np.max(self.output)
        
        self.min_dev = np.min(self.deviation)
        self.max_dev = np.max(self.deviation)
        


    def _get_poisson_var(self, datablah):
        OK2 = OrdinaryKriging(datablah[:, 0], datablah[:, 1], datablah[:, 3], variogram_model='exponential', verbose=True, enable_plotting=False)
        print "++++++++++++++++++++++++++++++"        
        print OK2.variogram_model_parameters
        print datablah[:, 3]
        try:
            z, ss = OK2.execute('grid', self.gridx, self.gridy)
            self.kriged=True
        except:
            print "2. this failed at: ", self.name
            print datablah[:, 3]
#            ss=np.full(self.shape,0,dtype=np.float64)
            z=np.full(self.shape,np.average(datablah[:, 3]),dtype=np.float64)
            
            for i in datablah:
                z[i[0]][i[1]]=i[3]
            self.kriged=False

        del OK2
        return z
