import utm
import yaml

import numpy as np

from map_coords import MapCoords
from krigging_data import KriggingDataPoint
from krigging_data import KriggingData
from map_polyareas import MapPolyareas


def PolyArea(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

class DataGrid(object):
    def __init__(self, limits_file, cell_size, limit_list=[]):
        self.limits=[]
        self.limit_lines=[] # Limit side lines
        self.area_splits=[]
        self.area_splits_coords=[]
        self.div_v_lines=[] # Vertical division lines
        self.areas=[]        
        
        self.corners=[]
        self.cells=[]       #coordinates of the center of each cell

        if limits_file:
            self._load_limits(limits_file)
        else:
            self.set_limits(limit_list)
            
        self.create_grid(cell_size)

        self.models=[]


    def krieg_all_mmodels(self):
        for i in self.models:
            i.do_krigging()


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
                    if cx >= 0 and cy >= 0:
                        dt.append(KriggingDataPoint(b,(cx,cy),j['data'][i]))
                        #print cx, cy
                    #print b
                    #print j['data'][i]
            kd.add_data(dt)
            self.models.append(kd)
    
    def add_data_point(self, model_name, coord, value):
        
        model_names=[x.name for x in self.models]
        
        #print model_name
        
#        print coord
        #print value
        
        if model_name not in model_names:
            a = KriggingData(self.shape, model_name)
            self.models.append(a)
            model_names=[x.name for x in self.models]
            
        modind = model_names.index(model_name)
        
        cx, cy = self.get_cell_inds_from_coords(coord)
        self.models[modind].add_data(KriggingDataPoint(coord,(cx,cy), value))
        
    
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
        if cx >= self.shape[1] or cx<0 or cy >= self.shape[0] or cy<0:
            cx =-1
            cy =-1
        return cx, cy
    
   
    
    def set_limits(self, limits):
        self.limits = limits
        for i in range(len(self.limits)-1):
            self.limit_lines.append((self.limits[i], self.limits[i+1]))
        
        self.limit_lines.append((self.limits[len(self.limits)-1], self.limits[0]))
        self.area = MapPolyareas(self.limits)



    def calculate_area(self, corner_coords):
        ncoords=[]
        ecoords=[]
        
        for i in corner_coords:
            ncoords.append(i.northing)
            ecoords.append(i.easting)
        
        area = PolyArea(np.asarray(ncoords),np.asarray(ecoords))
        return area


    def _split_area(self, vertical_splits, horizontal_splits):
        areas = self.area.split_v_area(vertical_splits)

        for j in areas:
            aa = j.split_h_area(horizontal_splits)
            for h in aa:
                self.area_splits.append(h)

        self.area_splits_coords = [x.centre for x in self.area_splits]
        print "AREA SPLITS:", len(self.area_splits_coords)
        print self.area_splits_coords
        



    def calculate_mean_grid(self):
        self.mean_output=np.full(self.shape,0,dtype=np.float64)
        self.mean_variance=np.full(self.shape,0,dtype=np.float64)
        self.mean_deviation=np.full(self.shape,0,dtype=np.float64)
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):   
                ou1=np.asarray([x.output[i][j] for x in self.models])
                if np.isnan(ou1).any():
                    print("NAN in outputs!!", ou1, i, j)
                self.mean_output[i][j]=np.mean(ou1)
                
                var1=np.asarray([x.variance[i][j] for x in self.models])
                if np.isnan(ou1).any():
                    print("NAN in variances!!", var1, i, j)
                self.mean_variance[i][j]=np.mean(var1)
                
                dv1=np.asarray([x.deviation[i][j] for x in self.models])
                if np.isnan(dv1).any():
                    print("NAN in devs!!", dv1, i, j)
                self.mean_deviation[i][j]=np.mean(dv1)
        
        
        self.min_mean_output = np.min(self.mean_output)
        self.max_mean_output = np.max(self.mean_output)
        self.min_mean_variance = np.min(self.mean_variance)
        self.max_mean_variance = np.max(self.mean_variance)
        self.min_mean_deviation = np.min(self.mean_deviation)
        self.max_mean_deviation = np.max(self.mean_deviation)
        
        print "-----"
        if np.isnan(self.min_mean_deviation) or np.isnan(self.max_mean_deviation):
            print "CAUTION!!!"
            print self.mean_deviation
            
        #print self.min_mean_deviation, self.max_mean_deviation
        #print self.mean_deviation
        print "-----"

    def get_max_min_vals(self):
        self.valmin = np.floor(np.min([x.lims[0] for x in self.models]))
        self.valmax = np.ceil(np.max([x.lims[1] for x in self.models]))
        return self.valmin, self.valmax

    
    def create_grid(self, cell_size):
        self.cell_size = cell_size
        deasting, dnorthing = self._get_grid_corners()
        print cell_size
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



#    def _get_intersection(self, line1, line2):
#        
#        l1=[[line1[0].northing, line1[0].easting], [line1[1].northing, line1[1].easting]]
#        l2=[[line2[0].northing, line2[0].easting], [line2[1].northing, line2[1].easting]]
#        
#        res, point = line_intersection(l1,l2)
#        
#        if res:
#            #print point, line1[0]
#            a = utm.to_latlon(point[1], point[0], line1[0].zone_number, line1[0].zone_letter)
#            return MapCoords(a[0], a[1])
#        else:
#            return None

#    def _sort_corners(self, polygon):
#        polygon2=[] #polygon[:]
#        angles = []
#        mde = np.average([x.easting for x in polygon])
#        mdn = np.average([x.northing for x in polygon])
#
#        a = utm.to_latlon(mde, mdn, polygon[0].zone_number, polygon[0].zone_letter)
#        mda = MapCoords(a[0], a[1])
#        
#        for i in polygon:
#            rr= mda - i
#            angles.append(rr[1]+180)
#            
#        angles2=angles[:]
#        angles.sort()        
#
#        for i in angles:
#            ind = angles2.index(i)
#            polygon2.append(polygon[ind])
#        
#        
#        
#        return polygon2
        
        
        


#    def divide_area(self, number):
#        print "finding midpoints"
#        blah = self.swc % self.sec
#        blah2 = self.nwc % self.nec
#
#        blah = blah._get_rel_point(0, -10)
#        blah2 = blah2._get_rel_point(0, 10)        
#        
#        diviline = (blah, blah2)
#        
#
#        error=1000
#        count = 0
#                
#        while abs(error)>10 and count <50:
#            left=[]
#            right=[]
#            
#            for i in self.limits:
#                if i.easting > blah.easting:
#                    left.append(i)
#                else:
#                    right.append(i)
#            
#            for i in self.limit_lines:
#                point = self._get_intersection(diviline, i)
#                if point:
#                    left.append(point)
#                    right.append(point)
#
#            left = self._sort_corners(left)
#            
#            right = self._sort_corners(right)
#            
#
#            larea = self.calculate_area(left)
#            rarea = self.calculate_area(right)
#            
#            error = (rarea-larea)
#            print count, " L AREA: ", larea, " R AREA: ", rarea, " Error: ", error
#            
#            if error > 0:
#                blah = blah._get_rel_point(-1, 0)
#                blah2 = blah2._get_rel_point(-1, 0)
#            else:
#                blah = blah._get_rel_point(1, 0)
#                blah2 = blah2._get_rel_point(1, 0)
#            
#            count+=1
#            diviline = (blah, blah2)
#            
#        self.div_v_lines.append(diviline)
#        self.areas.append(right)
#        self.areas.append(left)
        

