import utm
#import math
import numpy as np

from map_coords import MapCoords

def line_intersection(line1, line2):
    #print line1
    #print line2    
    
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return False, (-1, -1)

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    
    if x>= min(line1[0][0], line1[1][0]) and x <= max(line1[0][0], line1[1][0]):
        if x>= min(line2[0][0], line2[1][0]) and x <= max(line2[0][0], line2[1][0]):
            if  y>= min(line1[0][1], line1[1][1]) and y <= max(line1[0][1], line1[1][1]):
                if  y>= min(line2[0][1], line2[1][1]) and y <= max(line2[0][1], line2[1][1]):
                    return True, (x, y)
    
    return False, (-1, -1)


def getPArea(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))



def centroid_for_polygon(polygon, area):
    imax = len(polygon) - 1

    result_x = 0
    result_y = 0
    for i in range(0,imax):
        result_x += (polygon[i]['x'] + polygon[i+1]['x']) * ((polygon[i]['x'] * polygon[i+1]['y']) - (polygon[i+1]['x'] * polygon[i]['y']))
        result_y += (polygon[i]['y'] + polygon[i+1]['y']) * ((polygon[i]['x'] * polygon[i+1]['y']) - (polygon[i+1]['x'] * polygon[i]['y']))
    result_x += (polygon[imax]['x'] + polygon[0]['x']) * ((polygon[imax]['x'] * polygon[0]['y']) - (polygon[0]['x'] * polygon[imax]['y']))
    result_y += (polygon[imax]['y'] + polygon[0]['y']) * ((polygon[imax]['x'] * polygon[0]['y']) - (polygon[0]['x'] * polygon[imax]['y']))
    result_x /= (area * 6.0)
    result_y /= (area * 6.0)

    return {'x': result_x, 'y': result_y}



class MapPolyareas(object):

    def __init__(self, corners):
        self.corners = corners  #Polygon Vertices
        self.make_lines()
        self.width, self.height = self._get_boundbox_corners()
        self.area_size = self._calculate_area(self.corners)
        #self._get_poly_centroid()
        self._get_poly_centre()
        
    
    def _get_poly_centre(self):
        x=[]
        y=[]
        for i in self.corners:
            x.append(i.easting)
            y.append(i.northing)

        xc = np.average(np.asarray(x))
        yc = np.average(np.asarray(y))
        zone_number = self.corners[0].zone_number
        zone_letter = self.corners[0].zone_letter

        centreI = utm.to_latlon(xc, yc, zone_number, zone_letter=zone_letter)
        self.centre = MapCoords(float(abs(centreI[0])),float(abs(centreI[1])))
        
        


    def _get_poly_centroid(self):
        pol=[]
        for i in self.corners:
            d={}
            d['x']=i.easting
            d['y']=i.northing
            pol.append(d)
        
        zone_number = self.corners[0].zone_number
        zone_letter = self.corners[0].zone_letter

        centreI = centroid_for_polygon(pol, self.area_size)     
        print centreI
        centreI = utm.to_latlon(centreI['x'], centreI['y'], zone_number, zone_letter=zone_letter)
        self.centroid = MapCoords(float(centreI[0]),float(centreI[1]))
        
        
    def _calculate_area(self, corner_coords):
        ncoords=[]
        ecoords=[]
        
        for i in corner_coords:
            ncoords.append(i.northing)
            ecoords.append(i.easting)
        
        area = getPArea(np.asarray(ncoords),np.asarray(ecoords))
        return area

    def _get_boundbox_corners(self):
        
        easts = [x.easting for x in self.corners]
        norths = [x.northing for x in self.corners]
        mineast= min(easts)
        maxeast= max(easts)
        minnorth = min(norths)
        maxnorth = max(norths)

        zone_number = self.corners[0].zone_number
        zone_letter = self.corners[0].zone_letter
        
        
        corner0 = utm.to_latlon(maxeast, minnorth, zone_number, zone_letter=zone_letter)
        self.sec = MapCoords(float(corner0[0]),float(corner0[1]))
        corner1 = utm.to_latlon(mineast, maxnorth, zone_number, zone_letter=zone_letter)
        self.nwc = MapCoords(float(corner1[0]),float(corner1[1]))
        corner2 = utm.to_latlon(mineast, minnorth, zone_number, zone_letter=zone_letter)
        self.swc = MapCoords(float(corner2[0]),float(corner2[1]))
        corner3 = utm.to_latlon(maxeast, maxnorth, zone_number, zone_letter=zone_letter)
        self.nec = MapCoords(float(corner3[0]),float(corner3[1]))

        return np.ceil(maxeast-mineast), np.ceil(maxnorth-minnorth)


    def make_lines(self):
        self.limit_lines=[]
        for i in range(len(self.corners)-1):
            self.limit_lines.append((self.corners[i], self.corners[i+1]))
        self.limit_lines.append((self.corners[len(self.corners)-1], self.corners[0]))





    def _get_intersection(self, line1, line2):
        
        l1=[[line1[0].northing, line1[0].easting], [line1[1].northing, line1[1].easting]]
        l2=[[line2[0].northing, line2[0].easting], [line2[1].northing, line2[1].easting]]
        
        res, point = line_intersection(l1,l2)
        
        if res:
            #print point, line1[0]
            a = utm.to_latlon(point[1], point[0], line1[0].zone_number, line1[0].zone_letter)
            return MapCoords(a[0], a[1])
        else:
            return None

    def get_div_v_lines(self, number):
        divis=[]
        print ("Finding %d vertical lines"%number)

        #dnorth = abs(self.swc.northing - self.sec.northing)/(number+1)
        deast = abs(self.swc.easting - self.sec.easting)/(number+1)
        
        mineast = min(self.swc.easting, self.sec.easting)
        minnorth = min(self.swc.northing, self.sec.northing)
        maxnorth = max(self.nwc.northing, self.nec.northing)        
        
        zone_number = self.swc.zone_number
        zone_letter = self.swc.zone_letter
        
        a = utm.to_latlon(mineast, minnorth, zone_number, zone_letter)
        east_orig = MapCoords(a[0], a[1])
        a = utm.to_latlon(mineast, maxnorth, zone_number, zone_letter)
        north_orig = MapCoords(a[0], a[1])
        
        for i in range(number):
            pointa=east_orig._get_rel_point(deast*(i+1), -30.0)
            pointb=north_orig._get_rel_point(deast*(i+1), 30.0)
            divis.append([pointa, pointb])

        return divis


    def get_div_h_lines(self, number):
        divis=[]
        print("Finding %d horizontal lines"%number)

        dnorth = abs(self.nwc.northing - self.swc.northing)/(number+1)
        #deast = abs(self.swc.easting - self.sec.easting)/(number+1)
        
        mineast = min(self.swc.easting, self.sec.easting)
        minnorth = min(self.swc.northing, self.sec.northing)
        #maxnorth = max(self.nwc.northing, self.nec.northing)        
        maxeast = max(self.swc.easting, self.sec.easting)        
        
        zone_number = self.swc.zone_number
        zone_letter = self.swc.zone_letter
        
        a = utm.to_latlon(mineast, minnorth, zone_number, zone_letter)
        east_orig = MapCoords(a[0], a[1])
        a = utm.to_latlon(maxeast, minnorth, zone_number, zone_letter)
        west_orig = MapCoords(a[0], a[1])
        
        for i in range(number):
            dif=dnorth*(i+1)
            pointa=east_orig._get_rel_point(-100.0, dif)
            pointb=west_orig._get_rel_point(100.0, dif)
            divis.append([pointa, pointb])

        return divis
        

    def split_v_area(self, number):
        print "finding midpoints"
        areas=[]

        divilines=self.get_div_v_lines(number-1)
        divilines=divilines[:]
        divi_ini =self.get_div_v_lines(number-1)
        
        
        error=1000
        count = 0
        correction_factor=0.15
        precorrection=1.0
        
        target_area= self.area_size/number
        print target_area
        
        preeasting=-1


        for j in range(len(divilines)):
            left=[]
            first_time=True
            while abs(error)>50 and count <5000:
                del left[:]
                if j>0:
                    preeasting=divilines[j-1][0].easting
                               
                for i in self.corners:
                    if i.easting > preeasting and i.easting <= divilines[j][0].easting:
                        left.append(i)
                
                for i in self.limit_lines:
                    point = self._get_intersection(divilines[j], i)
                    if point:
                        left.append(point)
    
                if j>0:
                    for i in self.limit_lines:
                        point = self._get_intersection(divilines[j-1], i)
                        if point:
                            left.append(point)
    
                left = self._sort_corners(left)
                larea = self._calculate_area(left)
                error = (target_area-larea)
                
                if error > 0:
                    correction = 1.0*correction_factor*np.sqrt(np.sqrt(np.abs(error)))
                else:
                    correction = -1.0*correction_factor*np.sqrt(np.sqrt(np.abs(error)))
                
                if np.sign(precorrection) != np.sign(correction) and not first_time:
                    correction_factor=correction_factor*0.4
                    
                #print correction
                divilines[j][0] = divilines[j][0]._get_rel_point(correction, 0.0)
                divilines[j][1] = divilines[j][1]._get_rel_point(correction, 0.0)

                #divilines[j][0].easting = divilines[j][0].easting + correction
                #divilines[j][1].easting = divilines[j][1].easting + correction
                count+=1
                precorrection=correction

            print "iteration ", j+1, "number of corners: ", len(left), " final error: ", error, " count: ", count
            print "current easting ", divilines[j][0].easting, "previous ", preeasting  
            error=1000
            count=0             
            left = MapPolyareas(left)
            areas.append(left)


        right=[]
        for i in self.corners:
            if i.easting >= divilines[-1][0].easting:
                right.append(i)
                
        for i in self.limit_lines:
            point = self._get_intersection(divilines[-1], i)
            if point:
                right.append(point)
        
        
        right = self._sort_corners(right)
        right = MapPolyareas(right)
        areas.append(right)
        
        
        return areas#, divilines, divi_ini



    def split_h_area(self, number):
        areas=[]

        divilines=self.get_div_h_lines(number-1)
        divilines=divilines[:]
        
        error=1000
        count = 0
        correction_factor=1.0
        precorrection=1.0

        target_area= (self.area_size/number)
        print target_area
        prenorthing=-1

        for j in range(len(divilines)):
            left=[]
            first_time=True
            while abs(error)>2 and count <5000:
                del left[:]
                if j>0:
                    prenorthing=divilines[j-1][0].northing
                
                for i in self.corners:
                    if i.northing <= divilines[j][0].northing and i.northing>prenorthing:
                            left.append(i)
                
                for i in self.limit_lines:
                    point = self._get_intersection(divilines[j], i)
                    if point:
                        left.append(point)
    
                if j>0:
                    for i in self.limit_lines:
                        point = self._get_intersection(divilines[j-1], i)
                        if point:
                            left.append(point)
    
                left = self._sort_corners(left)
                larea = self._calculate_area(left)
                error = (target_area-larea)
                
                if error > 0:
                    correction = correction_factor*np.sqrt(np.sqrt(np.abs(error)))
                else:
                    correction = -1.0*correction_factor*np.sqrt(np.sqrt(np.abs(error)))
                
                if np.sign(precorrection) != np.sign(correction) and not first_time:
                    correction_factor=correction_factor*0.9
                    
#                print correction, error
                #divilines[j][0] = divilines[j][0]._get_rel_point(0, correction)
                divilines[j][0].northing = divilines[j][0].northing + correction
                #divilines[j][1] = divilines[j][1]._get_rel_point(0, correction)
                divilines[j][1].northing = divilines[j][1].northing + correction
                count+=1
                precorrection=correction
                first_time=False
    
            print "iteration ", j+1, "number of corners: ", len(left), " final error: ", error, " count: ", count
            print "current northing ", divilines[j][0].northing, "previous ", prenorthing
            error=1000
            count=0 
            left = MapPolyareas(left)
            areas.append(left)

        right=[]
        for i in self.corners:
            if i.northing >= divilines[-1][0].northing:
                right.append(i)

        for i in self.limit_lines:
            point = self._get_intersection(divilines[-1], i)
            if point:
                right.append(point)

        right = self._sort_corners(right)
        right = MapPolyareas(right)
        areas.append(right)
        
        return areas


    def _sort_corners(self, polygon):
        polygon2=[] #polygon[:]
        angles = []
        mde = np.average([x.easting for x in polygon])
        mdn = np.average([x.northing for x in polygon])

        a = utm.to_latlon(mde, mdn, polygon[0].zone_number, polygon[0].zone_letter)
        mda = MapCoords(a[0], a[1])
        
        for i in polygon:
            rr= mda - i
            angles.append(rr[1]+180)
            
        angles2=angles[:]
        angles.sort()        

        for i in angles:
            ind = angles2.index(i)
            polygon2.append(polygon[ind])
        
        return polygon2
