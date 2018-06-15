import utm
import math


class MapCoords(object):

    def __init__(self, lat, lon):
        self.lat =lat
        self.lon =lon
        a = utm.from_latlon(lat, lon)
        self.northing = a[1]
        self.easting = a[0]
        self.zone_number = a[2]
        self.zone_letter = a[3]

    def _get_rel_point(self, easting, northing):
        a = utm.to_latlon(self.easting + easting, self.northing + northing, self.zone_number, self.zone_letter)
        return MapCoords(a[0], a[1])

    def __sub__(self, other):
        dnorth = self.northing - other.northing
        deast = self.easting - other.easting
        dist = math.hypot(deast, dnorth)
        orient = math.atan2(dnorth, deast) * 180/math.pi
        #return dist, orient
        return dist, orient#, deast, dnorth


    def __mod__(self, other):
        dnorth = ((self.northing + other.northing)/2)
        deast = ((self.easting + other.easting)/2)
        a = utm.to_latlon(deast, dnorth, self.zone_number, self.zone_letter)
        return MapCoords(a[0], a[1])        
        
#        print dnorth, deast
#        midpoint = self._get_rel_point(deast, dnorth)
#        return midpoint


    def __repr__(self):
        a = dir(self)
        b = []
        s = ''
        for i in a:
            if not i.startswith('_'):
                b.append(str(i))
        for i in b:
                s = s + str(i) + ': ' + str(self.__getattribute__(i)) + '\n'
        return s


def coord_from_satnav_fix(msg):
    a = MapCoords(msg.latitude, msg.longitude)
    return a
