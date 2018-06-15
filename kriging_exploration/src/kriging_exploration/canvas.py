#import urllib
#import math
import numpy as np
import cv2
from matplotlib import colors as mcolors

from map_coords import MapCoords


class ViewerCanvas(object):

    def __init__(self, shape, centre, res):
        self.size = shape[0]
        self.res = res
        self.centre = centre
        self.image = np.zeros(shape, dtype=np.uint8)
        self.shape = shape

        
    def _coord2pix(self, point):
        dnorth = ((self.centre.northing- point.northing)/self.res) + (self.size/2)
        deast = ((point.easting - self.centre.easting)/self.res)  + (self.size/2)
        return deast, dnorth        
        
    def _pix2coord(self, x, y):
        xcord = (x - (self.size/2))*self.res
        ycord = -(y - (self.size/2))*self.res
        click_coord = self.centre._get_rel_point(xcord,ycord)
        return click_coord
        
    def _latlong2pix(self, lat, lon):
        point = MapCoords(lat, lon)
        deast, dnorth = self._coord2pix(point)
        return deast, dnorth

    def clear_image(self):
        self.image = np.zeros(self.shape, dtype=np.uint8)


    def draw_coordinate(self, coord, colour, size=6, thickness=2, alpha=128, shape='circle'):
        mx, my =self._coord2pix(coord)        
        b = [int(255*x) for x in mcolors.hex2color(mcolors.cnames[colour])]
        b = b[::-1]
        b.append(alpha)
        if shape == 'rect':
            cv2.rectangle(self.image, (int(mx)-size, int(my)-size), (int(mx)+size, int(my)+size), b, thickness)
        else:
            cv2.circle(self.image, (int(mx), int(my)), size, b, thickness)


    def draw_waypoints(self, waypoints, colour,thickness=2):
        for i in waypoints:
            mx0, my0 = self._coord2pix(i.coord)
            cv2.circle(self.image, (int(mx0), int(my0)), 2, colour, thickness=thickness)


    def draw_plan(self, waypoints, colour, thickness=2, alpha=128):
        b = [int(255*x) for x in mcolors.hex2color(mcolors.cnames[colour])]
        b = b[::-1]
        b.append(alpha)
        
        for i in range(0, len(waypoints)-1):
            mx0, my0 =self._coord2pix(waypoints[i].coord)
            mx1, my1 =self._coord2pix(waypoints[i+1].coord)
            cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), b, thickness=thickness)
        
        
    def draw_grid(self, grid, cell_size, colour, thickness=2):
        nx = len(grid)-1
        ny = len(grid[0])-1
        for i in range(0, len(grid[0])):
            mx0, my0 =self._coord2pix(grid[0][i]._get_rel_point(-cell_size/2,-cell_size/2))
            mx1, my1 =self._coord2pix(grid[nx][i]._get_rel_point(-cell_size/2,cell_size/2))
            cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)

        mx0, my0 =self._coord2pix(grid[0][ny]._get_rel_point(cell_size/2,-cell_size/2))
        mx1, my1 =self._coord2pix(grid[nx][ny]._get_rel_point(cell_size/2,cell_size/2))
        cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)

        for i in range(0, len(grid)):
            mx0, my0 =self._coord2pix(grid[i][0]._get_rel_point(-cell_size/2,-cell_size/2))
            mx1, my1 =self._coord2pix(grid[i][ny]._get_rel_point(cell_size/2,-cell_size/2))
            cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)

        mx0, my0 =self._coord2pix(grid[nx][0]._get_rel_point(-cell_size/2,cell_size/2))
        mx1, my1 =self._coord2pix(grid[nx][ny]._get_rel_point(cell_size/2,cell_size/2))
        cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        

    def draw_cell(self, cell, cell_size, colour, thickness=2):
        mx0, my0 =self._coord2pix(cell._get_rel_point(-cell_size/2,-cell_size/2))
        mx1, my1 =self._coord2pix(cell._get_rel_point(cell_size/2,cell_size/2))
        cv2.rectangle(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        

    def draw_list_of_coords(self, list_of_coords, colour, size=6, thickness=2):
        for i in list_of_coords:
            mx, my = self._coord2pix(i)
            cv2.circle(self.image, (int(mx), int(my)), size, colour, thickness)
    
    def draw_polygon(self, list_of_coords, colour, thickness=2):
        for i in range(1, len(list_of_coords)):
            mx0, my0 =self._coord2pix(list_of_coords[i-1])
            mx1, my1 =self._coord2pix(list_of_coords[i])
            cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        mx0, my0 =self._coord2pix(list_of_coords[0])
        cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
    
    
    
    def draw_line(self, line, colour, size=3, thickness=2, alpha=128):
        mx1, my1 =self._coord2pix(line[0])
        mx2, my2 =self._coord2pix(line[1])                
        b = [int(255*x) for x in mcolors.hex2color(mcolors.cnames[colour])]
        b = b[::-1]
        b.append(alpha)
        
        cv2.circle(self.image, (int(mx1), int(my1)), size, b, thickness)
        cv2.circle(self.image, (int(mx2), int(my2)), size, b, thickness)
        cv2.line(self.image, (int(mx1), int(my1)), (int(mx2), int(my2)), b, thickness=thickness)
    

    def draw_legend(self, vmin, vmax, colmap, title="OUTPUTS"):
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        if (vmax-vmin) > 1:
            step = float(vmax - vmin)/float(490-150)#float(600-40)
        else:
            vmax = vmax + 500
            vmin = vmin - 500
            step = float(vmax - vmin)/float(490-150)#float(600-40)
                   
        if step>1.0:
            ind = 0
            while ind < 340:
#                print int(vmin+(ind*step))
                a= colmap.to_rgba(int(vmin+(ind*step)))                
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255),int(a[3]*255))                
                #cv2.rectangle(self.image, (int(ind+40), int(510)), (int(ind+1+40), int(530)), b , thickness=-1)
                cv2.rectangle(self.image, (int(ind+150), int(490)), (int(ind+1+150), int(510)), b , thickness=-1)
                ind+=1
        else:
            step=1/step
            ind = 0
            while ind < 340:
#                print int(vmin+(ind*step))
                a= colmap.to_rgba(int(vmin+(ind/step)))                
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255),int(a[3]*255))                
                cv2.rectangle(self.image, (int(ind+150), int(490)), (int(ind+1+150), int(510)), b , thickness=-1)
                ind+=1

        a= colmap.to_rgba(int(vmin))
        b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*250))
        cv2.putText(self.image, str(np.floor(vmin)) + " Kpa", (int(150), int(480)), font, 0.6, b, 2)
        a= colmap.to_rgba(int(vmax))
        b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*250))
        tsz=cv2.getTextSize(str(np.ceil(vmax)) + " Kpa", font, 0.6, 2)
        cv2.putText(self.image, str(np.ceil(vmax)) + " Kpa", (int(490)-tsz[0][0]-5, int(480)), font, 0.6, b, 2)
        
#        cv2.putText(self.image, title, (230,175), font, 0.8, (220, 220, 220,255), 2)
        cv2.putText(self.image, title, (250,250), font, 0.8, (220, 220, 220,255), 2)    
    
    def put_text(self,text,colour=(230,230,230,255)):
        font = cv2.FONT_HERSHEY_SIMPLEX
#        cv2.putText(self.image, text, (int(400), int(250)), font, 0.8, colour, 2)
        cv2.putText(self.image, text, (int(400), int(175)), font, 0.8, colour, 2)
        