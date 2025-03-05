import numpy as np
import utm
import matplotlib.pyplot as plt
import os
from ament_index_python.packages import get_package_share_directory

'''
Created by: Daniel Webb
Date: 11/24/2024

This class takes a map (asc file) and then converts from UTM to Lat/Lon coordinates.

It is initialized with either a map (2D array) with its lower left corner UTM coordinates and resolution
OR
an ASCII file path from which it extracts this information.

You must also provide the UTM zone and hemisphere letter for either of the above options.
Resource to find this information: https://www.latlong.net/lat-long-utm.html
    Enter the lattitude and longitude, convert and then look at the UTM zone
    For example, "12N" would be zone=12, zone_letter='N'
'''

file_path=os.path.join(get_package_share_directory('path_planning'), 'data', 'gravel_pits.asc')

class Mapper:
    def __init__(self, map=None, yll=None, xll=None, res=None, zone=None, zone_letter=None,
                 file_path=None, file_type='ascii'):
        
        if map is not None:
            self.map = map
            self.yll = yll
            self.xll = xll
            self.res = res
            self.h = map.shape[0]

        elif file_type == 'ascii':
            self.map, (ncols, nrows, xllcorner, yllcorner, cellsize, nodata_value) = self.read_asc_file(file_path)
            self.yll = yllcorner
            self.xll = xllcorner
            self.res = cellsize
            self.h = nrows

        self.zone = zone
        self.zone_letter = zone_letter

        # Gradient Map
        dx, dy = np.gradient(self.map)
        self.grad_map = np.sqrt(dx**2 + dy**2)

    def chop_map(self, x1, x2, y1, y2):
        '''
        Crops the map to the specified coordinates #TODO: add some capability to figure out how to chop the map at the competition or add full map
        '''
        self.map = self.map[y1:y2, x1:x2]
        self.grad_map = self.grad_map[y1:y2, x1:x2]

        # Update the lower left corner coordinates and height of the map
        self.h = self.map.shape[0]
        self.yll = self.yll + y1*self.res
        self.xll = self.xll + x1*self.res

        return self.map.shape

    def xy_to_latlon(self, x, y):
        '''
        returns the latitude and longitude of a given x, y coordinate in the utm map
        where x is measured from left to right and y is measured from bottom to top
            For example self.map[0,0] is the top left corner of the map
            (resembles the layout of a 2D numpy array)
        '''
        x_utm = self.xll + x*self.res
        y_utm = self.yll + self.h - y*self.res
        
        return utm.to_latlon(x_utm, y_utm, self.zone, self.zone_letter)
    
    def latlon_to_xy(self, lat, lon):
        '''
        returns the nearest x, y coordinate of a given latitude and longitude
        where x is measured from left to right and y is measured from bottom to top
            For example self.map[0,0] is the top left corner of the map
            (resembles the layout of a 2D numpy array)

            NOTE: Returns None for both x and y if the lat/lon is outside the map
        '''
        x_utm, y_utm, zone, zone_letter = utm.from_latlon(lat, lon)
        x = int((x_utm - self.xll) / self.res)
        y = self.h - int((y_utm - self.yll) / self.res)
        
        if x < 0 or x >= self.map.shape[1] or y < 0 or y >= self.map.shape[0]:
            return None, None
        else:
            return x, y
    
    def display_map(self, map_type='Elevation'):
        '''
        Displays the map using matplotlib
        '''
        if map_type == 'Slope':
            disp_map = self.grad_map
        else:
            disp_map = self.map

        plt.imshow(disp_map, cmap='terrain')
        plt.title(map_type + ' Map')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.colorbar(label=map_type)
        plt.show()

    def read_asc_file(self, file_path):
        with open(file_path, 'r') as f:
            header = [next(f) for _ in range(6)]
        
        ncols = int(header[0].split()[1])
        nrows = int(header[1].split()[1])
        xllcorner = float(header[2].split()[1])
        yllcorner = float(header[3].split()[1])
        cellsize = float(header[4].split()[1])
        nodata_value = float(header[5].split()[1])
        
        data = np.loadtxt(file_path, skiprows=6)
        return data, (ncols, nrows, xllcorner, yllcorner, cellsize, nodata_value)

def main():
    # Testing the Mapper class
    mapper = Mapper(file_path=file_path, zone=12, zone_letter='N')

    # Cheking latlon/xy conversion accuracy
    x, y = 200, 300
    print('original x and y:')
    print(x, y)
    lat, lon = mapper.xy_to_latlon(x, y)
    x, y = mapper.latlon_to_xy(lat, lon)
    print('x and y after conversion to and from lat lon:')
    print(x, y)

    pass

if __name__ == "__main__":
    main()