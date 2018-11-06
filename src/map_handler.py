#!/usr/bin/env python

import rospy
import cv2
from PIL import Image
import numpy as np
from nav_msgs.msg import OccupancyGrid
import zlib

class Map():
    def __init__(self):
        np.set_printoptions(threshold = np.nan)
        #self.costmap_mat = np.zeros(1, 1, cv2.COLOR_BGR2GRAY)
        pass
    
    def callback(self, data):
        rospy.loginfo("Getting data")
        self.width = data.info.width
        self.height = data.info.height
        self.resolution = data.info.resolution
        self.length = len(data.data)
        #self.min_line = []

        self.list = np.zeros((self.length))
        for i in range(1,self.length):
            self.list[i] = data.data[i]
        
        #creat an mat to load costmap
        self.costmap_mat = np.zeros((self.width, self.height), dtype = np.uint8)
        for i in range(1,self.height):
            for j in range(1,self.width):
               self.costmap_mat[j,i] = (255-int(float(data.data[(i-1)*self.width+j])/100*255))
        
        img = Image.fromarray(self.costmap_mat)
        img.show()
        
        string = np.array2string(self.list, separator=',')
        b = ". "
        for char in b:
            string = string.replace(char,"")
        compressed_data = zlib.compress(string)
        print(string)
        print(len(string))
        print(len(compressed_data))
        
        uncompressed_data = zlib.decompress(compressed_data)

        b = "[]"
        for char in b:
            uncompressed_data = uncompressed_data.replace(char,"")
        
        costmap_uncomp = np.fromstring(uncompressed_data, dtype=int, sep=' ')
        
        if costmap_uncomp == self.costmap_mat:
            print("True")
        else:
            print("False")
        
        rospy.loginfo("Done")
        
    def getMap(self):
        img =Image.fromarray(self.costmap_mat)
        img.show()
                
if __name__ == "__main__":
    rospy.init_node('map_com')
    m = Map()
    rospy.Subscriber('/map', OccupancyGrid, m.callback)
    rospy.sleep(10)