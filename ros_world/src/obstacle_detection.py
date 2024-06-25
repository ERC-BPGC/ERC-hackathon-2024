#!/usr/bin/env python3

import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point
import cv2



########################################################################################

# Sup! This amazing piece of code (lol) provides you with some quintessential functions
#               for your path planning task. Be sure to use them wisely!

########################################################################################



# Some points for reference and testing
# ERC Room : (85, 367)
# Smokers : (139, 647)



# Class Maze
class Maze:
    def __init__(self):
        (thresh, self.roads) = cv2.threshold(cv2.imread("roads.png", cv2.IMREAD_GRAYSCALE), 127, 255, cv2.THRESH_BINARY)
        self.display_img = cv2.cvtColor(self.roads.copy(), cv2.COLOR_GRAY2RGB)


    # Helper function for isValidPoint function
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's Line Algorithm to generate points between two coordinates"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return points


    # Function to check if a point lies in an obstacle. If it does, function returns True
    def isPointInObstacles(self, xCoord, yCoord):
        is_point_in_obs = False
        if self.roads[yCoord, xCoord] == 0:
            is_point_in_obs = True

        return is_point_in_obs


    # Function to check if a valid link can form between two points i.e the line connecting the two points does not cross an obstacle
    def isValidPoint(self, parentNodeX, parentNodeY, childNodeX, childNodeY):
        valid_point = True
        line_points = self.bresenham_line(parentNodeX, parentNodeY, childNodeX, childNodeY)
        
        for x, y in line_points:
            if self.isPointInObstacles(x, y):
                valid_point = False

        return valid_point
    

    # Function to visualize your planned path. Accepts path as a nested list or a list of tuples of length 2 (x and y coords)
    def visualize_path(self, path):
        for i in range(len(path) - 1):
            cv2.line(self.display_img, path[i], path[i + 1], (255, 0, 0), 2) 

        cv2.imshow("Planned Path", self.display_img)
        cv2.waitKey(0)

    
    # Function to transform coordinates from opencv image to Gazebo simulation coordinate frame
    def point_transform(self, pnt):
        gazebo_x = (-1 * pnt[1] / 13.002) + 35.7567 + 0.01475
        gazebo_y = (-1 * pnt[0] / 13.002) + 22.6746 + 0.01475
        return (gazebo_x, gazebo_y)
        