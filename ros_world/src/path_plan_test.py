#!/usr/bin/env python3

from obstacle_detection import Maze
from random import randint
from math import sqrt

class RRT():
    def __init__(self):
        self.maze = Maze()
        self.width = 1919
        self.height = 1079
        self.start_point = (1248, 312)
        self.end_point = (1068, 394)
        self.path = [self.end_point]
        self.points = [self.start_point]
        self.parents = {}


    def dist(self, p1, p2):
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    
    def plan_path(self):
        done =False
        while not done:
            pnt = (randint(0, self.width), randint(0, self.height))
            print(pnt)

            if not self.maze.isPointInObstacles(pnt[0], pnt[1]):
                parent = None
                min_dist = 1000000
                for p in self.points:
                    d = self.dist(pnt, p)
                    if d <= min_dist:
                        min_dist = d
                        parent = p

                if (self.maze.isValidPoint(parent[0], parent[1], pnt[0], pnt[1])) and (min_dist <= 40):
                    self.parents[pnt] = parent
                    self.points.append(pnt)

                    if self.maze.isValidPoint(pnt[0], pnt[1], self.end_point[0], self.end_point[1]):
                        self.parents[self.end_point] = pnt
                        done = True       


        print(self.parents)

        pnt = self.parents[self.end_point]
        while pnt != self.start_point:
            self.path.append(pnt)
            pnt = self.parents[pnt]

        self.path.append(self.start_point)
        self.path = self.path[::-1]

        print("Yessss")

        self.maze.visualize_path(self.path)
                

           
planner = RRT()
planner.plan_path()