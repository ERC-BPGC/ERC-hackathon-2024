#!/usr/bin/env python3

from obstacle_detection import Maze
__build_class__


########################################################################################################

# Yo! This demo code demonstrates basic usage of functions provided to you in obstacle_detection.py
#                                           Have Fun!!!

########################################################################################################



# Creating maze object
maze = Maze()

# Testing isPointInObstacles function
dummy_pnt = (5, 10)
print(maze.isPointInObstacles(dummy_pnt[0], dummy_pnt[1]))

# Testing isValidPoint function
pnt1 = (5, 10)
pnt2 = (50, 90)
print(maze.isValidPoint(pnt1[0], pnt1[1], pnt2[0], pnt2[1]))

# Testing visualize_path function
# Dummy path from ERC Room to Smoker's Lane
dummy_path = [(70, 367), (71, 381), (71, 382), (80, 398), (88, 407), (111, 437), (138, 449), (135, 454), 
              (150, 461), (149, 461), (143, 461), (135, 467), (131, 474), (121, 483), (93, 491), (82, 494), 
              (73, 498), (61, 513), (54, 515), (46, 548), (48, 566), (144, 650)]

maze.visualize_path(dummy_path)

# Testing point transform function
print(maze.point_transform((144, 650)))