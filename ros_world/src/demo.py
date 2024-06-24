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
dummy_path = [(1248, 312), (1254, 311), (1258, 300), (1244, 289), (1243, 284), (1243, 276), (1239, 280), (1226, 276), (1214, 272),
              (1199, 260), (1184, 261), (1178, 261), (1162, 264), (1153, 266), (1152, 263), (1136, 265), (1122, 264), (1122, 270),
              (1109, 274), (1097, 286), (1081, 294), (1078, 309), (1067, 311), (1059, 317), (1060, 324), (1054, 338), (1053, 338),
              (1044, 348), (1038, 348), (1040, 365), (1034, 365), (1035, 360), (1033, 358), (1031, 356), (1027, 352), (1017, 339),
              (1001, 330), (995, 327), (998, 321), (995, 321), (989, 307), (987, 302), (987, 282), (979, 264), (980, 260), (966, 249),
              (955, 243), (945, 235), (935, 239), (925, 231), (906, 227), (892, 234), (886, 235), (877, 247), (878, 239), (682, 477)]

maze.visualize_path(dummy_path)