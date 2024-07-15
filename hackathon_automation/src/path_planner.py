########################################################################################################

# Wassup! This is where your Path Planning code goes for the Automation Task of the ERC Hackathon '24
#
# Remember to use the transform points function in obstacle_detection.py to get points corresponding to
#         your path in the gazebo coordinate system. This is what controller.py should use!!
#
#                                       Happy Planning!!!

########################################################################################################

# import functions from obstacle_detection.py

# Cooridinates of the cones as per the map (image version) - look at get_landmarks.py & obstacle_detection.py
# ERC Room: 109, 296
# Cone 1: 159, 208
# Cone 2: 296, 142
# Cone 3: 502, 539
# Cone 4: 244, 640
# Cone 5: 190, 326

# You can look at the visualization function in demo.py to see how to visualize the path

# These are the coordinates you have to go to in Gazebo
# Start Location (ERC Room): 12.994061, 14.233000
# Cone 1 (near Sandbox): 19.765875, 10.429128
# Cone 2 (NAB): 24.820259, -0.089209
# Cone 3 (VGH): -5.729289, -15.973051
# Cone 4 (SAC): -13.520048, 3.922464
# Cone 5 (BDome): 10.654515,8.028189
# Cones colors will be randomized

# There is a function to convert the obstacle_detection-> Maze-> points point_transform
