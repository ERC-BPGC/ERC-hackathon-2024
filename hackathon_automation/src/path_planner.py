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
# ERC Room: 109.95, 296.15
# Cone 1: 155.93, 205.52
# Cone 2: 299.81, 142.61
# Cone 3: 506.77, 536.82
# Cone 4: 239.65, 643.55
# Cone 5: 193.79, 329.94

# These are the coordinates you have to go to in Gazebo
# Start Location (ERC Room): 12.994061, 14.233000
# Cone 1 (near Sandbox): 19.964537, 10.696421
# Cone 2 (NAB): 24.802801, -0.369133
# Cone 3 (VGH): -5.516354, -16.286877
# Cone 4 (SAC): -13.724400, 4.257628
# Cone 5 (BDome): 10.395211, 7.784442
# Cones colors will be randomized

# There is a function to convert the obstacle_detection-> Maze-> points point_transform