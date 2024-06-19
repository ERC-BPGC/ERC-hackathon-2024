#!/usr/bin/env python3

import cv2



########################################################################################

# Hii! This code helps you get coordinates of any points on the BITS Map provided.
# Just Run the code, and click on any point to retreive it's coordinates!

########################################################################################



# Mouse callback function to display the coordinates of the pixel
def show_coordinates(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Mouse coordinates: ({x}, {y})")

# Load the image
image_path = 'roads.png'  # Replace with your image path
image = cv2.imread(image_path)

# Create a window and set the mouse callback function
cv2.namedWindow('Image')
cv2.setMouseCallback('Image', show_coordinates)

while True:
    cv2.imshow('Image', image)
    if cv2.waitKey(1) & 0xFF == 27:  # Press 'ESC' to exit
        break


cv2.destroyAllWindows()