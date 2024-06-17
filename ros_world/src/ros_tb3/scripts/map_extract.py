import cv2
import numpy as np
import svgwrite
from PIL import Image

# Load the image
image_path = 'src/ros_tb3/scripts/bits goa map.png'
image = cv2.imread(image_path)

if image is None:
    raise ValueError(f"Image not found or unable to load: {image_path}")


# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Threshold the image to extract the white components (roads)
_, binary = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)

# Invert the binary image
# binary = cv2.bitwise_not(binary)

# Extract the white components
roads = cv2.bitwise_and(image, image, mask=binary)

# Save the image with extracted roads as JPG
roads_path_jpg = 'roads.jpg'
cv2.imwrite(roads_path_jpg, roads)

# Optimized function to convert image to SVG
def optimized_image_to_svg(image_path, svg_path):
    img = Image.open(image_path).convert("L")
    img = img.point(lambda x: 0 if x < 128 else 255, '1')  # Binarize the image
    width, height = img.size
    dwg = svgwrite.Drawing(svg_path, profile='tiny', size=(width, height))
    for y in range(height):
        for x in range(width):
            if img.getpixel((x, y)) == 0:
                dwg.add(dwg.rect(insert=(x, y), size=(1, 1), fill='black', stroke='none'))
    dwg.save()

# Save the image with extracted roads as SVG
roads_path_svg = 'roads.svg'
optimized_image_to_svg(roads_path_jpg, roads_path_svg)
