import cv2
import numpy as np

# Select the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)

# Define marker ID and size
marker_id = 2  # Change this to generate different markers (0 to 99)
marker_size = 700  # Size in pixels

# Generate the marker image
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# Save the marker image
cv2.imwrite(f"aruco_7X7_1000_id{marker_id}.png", marker_image)
# Display the marker image
cv2.imshow(f"ArUco Marker ID {marker_id}", marker_image)