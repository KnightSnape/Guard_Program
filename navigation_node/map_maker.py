import numpy as np
import cv2
import yaml

resolution = 0.05
width = 640
height = 640

map_data = np.zeros((height,width), dtype = np.uint8)

map_metdata = {
    'image': 'map.pgm',
    'resolution': resolution,
    'origin': [0.0, 0.0, 0.0],
    'negate': 0,
    'occupied_thresh': 0.65,
    'free_thresh': 0.196
}

cv2.imwrite('map.pgm',map_data)
with open('map.yaml', 'w') as f:
    yaml.dump(map_metdata, f)

#目前只有空白地图