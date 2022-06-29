import sys
from types import coroutine
import numpy as np
import pyzed.sl as sl
import cv2
import math as m
import yaml
from yaml import load

RESOLUTION_V = 720
RESOLUTION_H = 1280

STARTING_PIXEL_X  = -RESOLUTION_H/2
STARTING_PIXEL_Y = RESOLUTION_V/2

LIDAR_X_DIFF = 1.2
LIDAR_Y_DIFF = 4
LIDAR_Z_DIFF = 0.5

FOV_V = 70
FOV_H = 110

V_PER_PIXEL = FOV_V/RESOLUTION_V # 0.097 deg 
H_PER_PIXEL = FOV_H/RESOLUTION_H # 0.085 deg

def yaml_reading(filepath):
  with open(filepath) as file:
    documents = yaml.full_load(file)

    for item, doc in documents.items():
      if item == "ranges":
        return doc
      else:
        continue

def cartesian_coordinates(pixel_dist, starting_pixel_pos_x, starting_pixel_pos_y):
  p = pixel_dist
  #get distance from camra to pixel
  
  phi = float(abs(starting_pixel_pos_x * H_PER_PIXEL))
  theta = float(abs(90-(starting_pixel_pos_y * V_PER_PIXEL)))
  #print("Couple of pixel : ({0}, {1}) has for angles theta = {2}, phi = {3} and the distance is : {4}".format(starting_pixel_pos_x, starting_pixel_pos_y, theta, phi, p))
  #print(p)

  x = p * m.sin(m.radians(theta)) * m.cos(m.radians(phi))
  y = p * m.sin(m.radians(theta)) * m.sin(m.radians(phi))
  z = p * m.cos(m.radians(theta))
  coordinates=[x,y,z]
  #print("Its coordinates are : {}".format(coordinates))
  #print(coordinates)
  return coordinates

def resize_lidar(lidar_arr):
  print("RESIZING LIDAR LIST")
  resized = []
  for x,elem in enumerate(lidar_arr):
    if 125 <= x <= 235:
      if elem == "inf":
        elem = 12
      resized.append(elem)
  return resized

def lidar_cartesian_coordinates(lidar_elem_dist, angle_incr):
  p = float(lidar_elem_dist)
  theta = 0
  phi = 125 + angle_incr 

  x = p * m.sin(m.radians(theta)) * m.cos(m.radians(phi))
  y = p * m.sin(m.radians(theta)) * m.sin(m.radians(phi))
  z = p * m.cos(m.radians(theta))

  coordinates=[x,y,z]
  return coordinates

def distances(cartesian_coordinates_arr):
  print("DISTANCES")
  distance_arr = []
  for elem in cartesian_coordinates_arr:
    p = m.sqrt((elem[0]**2) + (elem[1]**2) + (elem[2]**2))
    distance_arr.append(p)
  return distance_arr

def translate_plane(coordinates_arr,xdiff,ydiff,zdiff):
  print("TRANSLATING COORDINATES OF DEPTH MAP TO LIDAR LIST PLANE")
  for coordinates in coordinates_arr:
    if(xdiff != 0):
      coordinates[0] = coordinates[0] + xdiff
    if(ydiff != 0):
      coordinates[1] = coordinates[1] + ydiff
    if(zdiff != 0):
      coordinates[2] = coordinates[2] + zdiff
    #print("Translated coordinates : {0}".format(coordinates))

  return coordinates_arr

#------------------------------------MAIN PROCESSING FUNCTIONS------------------------------------#

def lidar_zed_coordinates_eval(cartesian_zed_arr):
  #print(cartesian_zed_arr.shape)
  print("STARTING LIDAR+ZED PROCESSING")
  #np.reshape(cartesian_zed_arr,(RESOLUTION_V,RESOLUTION_H))
  cartesian_zed_arr = cartesian_zed_arr.reshape((1280, 720, 3))
  list = []
  Y_list = []
  index_list = [] 
  nb_cols = cartesian_zed_arr.shape[0]
  nb_elem_cols = cartesian_zed_arr.shape[1]
  cpt = 0
  for i in range(nb_cols):
    if(i % 11 == 0):
      cpt = cpt + 1
      #print("COL NUMBER: {0}".format(i))
      for j in range (nb_elem_cols-1):
        #print(abs(cartesian_zed_arr[i, :][j]))
        list.append(abs(cartesian_zed_arr[i, :][j]))
        Y_list.append(list[j][2])
        #print(Y_list)
      if Y_list:
        index_list.append(np.argmin(Y_list))
      list = []
      Y_list = []
  #RESIZING LIST TO MATCH LIDAR'S LIST
  for k in range(6):
    index_list.pop(len(index_list)-1-k)
  #print("LENGTH : {0}".format(len(index_list)))
  #print(max(set(index_list), key=index_list.count))

  return(max(set(index_list), key=index_list.count))

def lidar_zed_distance_eval(cartesian_zed_arr, cartesian_lidar_arr):
  distances_zed = distances(cartesian_zed_arr)
  distances_zed = np.array(distances_zed)
  distances_lidar = distances(cartesian_lidar_arr)
  distances_lidar = np.array(distances_lidar)
  distances_zed = distances_zed.reshape((1280,720))

  min_dist = []
  nb_elem_cols = distances_zed.shape[1]
  i = 0
  
  while i < len(distances_lidar)-1: 
      for j in range (nb_elem_cols):
        diff = abs(distances_zed[i*11][j] - distances_lidar[i])
        min_dist.append(diff)
      i = i + 1


  min_dist = np.array(min_dist)
  min_dist = min_dist.reshape((110,720))
  min_index = np.argmin(min_dist,1)
  #print(min_index)

  counts = np.bincount(min_index)
  line = np.argmax(counts)

  return line

def depth_to_cartesian():
    if len(sys.argv) != 3:
        print("Please specify path to .svo file and/or .txt file.")
        exit()

    filepath = sys.argv[1]
    print("Reading SVO file: {0}".format(filepath))

    yamlpath = sys.argv[2]
    print("Reading LIDAR file: {0}".format(yamlpath))

    input_type = sl.InputType()
    input_type.set_from_svo_file(filepath)
    init = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)

    zed = sl.Camera()

    init.sdk_verbose = True # Enable verbose logging
    init.depth_mode = sl.DEPTH_MODE.QUALITY # Set the depth mode to performance (fastest)
    init.coordinate_units = sl.UNIT.METER  # Use millimeter units

    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Error {}, exit program".format(err)) # Display the error
        exit()

    # Capture 50 images and depth, then stop
    i = 0
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()
    cartesian_arr = []
    lidar_cartesian_arr = []
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL
        # Grab an image
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns sl.ERROR_CODE.SUCCESS
        zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH) # Retrieve depth matrix. Depth is aligned on the left RGB image
        #zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA) # Retrieve colored point cloud
        #image_ocv = image.get_data()
        depth_image_ocv = depth.get_data()
        print(depth_image_ocv.shape)
        depth_image_ocv = depth_image_ocv.reshape((1280, 720))
        print("CALCULATING CARTESIAN COORDINATES FOR DEPTH MAP")
        for i in range (0, RESOLUTION_H):
            for j in range (0, RESOLUTION_V):
      
              coordinates = cartesian_coordinates(depth_image_ocv[i][j],(STARTING_PIXEL_X)+i,(STARTING_PIXEL_Y)-j)
              cartesian_arr.append(coordinates)
    
        translated_cartesian_arr = translate_plane(cartesian_arr,LIDAR_X_DIFF,LIDAR_Y_DIFF,LIDAR_Z_DIFF)
        translated_cartesian_arr = np.array(translated_cartesian_arr)
        #print(len(translated_cartesian_arr))
        np.reshape(translated_cartesian_arr,(RESOLUTION_V,RESOLUTION_H,3))

        lidar_array = yaml_reading(yamlpath)
        lidar_array = resize_lidar(lidar_array)
        print("CALCULATING CARTESIAN COORDINATES FOR LIDAR LIST")
        for k in range(0,len(lidar_array)):
          lidar_coordinates = lidar_cartesian_coordinates(lidar_array[k],k)
          lidar_cartesian_arr.append(lidar_coordinates)
        print("LENGHT OF LIDAR ARRAY SHOULD BE 111, THE OUTPUT IS: {0} FOR LENGHT".format(len(lidar_cartesian_arr)))
    return [translated_cartesian_arr, lidar_cartesian_arr]

#------------------------------------MAIN PROCESSING FUNCTIONS------------------------------------#

#----------------------------------------------MAIN----------------------------------------------#

def main():
  print("MAIN")
  cartesian_zed, cartesian_lidar = depth_to_cartesian()

  coordinate_line = lidar_zed_coordinates_eval(cartesian_zed)
  distance_line = lidar_zed_distance_eval(cartesian_zed, cartesian_lidar)

  print("[RESULTS][COORDINATES] LINE {0} IS THE MOST PROBABLE LINE FOR LIDAR SCANNING".format(coordinate_line))
  print("[RESULTS][DISTANCES] LINE {0} IS THE MOST PROBABLE LINE FOR LIDAR SCANNING".format(distance_line))

if __name__ == "__main__":
    main()