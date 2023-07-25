# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: Perform camera calibration using a chessboard.
 
import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
import glob # Used to get retrieve files that have a specified pattern
import pickle
 
# Path to the image that you want to undistort
distorted_img_filename = 'test.png'
 
def main():
                                                                                                                     
  # Now take a distorted image and undistort it 
  distorted_image = cv2.imread(distorted_img_filename)
  gray = cv2.cvtColor(distorted_image, cv2.COLOR_BGR2GRAY)
 
  height, width = distorted_image.shape[:2]
     
  size = len(distorted_img_filename)
  new_filename = distorted_img_filename[:size - 4]
  new_filename = new_filename + '_undistorted.png'
  
  # Save the camera calibration results.
  #calib_result_pickle = {}
  #calib_result_pickle["mtx"] = mtx
  #calib_result_pickle["optimal_camera_matrix"] = optimal_camera_matrix
  #calib_result_pickle["dist"] = dist
  #calib_result_pickle["rvecs"] = rvecs
  #calib_result_pickle["tvecs"] = tvecs
  #pickle.dump(calib_result_pickle, open("camera_calib_pickle.p", "wb" )) 
  
  calib_result_pickle = pickle.load(open("camera_calib_pickle.p", "rb" ))
  mtx = calib_result_pickle["mtx"]
  optimal_camera_matrix = calib_result_pickle["optimal_camera_matrix"]
  dist = calib_result_pickle["dist"]
  undistorted_image = cv2.undistort(distorted_image, mtx, dist, None, optimal_camera_matrix)

  # Save the undistorted image
  cv2.imwrite(new_filename, undistorted_image)
 
  # Close all windows
  cv2.destroyAllWindows() 
     
main()
