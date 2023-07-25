#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import pickle
from imutils.video import WebcamVideoStream
import numpy as np
from sambot_msgs.msg import NavState
import math

inch_to_meter = 0.0254
canvas_size = 12.0 * 9.0 * inch_to_meter
canvas_size_in_pixel = 800.0
half_canvas_size = canvas_size / 2.0

def coord_to_pixel(x, y):
    x_new, y_new = (x + half_canvas_size) / canvas_size * canvas_size_in_pixel, (y + half_canvas_size) / canvas_size * canvas_size_in_pixel
    return (int(x_new), int(y_new))

# ------- CALLBACKS -------
def sambot1_nav_callback(data):
    global position
    if (math.isnan(data.position[0]) or math.isnan(data.position[1])):
        return
    position[0] = coord_to_pixel(-data.position[0], data.position[1])

def sambot2_nav_callback(data):
    global position
    if (math.isnan(data.position[0]) or math.isnan(data.position[1])):
        return
    position[1] = coord_to_pixel(-data.position[0], data.position[1])

def sambot3_nav_callback(data):
    global position
    if (math.isnan(data.position[0]) or math.isnan(data.position[1])):
        return
    position[2] = coord_to_pixel(-data.position[0], data.position[1])

def sambot4_nav_callback(data):
    global position
    if (math.isnan(data.position[0]) or math.isnan(data.position[1])):
        return
    position[3] = coord_to_pixel(-data.position[0], data.position[1])

def center_goal_callback(data):
    global center_goal
    center_goal = coord_to_pixel(-data.goal[0], data.goal[1]) if (data.idle != True) else None


CAMERA_SRC_1 = 'rtsp://admin:sambotcam1@192.168.0.101:554/'
CAMERA_SRC_2 = 'rtsp://admin:sambotcam1@192.168.0.103:554/'


DIR = '/home/thinh/workspace/ros_ws/src/sambot_rl/scripts/'
# FIELD_IMAGE_PATH = DIR + 'field.png'
FIELD_IMAGE_PATH = DIR + 'field_2srcs.png'
BACKUP_IMAGE_PATH = DIR + 'test_frame.png'

center_goal = None
position = [None, None, None, None]

env_field_img = None
agent_field_img = None

animated_radius = 0
max_radius = 12
min_radius = 1
increment_constant = 0.75
sign = 1

def get_animated_radius():
    global animated_radius, max_radius, min_radius, increment_constant, sign
    animated_radius += sign * increment_constant
    if animated_radius > max_radius:
        animated_radius = max_radius
        sign = -1
    elif animated_radius < min_radius:
        animated_radius = min_radius
        sign = 1
    return int(animated_radius)

def apply_projection_top(background_img, content_img, plot_img=None):
    (imgH, imgW), (srcH, srcW) = background_img.shape[:2], content_img.shape[:2]
    srcMat = np.array([[0, 0], [srcW, 0], [srcW, srcH], [0, srcH]])
    # dstMat = np.array([[429,317],[858,313],[1048,652],[200,647]])
    dstMat = np.array([[267,18],[936,14],[989,704],[234,710]])

    (H, _) = cv2.findHomography(srcMat, dstMat)
    warped = cv2.warpPerspective(content_img, H, (imgW, imgH))

    mask = np.zeros((imgH, imgW), dtype="uint8")
    cv2.fillConvexPoly(mask, dstMat.astype("int32"), (255, 255, 255),
        cv2.LINE_AA)

    rect = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.dilate(mask, rect, iterations=2)

    maskScaled = mask.copy() / 255.0
    maskScaled = np.dstack([maskScaled] * 3)

    warpedMultiplied = cv2.multiply(warped.astype("float"), maskScaled)
    #imageMultiplied = cv2.multiply(background_img.astype(float), 1.0 - maskScaled)
    # output = cv2.add(warpedMultiplied, imageMultiplied)
    output = cv2.addWeighted(warpedMultiplied, 0.25, background_img.astype(float), 0.75,0)
    output = output.astype("uint8")

    return output

def apply_projection_front(background_img, content_img, top_img=None, plot_img=None):
    (imgH, imgW), (srcH, srcW) = background_img.shape[:2], content_img.shape[:2]
    srcMat = np.array([[0, 0], [srcW, 0], [srcW, srcH], [0, srcH]])
    dstMat = np.array([[384,309],[813,300],[999,632],[146,643]])
    # dstMat = np.array([[267,18],[936,14],[989,704],[234,710]])

    (H, _) = cv2.findHomography(srcMat, dstMat)
    warped = cv2.warpPerspective(content_img, H, (imgW, imgH))

    mask = np.zeros((imgH, imgW), dtype="uint8")
    cv2.fillConvexPoly(mask, dstMat.astype("int32"), (255, 255, 255),
        cv2.LINE_AA)

    rect = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.dilate(mask, rect, iterations=2)

    maskScaled = mask.copy() / 255.0
    maskScaled = np.dstack([maskScaled] * 3)

    warpedMultiplied = cv2.multiply(warped.astype("float"), maskScaled)
    #imageMultiplied = cv2.multiply(background_img.astype(float), 1.0 - maskScaled)
    # output = cv2.add(warpedMultiplied, imageMultiplied)
    output = cv2.addWeighted(warpedMultiplied, 0.25, background_img.astype(float), 0.75,0)
    output = output.astype("uint8")

    if plot_img is not None:
        plotH, plotW = plot_img.shape[0], plot_img.shape[1]
        output[:plotH, :plotW, :] = plot_img

    if top_img is not None:
        topH, topW = top_img.shape[0], top_img.shape[1]
        offset = (topW-topH)//2
        top_img = top_img[:, offset-50:topW-offset]
        top_img = cv2.resize(top_img, (350,350), interpolation = cv2.INTER_AREA)
        topH, topW = top_img.shape[0], top_img.shape[1]
        output[:topH, imgW-topW:, :] = top_img

    return output

def draw_target_location(content_img):
    global center_goal, position
    if center_goal:
        (srcH, srcW) = content_img.shape[:2]
        overlay = content_img.copy()
        radius = get_animated_radius()
        overlay = cv2.circle(overlay, center_goal, radius, (0, 255, 0), -1)
        for i in range(4):
            if position[i] is not None:
                overlay = cv2.line(overlay, center_goal, position[i], (0, 255, 0), 1)
        alpha = 0.8  # Transparency factor.
        image_with_target = cv2.addWeighted(overlay, alpha, content_img, 1 - alpha, 0)
        return image_with_target
    else:
        return content_img

def env_field_cb(data): 
  global env_field_img
  br = CvBridge()
  env_field_img = br.imgmsg_to_cv2(data)

def agent_field_cb(data): 
  global agent_field_img
  br = CvBridge()
  agent_field_img = br.imgmsg_to_cv2(data)

def publish_message():
 
    # Node is publishing to the video_frames topic using 
    # the message type Image
    # pub_raw = rospy.Publisher('camera/raw', Image, queue_size=10)
    pub_proj = rospy.Publisher('camera/projection', Image, queue_size=10)

    rospy.Subscriber('/env_field_img', Image, env_field_cb)
    rospy.Subscriber('/agent_field_img', Image, agent_field_cb)

    rospy.Subscriber("/center_goal", NavState, center_goal_callback)
    rospy.Subscriber("/sambot1/navigation", NavState, sambot1_nav_callback)
    rospy.Subscriber("/sambot2/navigation", NavState, sambot2_nav_callback)
    rospy.Subscriber("/sambot3/navigation", NavState, sambot3_nav_callback)
    rospy.Subscriber("/sambot4/navigation", NavState, sambot4_nav_callback)

    calib_result_pickle = pickle.load(open("/home/thinh/workspace/ros_ws/src/sambot_rl/scripts/camera_calib_pickle.p", "rb" ))
    mtx = calib_result_pickle["mtx"]
    optimal_camera_matrix = calib_result_pickle["optimal_camera_matrix"]
    dist = calib_result_pickle["dist"]
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('camera_pub_py', anonymous=True)
        
    # Go through the loop 10 times per second
    rate = rospy.Rate(30) # 10hz
        
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    #cap = cv2.VideoCapture(0)
    cap_top = WebcamVideoStream(CAMERA_SRC_1).start()
    cap_front = WebcamVideoStream(CAMERA_SRC_2).start()
    backup = cv2.imread(BACKUP_IMAGE_PATH)
    source_backup = cv2.imread(FIELD_IMAGE_PATH)
    #frame = cap.read()	

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # While ROS is still running.
    while not rospy.is_shutdown():
        
        #ret, frame = cap.read()
        frame_front = cap_front.read()
        frame_top = cap_top.read()
        #frame = backup

        projection_canvas = draw_target_location(source_backup if (env_field_img is None) else env_field_img)

        if frame_top is not None:
            undistorted_frame_top = cv2.undistort(frame_top, mtx, dist, None, optimal_camera_matrix)
            flipped_frame_top = cv2.flip(undistorted_frame_top, -1)            
            projected_frame_top = apply_projection_top(flipped_frame_top, projection_canvas, plot_img=agent_field_img)

        if frame_front is not None:
            undistorted_frame_front = cv2.undistort(frame_front, mtx, dist, None, optimal_camera_matrix)
            projected_frame_front = apply_projection_front(undistorted_frame_front, projection_canvas, plot_img=agent_field_img, top_img=projected_frame_top)

            # Publish the image.
            pub_proj.publish(br.cv2_to_imgmsg(projected_frame_front, 'bgr8'))
                
        # Sleep just enough to maintain the desired rate
        rate.sleep()
    
    cap_front.stop()
    cap_top.stop()
            
if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
