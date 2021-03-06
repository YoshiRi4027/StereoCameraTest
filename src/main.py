#!/usr/bin/env python
## Look at here https://answers.ros.org/question/305589/depth-image-from-rectified-stereo-images-or-disparity-image/
import sys
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

# assuming monocular image
class stereo_rectifier:

  def __init__(self):
    self.bridge = CvBridge()

    self.__sub_lcam_name = rospy.get_param('~sub_leftcamera_name', "/stereo/left")
    self.__sub_rcam_name = rospy.get_param('~sub_rightcamera_name', "/stereo/right")
    
    # publisher
    self.image_publ = rospy.Publisher(self.__sub_lcam_name+"/image_rect",Image,queue_size=5)
    self.image_pubr = rospy.Publisher(self.__sub_rcam_name+"/image_rect",Image,queue_size=5)
    self.caminfo_publ = rospy.Publisher(self.__sub_lcam_name+"/rect/camera_info",CameraInfo,queue_size=5)
    self.caminfo_pubr = rospy.Publisher(self.__sub_rcam_name+"/rect/camera_info",CameraInfo,queue_size=5)


    print("Create subscribers for each topic")
    self.left = message_filters.Subscriber(self.__sub_lcam_name+"/image_raw", Image)
    self.left_caminfo = message_filters.Subscriber(self.__sub_lcam_name+"/camera_info", CameraInfo)
    self.right = message_filters.Subscriber(self.__sub_rcam_name+"/image_raw", Image)
    self.right_caminfo = message_filters.Subscriber(self.__sub_rcam_name+"/camera_info", CameraInfo)
    print("Create sync filter. Use exact or approximate as appropriate.")
    self.ts = message_filters.ApproximateTimeSynchronizer([self.left, self.left_caminfo, self.right, self.right_caminfo], queue_size=5, slop=0.1)
    print("Registering callback")
    self.ts.registerCallback(self.callback)

  def callback(self,left,lcam_info,right,rcam_info):
    try:
        # assuming monocular image
        cv_image_left = self.bridge.imgmsg_to_cv2(left, "mono8")
        cv_image_right = self.bridge.imgmsg_to_cv2(right, "mono8")

    except CvBridgeError as e:
        print(e)

    # get camera info
    Kl = np.array(lcam_info.K).reshape([3,3])
    Kr = np.array(rcam_info.K).reshape([3,3])
    Dl = np.array(lcam_info.D)
    Dr = np.array(rcam_info.D)
    
    ## Undistort with fisheye image: https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
    undistorted_l = cv2.fisheye.undistortImage(cv_image_left,Kl,Dl[:4],Knew=Kl)
    undistorted_r = cv2.fisheye.undistortImage(cv_image_right,Kr,Dr[:4],Knew=Kr)
    #print("showing depth image")
    #cv2.imshow("Image window", undistorted_left)
    #cv2.waitKey(3)

    # Camera Info 再発行
    New_infol = lcam_info
    New_infor = rcam_info

    # Baselineの情報がないので仕方なく http://official-rtab-map-forum.67519.x6.nabble.com/Slam-using-Intel-RealSense-tracking-camera-T265-td6333.html から計算
    Tx = 0.064138
    fx = Kr[0,0]
    p14 = - Tx * fx
    New_infor.P[3] = p14
    
    
    # Publish
    try:
      self.image_publ.publish(self.bridge.cv2_to_imgmsg(undistorted_l, "mono8"))
      self.image_pubr.publish(self.bridge.cv2_to_imgmsg(undistorted_r, "mono8"))
      self.caminfo_publ.publish(New_infol)
      self.caminfo_pubr.publish(New_infor)
    except CvBridgeError as e:
      print(e)
    


def main(args):
  rospy.init_node('image_rect', anonymous=True)
  ic = stereo_rectifier()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)