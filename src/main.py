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
    self.image_pub = rospy.Publisher("depth",Image,queue_size=5)

    self.bridge = CvBridge()

    self.__sub_lcam_name = rospy.get_param('~sub_leftcamera_name', "/stereo/left")
    self.__sub_rcam_name = rospy.get_param('~sub_rightcamera_name', "/stereo/right")
    
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
    print("creating stereo image",Dl)
    
    ## Undistort with fisheye image: https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
    undistorted_left = cv2.fisheye.undistortImage(cv_image_left,Kl,Dl[:4])
    print("showing depth image")
    cv2.imshow("Image window", undistorted_left)
    cv2.waitKey(3)

    """ do not publish now
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(depth, "16SC1"))
    except CvBridgeError as e:
      print(e)
    """


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