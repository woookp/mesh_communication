#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# 初始化CvBridge
bridge = CvBridge()

def callback(data):
    try:
        # 将ROS的压缩图像消息转换为OpenCV格式
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 显示图像
        cv2.imshow('Compressed Image', image_np)
        cv2.waitKey(2)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    rospy.init_node('compressed_image_subscriber', anonymous=True)
    
    # 订阅sensor_msgs/CompressedImage消息
    rospy.Subscriber("/camera/image/compressed", CompressedImage, callback)
    
    # 防止Python退出直到节点被关闭
    rospy.spin()

if __name__ == '__main__':
    main()
