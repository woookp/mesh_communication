#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# 初始化cv_bridge
bridge = CvBridge()

def callback(data):
        # 转换压缩图像数据为OpenCV格式
        print(len(data.data)/8)
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image_np is not None:
            print("Decoded image shape:", image_np.shape)
            cv2.imshow('Compressed Video', image_np)
            cv2.waitKey(1)
        else:
            print("Failed to decode image")       
        # 显示图像
        cv2.imshow('Compressed Video', image_np)
        cv2.waitKey(1)


def main():
    # 初始化ROS节点
    rospy.init_node('compressed_image_subscriber', anonymous=True)
    
    # 订阅compressed image data的话题
    rospy.Subscriber("/received_compressed_image", CompressedImage, callback)
    
    # 防止python退出直到节点停止
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

