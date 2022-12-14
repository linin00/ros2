# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import time
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')

    qos_profile = QoSProfile(
    	reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    	history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    	depth=1
    )
     
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/camera/color/image_raw', 
      self.listener_callback, 
      qos_profile=qos_profile)
    #self.subscription # prevent unused variable warning
    

    self.pub_novel = self.create_publisher(Twist,"/cmd_vel", 10) 
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.last_t = 0
    self.i = 0
  
  def listener_callback(self, data):
    """
    Callback function.
    """
    #print("hello")
    # Display the message on the console
    self.get_logger().info('Receiving video frame')

    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
   
    # Display image
    # cv2.imshow("camera", current_frame)
    t = time.time()
    t = int(round(t * 1000))
    print(f"fps = {1000. / (t - self.last_t)}")
    self.last_t = t
    cv2.imwrite(f"{t}.png", current_frame)
    cv2.waitKey(1)
    # ??????????????????
    gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    # ??????????????????
    retval, dst = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
    # ????????????????????????
    dst = cv2.dilate(dst, None, iterations=2)
    # # ????????????????????????
    # dst = cv2.erode(dst, None, iterations=6)
    # cv2.imshow("dst", dst)
    cv2.imwrite(f"{t}_dst.png", dst)
    # ?????????400???????????????
    color = dst[400]
    # ??????????????????????????????
    white_count = np.sum(color == 0)
    # ??????????????????????????????
    white_count_judge = np.sum(color == 255)  # ?????????????????????????????????????????????????????????
    if white_count_judge == 640:
        print("??????????????????0")
        pass
    else:
        white_index = np.where(color == 0)
        # ??????white_count=0?????????
        if white_count == 0:
            white_count = 1

        # ????????????????????????????????????
        center = (white_index[0][white_count - 1] + white_index[0][0]) / 2
        direction = center - 320
        print(direction)
        # ?????????center??????????????????????????????    
    
        msg = Twist()
        msg.linear.x = 0.4
        msg.angular.z = direction / 320. * (-1)
        self.pub_novel.publish(msg)  #???????????????????????????
        print("vel_command = ", msg)
    
 
def main(args=None):
 
  # Initialize the rclpy library
  rclpy.init(args=args)
 
  # Create the node
  image_subscriber = ImageSubscriber()
 
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
 
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
 
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()
