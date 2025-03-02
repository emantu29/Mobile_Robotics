import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, PointCloud2
import sensor_msgs_py.point_cloud2 as pcl2
from std_msgs.msg import Float32

SECURITY_DISTANCE = 4.0  # minimum distance to avoid the obstacle
TARGET_COLOR_LOWER = np.array([0, 120, 70])  # lower HSV limit for red color
TARGET_COLOR_UPPER = np.array([10, 255, 255])  # higher HSV limit for red color
TURN_SPEED = 0.5  # turning power
FORWARD_SPEED = 0.2  # power to go straight
PERC_OF_WHITE_PIX = 0.028 # minimum percentage of RGB camera fov covered by the target necessary to stop the boat

class TargetFollowing(Node):
    def __init__(self):
        super().__init__('target_following')
        self.bridge = CvBridge()
        self.target_depth = None
        self.target_offset = None
        
        self.image_sub = self.create_subscription(CompressedImage, '/camera/color/image/compressed', self.image_callback, 10)
        self.pcl_sub = self.create_subscription(PointCloud2, '/camera/depth/points', self.pointcloud_callback, 10)
        self.motor_left_pub = self.create_publisher(Float32, '/motor_left', 10)
        self.motor_right_pub = self.create_publisher(Float32, '/motor_right', 10)
        
        self.get_logger().info("Target Following Node Started")
    
    #With the RGB camera we find the coordinates on the image of the red target
    def image_callback(self, msg): 
        np_arr = np.frombuffer(msg.data, np.uint8) #loading the data
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # trasform the color from bgr to hsv format
        mask = cv2.inRange(hsv, TARGET_COLOR_LOWER, TARGET_COLOR_UPPER) # every pixel with the color in the range becomes white, the others black
        moments = cv2.moments(mask) # find the moment of the image

                
        if moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00']) #find the x coordinate of the centroid of the image
            #cy = int(moments['m01'] / moments['m00']) #find the y coordinate of the centroid of the image, not used in the code
            self.target_offset = (cx - image.shape[1] // 2) / (image.shape[1] // 2) # find the position of the centroid of the target based on the camera field of view
            self.get_logger().info(f"Target detected at offset {self.target_offset:.2f}")
            self.num_white_pix = np.sum(mask == 255) #number of white pixel in the image
        else:
            self.target_offset = None
            self.num_white_pix = 0
    
    #with the depth camera check if there are any obstacles near the boat
    def pointcloud_callback(self, msg):
        points = np.array(list(pcl2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        if points.size == 0:
            self.get_logger().warn("No valid points received!")
            return
        
        depth_index = 0 

        # filter the points in front of the robot
        front_points = points[
            (np.abs(points[:, 1]) < 0.5) &  # filter lateral points, where index 1 is equal to y axis
            (points[:, depth_index] > 0.1) &  # do not consider points to close
            (points[:, depth_index] < 10.0)  # do not consider points to far
        ]

        if front_points.size == 0:
            self.get_logger().info("No obstacles detected in front of the boat...")
            min_distance = float('inf')  
        else:
            min_distance = np.min(front_points[:, 0])
            posx_min_distance = np.where(front_points[:, 0] == (front_points[:, 0]).min()) #find the posotion in the array of the closest points to the boat
            if len(posx_min_distance) > 1:
                pos_min_distance = posx_min_distance[0] 
            else:
                pos_min_distance = posx_min_distance


        ################ avoiding obstacle procedure #####################
        left_motor = Float32()
        right_motor = Float32()
        
        if self.target_offset is not None:
            if min_distance <= SECURITY_DISTANCE and (self.num_white_pix / (640 * 480)) > PERC_OF_WHITE_PIX and abs(self.target_offset) <= 0.40:
                left_motor.data = 0.06 * self.target_offset  # both motors at almost power off, they produce little power in order to keep the boat still against the waves
                right_motor.data = -0.06 * self.target_offset
                print("Target reached successfully!")
            elif min_distance <= SECURITY_DISTANCE:
                if float(front_points[pos_min_distance, 1]) >= 0:
                    self.get_logger().warn(f"Obstacle found at: {min_distance:.2f}m! Going right...")
                else:
                    self.get_logger().warn(f"Obstacle found at: {min_distance:.2f}m! Going left...")
                left_motor.data = FORWARD_SPEED + 0.025 * (1.2 - float(front_points[pos_min_distance, 1])) * np.sign(float(front_points[pos_min_distance, 1]))  # avoiding the obstacle
                right_motor.data = FORWARD_SPEED - 0.025 * (1.2 - float(front_points[pos_min_distance, 1])) * np.sign(float(front_points[pos_min_distance, 1]))
            elif min_distance > SECURITY_DISTANCE and abs(self.target_offset) <= 0.03:
                left_motor.data = FORWARD_SPEED  # both motors power on
                right_motor.data = FORWARD_SPEED
            elif min_distance > SECURITY_DISTANCE and abs(self.target_offset) > 0.03:
                left_motor.data = FORWARD_SPEED + 0.06 * self.target_offset  # turning towards the target
                right_motor.data = FORWARD_SPEED - 0.06 * self.target_offset
            else:
                left_motor.data = FORWARD_SPEED  # both motors power on
                right_motor.data = FORWARD_SPEED

        else:
            self.get_logger().warn("No target detected! Turning clockwise...")
            left_motor.data = 0.1  # turning clockwise to find the target
            right_motor.data = -0.1

        #publish the values of the motors
        self.motor_left_pub.publish(left_motor)
        self.motor_right_pub.publish(right_motor)


def main(args=None):
    rclpy.init(args=args)
    node = TargetFollowing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
