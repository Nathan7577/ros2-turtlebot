import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionClient
import numpy as np
import time

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.action import RotateAngle

namespace = 'robot_0'  

class wall_follow_node(Node):
    
    def __init__(self):
        # initialize node and give it a name
        super().__init__('wall_follow_node')

        # define custom QoS profile for the subscription to match the IR Sensor
        custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        #create publisher 
        self.publisher_ = self.create_publisher(Twist, 'robot_0/cmd_vel', 1)

        #create subscriber
        self.ir_subscription = self.create_subscription(IrIntensityVector, 'robot_0/ir_intensity', self.ir_callback, qos_profile=custom_qos)
        
        #create action client
        # self.turn_client = ActionClient(self, RotateAngle, 'robot_0/rotate_angle')


    def ir_callback(self, msg_in):
        # generates commands based on ir values
        
        # define output message type
        msg_out = Twist() 
        
        # read in values from the IR sensors (type uint16)

        readings = [msg_in.readings[0].value, msg_in.readings[1].value, msg_in.readings[2].value, 
                    msg_in.readings[3].value, msg_in.readings[4].value, msg_in.readings[5].value,
                    msg_in.readings[6].value]

        left = np.average(readings[:2])
        front = np.average(readings[2:5])
        right = np.average(readings[-2:])
        left_front = np.average(readings[:5])
        
        # sensors set 0 as the infinity value this changes them so the logic works
        if left < 6:
            left = 1000
        
        if front < 6:
            front = 1000
        
        if right < 6:
            right = 1000
        
        # set sensor thresholds and stright zone threshold
        front_threshold = np.uint16(700)
        left_threshold = np.uint16(700)
        straight_zone_bound = np.uint16(100)
        right_threshold = np.uint16(400)
        left_des = np.uint16(50)
        stop_rotate = np.uint16(100)


        
        print('')
        print('left          ' + str(left))
        print('front         ' + str(front))

        if front < stop_rotate:
            print("emergency right turn")
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -1.0

        elif front < front_threshold and left < left_threshold: # right turn
            print("turning right")
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -1.0

        elif left == 1000: # left turn
            print('turning left')
            msg_out.linear.x = 0.1
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.35

        elif left >= left_des:
            print('DRIVE STRAIGHT, WALL')
            msg_out.linear.x = 0.1
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -0.6
        else:
            print('DRIVE STRAIGHT, NO WALL')
            msg_out.linear.x = 0.1
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.6

        #publish message
        self.publisher_.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = wall_follow_node()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':
    main()