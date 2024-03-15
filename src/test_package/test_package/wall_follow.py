import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

from example_interfaces.srv import Trigger
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector

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
        
        #create service client
        # self.srv = self.create_service(Trigger, 'wall_contacted', self.wall_contacted_callback)


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
        left_des = np.uint16(50)
        stop_rotate = np.uint16(100)


        
        print('')
        # print('left          ' + str(left))
        # print('front         ' + str(front))

        if front < stop_rotate:
            # print("emergency right turn")
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -1.0
            # self.wall_contacted_callback()

        elif front < front_threshold and left < left_threshold: # right turn
            # print("turning right")
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -1.0
            # self.wall_contacted_callback()

        elif left == 1000: # left turn
            # print('turning left')
            msg_out.linear.x = 0.1
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.35
            # self.wall_contacted_callback()

        elif left >= left_des:
            # print('DRIVE STRAIGHT, WALL')
            msg_out.linear.x = 0.1
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -0.6
            # self.wall_contacted_callback()

        else:
            # print('DRIVE STRAIGHT, NO WALL')
            msg_out.linear.x = 0.1
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.6

        #publish message
        self.publisher_.publish(msg_out)

    # def wall_contacted_callback(self): 
    #     print('WALL CONTACT SERVICE CALLED')
    #     rclpy.spin_until_future_complete(self)
    #     return

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = wall_follow_node()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':
    main()