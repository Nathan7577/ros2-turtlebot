import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np


from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector


class Publisher(Node):
    
    def __init__(self):
        # initialize node and give it a name
        super().__init__('wall_follow')

        # define custom QoS profile for the subscription to match the IR Sensor
        custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )


        #create publisher 
        self.publisher_ = self.create_publisher(Twist, 'robot_0/cmd_vel', 10)

        #create subscriber
        self.ir_subscription = self.create_subscription(IrIntensityVector, 'robot_0/ir_intensity', self.ir_callback, qos_profile=custom_qos)


    def ir_callback(self, msg_in):
        # generates commands based on ir values
        
        # define output message type
        msg_out = Twist() 
        
        # read in values from the IR sensors (type uint16)
        side_left = msg_in.readings[0].value
        left = msg_in.readings[1].value 
        front_left = msg_in.readings[2].value
        front_center_left = msg_in.readings[3].value
        front_center_right = msg_in.readings[4].value
        front_right = msg_in.readings[5].value
        right = msg_in.readings[6].value
        
        # set distance threshold and typecast to uint16
        threshold = np.uint16(50)

        # Current controller is a bang bang controller which cannot handle turns.
        # 
        # Potential solution: implement a regulation controller based on left sensor data to keep sensor data
        # near a set point. If the data goes outside of specified threshold, robot stops motion and starts
        # rotating counter-clockwise until data is recieved again. 
        #
        # Potential solution 2: specifically set turn logic and wall follow logic sepratley and then assess the 
        # situation based on sensor data. then follow protocall for the given situation.

        # state machine logic for wall following
        if side_left >= threshold:
            print('WALL DETECTED')
            msg_out.linear.x = 0.25
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -0.5
        else:
            print('NOTHING DETECTED')
            msg_out.linear.x = 0.5
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.5

        # publish message
        self.publisher_.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = Publisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()