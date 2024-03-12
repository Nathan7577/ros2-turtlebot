import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


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
        msg_out = Twist()
        # IrIntensity[1] = side_left
        # IrIntensity[2] = left
        # IrIntensity[3] = front_left
        # IrIntensity[4] = front_center_left
        # IrIntensity[5] = front_center_right
        # IrIntensity[6] = front_right
        # IrIntensity[7] = right
        print(msg_in.readings)
        
        if msg_in.readings[1] >= 50:
            print('WALL DETECTED')
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -0.5
        else:
            print('NOTHING DETECTED')
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.5

        self.publisher_.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = Publisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()