import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from keyboard_msgs.msg import Key


class Publisher(Node):
    
    def __init__(self):
        # initialize node and give it a name
        super().__init__('keyboard_teleop')

        #create publisher 
        self.publisher_ = self.create_publisher(Twist, 'robot_0/cmd_vel', 10)
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        #create subscriber
        self.subscription = self.create_subscription(Key, 'keydown', self.listener_callback, 10)
        self.subscription


    def listener_callback(self, msg_in):
        #function to print when the subscriber recieves data
        msg_out = Twist()
        if msg_in.code == 273:
            print('UP_ARROW')
            msg_out.linear.x = 0.5
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.0
        elif msg_in.code == 274:
            print('DOWN_ARROW')
            msg_out.linear.x = -0.5
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.0
        elif msg_in.code == 275:
            print('RIGHT_ARROW')
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -0.5
        elif msg_in.code == 276:
            print('LEFT_ARROW')
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.5
        else:
            print('NO_INPUT')
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.0
        self.get_logger().info('Key pressed')
        self.publisher_.publish(msg_out)

        
    # def timer_callback(self):
    #     msg = Twist()
    #     msg.linear.x = 0.0
    #     msg.linear.y = 0.0
    #     msg.linear.z = 0.0
    #     msg.angular.x = 0.0
    #     msg.angular.y = 0.0
    #     msg.angular.z = 0.0
    #     self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Publisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()