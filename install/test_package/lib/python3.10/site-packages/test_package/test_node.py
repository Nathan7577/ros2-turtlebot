import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Publisher(Node):
    
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(Twist, 'robot_0/cmd_vel', 10)
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Publisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()