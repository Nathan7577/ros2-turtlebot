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
        self.publisher_ = self.create_publisher(Twist, 'robot_0/cmd_vel', 10)

        #create subscriber
        self.ir_subscription = self.create_subscription(IrIntensityVector, 'robot_0/ir_intensity', self.ir_callback, qos_profile=custom_qos)
        
        #create action client
        # self.turn_client = ActionClient(self, RotateAngle, 'robot_0/rotate_angle')

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
        

        # set sensor thresholds and stright zone threshold
        front_threshold = np.uint16(50)
        left_threshold = np.uint16(50)
        straight_zone_bound = np.uint16(100)
        left_des = np.uint16(50)

        if front_center_left < front_threshold and side_left >= straight_zone_bound: # right turn
            print('TURN RIGHT')
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -0.5
            for i in range(10):
                self.publisher_.publish(msg_out)
                time.sleep(.05)
                print('RIGHT LOOP')

        elif front_center_left > front_threshold and side_left > left_threshold: # left turn
            print('TURN LEFT')
            msg_out.linear.x = 0.0
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.5
            for i in range(100):
                self.publisher_.publish(msg_out)

        elif side_left >= left_des:
            print('DRIVE STRIGHT, WALL')
            msg_out.linear.x = 0.25
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = -0.5
        else:
            print('DRIVE STRIGHT, NO WALL')
            msg_out.linear.x = 0.5
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = 0.5
        # publish message
        self.publisher_.publish(msg_out)

# class RotateActionClient(Node):
#     '''
#     This is an action client. Action clients send goal requests to action servers,
#     which sends goal feedback and results back to action clients. This action client
#     tells the robot to turn 90 degrees at a speed of 0.15. Subclass of Node.
#     '''

#     def __init__(self):
#         super().__init__('rotate_action_client') #create node
                
#         self._action_client = ActionClient(self, RotateAngle, namespace + '/rotate_angle') # create action client

#     def send_goal(self, angle=1.57, max_rotation_speed=0.5):

#         goal_msg = RotateAngle.Goal()
#         goal_msg.angle = angle 
#         goal_msg.max_rotation_speed = max_rotation_speed
#         '''
#         This method waits for the action server to be available.
#         '''
#         print('Waiting for action server to be available...')
#         self._action_client.wait_for_server()
#         '''
#         Sends a goal to the server.
#         '''   
#         print('Action server available. Sending rotate goal to server.')
#         self._send_goal_future = self._action_client.send_goal_async(goal_msg)
#         '''
#         Returns a future to a goal handle. We need to register a callback 
#         for when the future is complete.
#         '''        
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         '''
#         A callback that is executed when the future is complete.
#         The future is completed when an action server accepts or rejects the goal request.
#         Since there will be no result, we can check and determine if the goal was rejected
#         and return early. 
#         '''
#         print('Checking if goal was accepted or rejected...')
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected :(')
#             return

#         self.get_logger().info('Goal accepted :)')
#         '''
#         We can request to see if the goal request was accepted or rejected.
#         Future will complete when the result is ready.
#         This step is registering a callback (similar to that of the goal response).
#         '''
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         '''
#         Here, we are logging the result sequence.
#         '''
#         result = future.result().result
#         self.get_logger().info('Result: {0}'.format(result))
#         '''
#         This shuts down the node.
#         '''        
#         print('Shutting down rotate action client node.')
#         rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = wall_follow_node()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()