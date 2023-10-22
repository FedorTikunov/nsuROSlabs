import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_turtle_interfaces.action import MessageTurtleCommands
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.executors import MultiThreadedExecutor

curr = Pose()

class MessageActionServer(Node):

    def __init__(self):
        super().__init__('action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'execute_turtle_interfaces',
            self.execute_callback)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0
        
        global curr

        x, y, t = curr.x, curr.y, curr.theta

        
        twist = Twist()
        
        if goal_handle.request.command == 'forward':
        	twist.linear.x = float(goal_handle.request.s)
        	self.publisher.publish(twist)
        	while feedback_msg.odom != goal_handle.request.s:
        		feedback_msg.odom = int(math.sqrt((curr.x - x)**2+(curr.y - y)**2))
        		self.get_logger().info(f'went: {curr.x} {x} m')
        		goal_handle.publish_feedback(feedback_msg)

        else:
            if goal_handle.request.command == 'turn_right':
                twist.angular.z = -1.0*float(goal_handle.request.angle)*2*3.14/360  # deg -> rad
                self.publisher.publish(twist)
                while (abs((curr.theta-t)) < float(goal_handle.request.angle)*2*3.14/360):
                    self.get_logger().info(f'rotated: {(curr.theta-t)*360/6.28} degrees')
            else: 
                twist.angular.z = float(goal_handle.request.angle)*2*3.14/360
                self.publisher.publish(twist)
                while (abs((curr.theta-t)) < float(goal_handle.request.angle)*2*3.14/360):
                    self.get_logger().info(f'rotated: {abs(curr.theta-t)*360/6.28} degrees')

        goal_handle.succeed()
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        if goal_handle.request.command == 'forward':
            self.get_logger().info(f'position: {curr.x} {curr.y} m')
        else:
            self.get_logger().info(f'angle: {curr.theta*360/6.28} degrees')
        
        result = MessageTurtleCommands.Result()
        result.result = True
        return result
        

class ActionSubscriber(Node):
    """
    Subscriber node to the current battery state
    """     
    def __init__(self):
   
      # Initialize the class using the constructor
      super().__init__('action_subscriber')
     
      # Create a subscriber 
      # This node subscribes to messages of type
      # Pose
      self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
    	global curr
    	curr = msg


def main(args=None):
    rclpy.init(args=args)

    try:
        action_subscriber = ActionSubscriber()
        action_server = MessageActionServer()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(action_server)
        executor.add_node(action_subscriber)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            action_server.destroy_node()
            action_subscriber.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
