
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import rclpy
import sys
import math
import time

class TurtleBot(Node):

     def __init__(self):
         # Creates a node with name 'turtlebot_controller' and make sure it is a
         # unique node (using anonymous=True).
         super().__init__('turtlebot_controller')
 
         # Publisher which will publish to the topic '/turtle1/cmd_vel'.
         self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
 
         # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
         # when a message of type Pose is received.
         self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
 
         self.pose = Pose()
         self.timer = self.create_timer(0.1, self.move2goal)

     def update_pose(self, data):
         """Callback function which is called when a new message of type Pose is
         received by the subscriber."""
         self.pose = data
 
     def steering_angle(self, goal_pose):
         return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

     def move2goal(self):
         """Moves the turtle to the goal."""
         const = 5
         vel_msg = Twist()
         goal_pose = Pose()
 
         # Get the input from the user.
         goal_pose.x = float(sys.argv[1])
         goal_pose.y = float(sys.argv[2])
         goal_pose.theta = float(sys.argv[3])*180/3.14
         tolerance =  0.01 
 		
         if (goal_pose.x>10 or goal_pose.x<0 or goal_pose.y>10 or goal_pose.y<0 or goal_pose.theta>360 or goal_pose.theta<-360):
            self.get_logger().info("Wrong data")
            quit()
 
         distance_to_goal = math.sqrt(math.pow((goal_pose.x - self.pose.x), 2) + math.pow((goal_pose.y - self.pose.y), 2))
         '''
          angle_error = self.steering_angle(goal_pose) - self.pose.theta
         
         if abs(angle_error) > tolerance:
             vel_msg.angular.z = const*angle_error  
         elif distance_to_goal >= tolerance:
             # Linear velocity in the x-axis.
             vel_msg.linear.x = const*distance_to_goal 
         else:
             vel_msg.linear.x = 0.0
             vel_msg.angular.z = goal_pose.theta
             self.get_logger().info("Goal Reached!! ")
             quit()
          '''
         #self.get_logger().info('%s" ' % goal_pose.theta) 
         angle = self.steering_angle(goal_pose) 
         vel_msg.angular.z = -self.pose.theta
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         vel_msg.angular.z = angle
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         vel_msg.linear.x = distance_to_goal
         vel_msg.angular.z = 0.0
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         vel_msg.linear.x = 0.0
         vel_msg.angular.z = -angle
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)
         #this is turn angle
         vel_msg.angular.z = goal_pose.theta
         self.velocity_publisher.publish(vel_msg)
         time.sleep(2)

         self.get_logger().info("Goal Reached!! ")
         quit()
         
def main(args=None):
    rclpy.init(args=args)
    x = TurtleBot()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

 
if __name__ == '__main__':
    main()
