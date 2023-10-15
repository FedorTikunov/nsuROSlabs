from action_turtle_interfaces.action import MessageTurtleCommands

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

tasks = []

class MessageActionClient(Node):

    def __init__(self):
        super().__init__('action_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'execute_turtle_commands')

    def send_goal(self, command, s, angle):
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.s = s
        goal_msg.angle = angle
        goal_msg.command = command


        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        global goals
        goals.pop(0)
        if goals:
            self.send_goal(*tasks[0])
        else:
            rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    global tasks
    rclpy.init(args=args)

    action_client = MessageActionClient()
    
    tasks.append(['forward', 2, 0])
    tasks.append(['turn_right', 0, 90])
    tasks.append(['forward', 1, 0])
    
    action_client.send_goal(*tasks[0])

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
