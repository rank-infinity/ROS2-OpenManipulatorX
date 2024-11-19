import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys


class BasicRobotControl(Node):
    def __init__(self):
        super().__init__('basic_robot_control')
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')

        self.tool_control_req = SetJointPosition.Request()

        self.send_request()
        self.send_tool_control_request()

    def send_tool_control_request(self):
        self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2',
        'joint3', 'joint4', 'gripper']
        self.tool_control_req.joint_position.position = [0.0, 0.0, 0.0, 0.0, 0.01]
        self.tool_control_req.path_time = 1.0
        try:
            self.tool_control_result = self.tool_control.call_async(self.tool_control_req)
            self.tool_control.call_async(self.tool_control_req)
        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))

    def send_request(self):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        request.joint_position.position = [-0.7, 0.0, 0.0, 0.0, 0.0]
        request.path_time = 1.0

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

def main(args=None):
    rclpy.init(args=args)

    basic_robot_control = BasicRobotControl()

    while rclpy.ok():
        rclpy.spin_once(basic_robot_control)
        print(basic_robot_control.future.done())
        if basic_robot_control.future.done():
            try:
                response = basic_robot_control.future.result()
                print(response)
            except Exception as e:
                basic_robot_control.get_logger().error('Service call failed %r' % (e,))
            break

    basic_robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
