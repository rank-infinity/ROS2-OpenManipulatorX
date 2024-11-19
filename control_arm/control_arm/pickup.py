import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
from geometry_msgs.msg import Pose
import sys
from .arm import Arm
from custom_interfaces.srv import Pose2Joint
import time

class RobotControl(Node):
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
        # self.send_request()

    def send_tool_control_request(self, open=True):
        if open:
            gripper = 0.01
        else:
            gripper = -0.01
        self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2',
        'joint3', 'joint4', 'gripper']
        self.tool_control_req.joint_position.position = [0.0, 0.0, 0.0, 0.0, gripper]
        self.tool_control_req.path_time = 1.0
        try:
            print("Opening")
            self.tool_control_result = self.tool_control.call_async(self.tool_control_req)
            self.future = self.tool_control.call_async(self.tool_control_req)
        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))

        rclpy.spin_until_future_complete(self, self.future)
        print("opened")

    def send_request(self, position=[0.0, 0.0, 0.0, 0.0, 0.0], path_time=1.0):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        request.joint_position.position = position
        request.path_time = path_time

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        print("Reached Position!")

class Pickup(Node):
    def __init__(self, object_pose:Pose=None):
        super().__init__('inverse_kinematics')
        self.arm = Arm()
        self.robot_control = RobotControl()
        # self.inv_kin_client = self.create_client(Pose2Joint, 'move_to_pose')
        # while not self.inv_kin_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Inverse Kinematics service not available')
        self.object_pose = object_pose

    def set_object_pose(self, object_pose:Pose):
        self.object_pose = object_pose

    def pickup_object(self):
        # above_object_pose = self.object_pose

        # above_object_pose.position.z = 100
        # above_object_pose.orientation.x = 0.0
        # above_object_pose.orientation.y = 0.0
        # above_object_pose.orientation.z = 0.0
        # above_object_pose.orientation.w = 1.0

        # self.object_pose.orientation.x = 0.0
        # self.object_pose.orientation.y = 0.0
        # self.object_pose.orientation.z = 0.0
        # self.object_pose.orientation.w = 1.0

        # above_object_joint_states = self.inv_kin_client.call(above_object_pose)
        # object_joint_states = self.inv_kin_client.call(object_joint_states)
        above_object_joint_states = [-0.707, 0.3, -0.3, 1.4, 0.0]
        object_joint_states = [-0.707, 0.3, 0.3, 1.4, 0.0]
        
        # self.robot_control.send_request(position=[0.7, 0.0,0.0,0.0,0.0])

        self.robot_control.send_request(above_object_joint_states)
        self.robot_control.send_tool_control_request(open=True)
        time.sleep(1)
        self.robot_control.send_request(object_joint_states)
        self.robot_control.send_tool_control_request(open=False)   
        time.sleep(1)     
        self.robot_control.send_request(above_object_joint_states)


def main(args=None):
    rclpy.init(args=args)
    
    object_position = [0.0,0.0,0.0]
    p = Pickup(object_position)
    p.pickup_object()


    rclpy.shutdown()

if __name__ == '__main__':
    main()
