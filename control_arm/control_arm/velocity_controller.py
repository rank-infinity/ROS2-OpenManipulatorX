# (3.5 pts) Write a node that provides incremental position references to the robot joints, i.e.
# q_ref = q_ref_old + delta_q * sampling_time. The node would then send the q_ref to the joint
# position controllers of the robot as joint goals. You will use this node just like a velocity
# controller as follows in the next item.


import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

import sys
from .arm import Arm
from custom_interfaces.srv import Twist2Joint
import time
import numpy as np


class RobotVelocityControl(Node):
    def __init__(self):
        super().__init__('robot_velocity_control')
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        self.tool_control_req = SetJointPosition.Request()

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
        send_position = []
        send_position.append(position[0])
        send_position.append(position[1])
        send_position.append(position[2])
        send_position.append(position[3])
        send_position.append(0.0)
        request.joint_position.position = send_position
        request.path_time = path_time

        try:
            self.client.call_async(request)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        print("Reached Position!")



class MoveVelocity(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.robot_control= RobotVelocityControl()

        self.vel_client= self.create_client(Twist2Joint, 'calculate_joint_velocity')
        while not self.vel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Velocity Kinematics service not available')

        self.joint_state_subscription = self.create_subscription( # subscribe to the topic publishing q1,q2,q3
            JointState,                         # define the type of msg
            '/joint_states',                    # define the topic name
            self.joint_state_cb,                # define the callback function
            10                                  # define the message queue size
        ) 

        self.req= Twist2Joint.Request()


    def joint_state_cb(self, msg):
        self.joint_positions= msg.position[0:4]

    def send_request(self, twist):
        self.req.twist= twist
        self.future = self.vel_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def set_zero_pos(self):
        self.robot_control.send_request([0.0, 0.0, 0.0, 0.0, 0.0])
    
    def move_with(self, time, twist):
        current_joint_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.set_zero_pos()

        start= time.time()
        while( time.time() - start < time):
            # List of joint velocities
            jointVels = np.array(self.send_request(twist).joint_velocity.data.tolist())
            current_joint_pos += jointVels*0.1
            self.robot_control.send_request(current_joint_pos.tolist())
            time.sleep(0.1)
            







        
