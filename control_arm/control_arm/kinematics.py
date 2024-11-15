import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from scipy.spatial.transform import Rotation
import numpy as np
from .arm import Arm

class EndEffector(Node):
    def __init__(self, arm_params=None):
        super().__init__('end_effector')
        
        self.arm  = Arm(arm_params)
        self.joint_state_subscription = self.create_subscription( # subscribe to the topic publishing q1,q2,q3
            JointState,                         # define the type of msg
            '/joint_states',                    # define the topic name
            self.joint_state_cb,                # define the callback function
            10                                  # define the message queue size
        )        
        self.pose_publisher = self.create_publisher(
            Pose,
            'pose',
            10
        )

    def mat2Pose(self, matrix):
        # refer to https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
        q = Rotation.from_matrix(matrix[0:3, 0:3]).as_quat() # Convert the Rotation matrix to quaternion
        
        pose = Pose()

        pose.position.x = matrix[0][-1]     # extract the position from the last column of the homogenous matrix
        pose.position.y = matrix[1][-1] 
        pose.position.z = matrix[2][-1] 
        
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # print(Rotation.from_quat(q).as_euler("XYZ"))

        return pose

    def joint_state_cb(self, msg:JointState):
        joint_state = np.array(msg.position[0:4])           # get the joint states of the first 4 actuators
        eef_pose_mat = self.arm.get_eef_pose(joint_state)   # get the end effector pose as a 4x4 homogenous matrix
        pose = self.mat2Pose(eef_pose_mat)                  # convert the pose to geometry_msgs/Pose
        self.pose_publisher.publish(pose)
        
     
        
def main(args=None):
    rclpy.init(args=args)
    
    # same as DH params of the arm with q1,q1,q3,q4 = 0
    # we will dynamically add the qis to this to get the realtime dh params to get forward/inverse kinematics
    arm_params = [[0,                   96.326,   0,         np.deg2rad(90)],
                  [np.deg2rad(79.38),   0,        130.2305,  0],
                  [np.deg2rad(-79.38),  0,        124,       0],
                  [0,                   0,        133.4,     0]]
    
    eef = EndEffector(arm_params) 
    rclpy.spin(eef)

    # would be destroyed anyways by the garbage collector, but just to be sure!
    eef.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()
