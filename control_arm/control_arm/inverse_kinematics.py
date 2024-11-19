from custom_interfaces.srv import Pose2Joint

import rclpy
from rclpy.node import Node
from .arm import Arm
from std_msgs.msg import Float32MultiArray
import numpy as np

class InverseKinematics(Node):   

    def __init__(self, arm_params=None):
        super().__init__('inverse_kinematics')
        self.arm  = Arm(arm_params)
        
        self.serv= self.create_service(Pose2Joint, 'calculate_joint_values', self.calculate_joint_values_cb)

        self.pub_joint_values = self.create_publisher(
            Float32MultiArray,
            '/joint_values',
            10
        )

    def calculate_joint_values_cb(self, request, response):
        angles = self.arm.get_joint_angles(request.pose)
        print("Needed joint configuration- ", angles)
        
        float_array = Float32MultiArray()
        float_array.data.extend(angles)
        response.array = float_array
        self.pub_joint_values.publish(float_array)
        print(response)
        return response
    

def main(args= None):
    rclpy.init(args=args)
    arm_params = [[0,                   96.326,   0,         np.deg2rad(-90)],
                  [np.deg2rad(-79.38),   0,        130.2305,  0],
                  [np.deg2rad(79.38),  0,        124,       0],
                  [0,                   0,        133.4,     0]]
    invKin = InverseKinematics(arm_params)

    rclpy.spin(invKin)
    rclpy.shutdown()


if __name__=='__main__':
    main()