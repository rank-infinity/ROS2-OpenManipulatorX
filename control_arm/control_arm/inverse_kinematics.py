import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose

from scipy.spatial.transform import Rotation
import numpy as np
from .arm import Arm

import math

# Define the inverse kinematics solver class
class InverseKinematicsSolver(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_solver')
        self.srv = self.create_service(InverseKinematics, 'inverse_kinematics', self.handle_inverse_kinematics_request)

    def handle_inverse_kinematics_request(self, request, response):
        # get theta1 from provided matrix
        
        
        #get phi and position of frame 3 
        r2 = request.r2
        z2 = request.z2
        phi = request.phi
        
        # Kinematic parameters (you can adjust these as per your robot)
        a1 = 96.326 # Link 1 length
        a2 = 130.2305  # Link 2 length 
        a3 = 124.0  # Link 3 length 
        a4 = 133.4  # Link 4 length

        # Solve for theta3 using the cosine law
        cos_theta3 = (r2**2 + z2**2 - (a2**2 + a3**2)) / (2 * a2 * a3)
        sin_theta3 = math.sqrt(1 - cos_theta3**2) # sin(theta3) to get positive or negative angle

        # Calculate theta3
        theta3 = math.atan2(sin_theta3, cos_theta3)

        # Calculate cos(theta2) and sin(theta2) using equations 23 and 24
        cos_theta2 = ((a2 + a3 * math.cos(theta3)) * r2 + (a3 * math.sin(theta3)) * z2) / (r2**2 + z2**2)
        sin_theta2 = ((a2 + a3 * math.cos(theta3)) * z2 + (a3 * math.sin(theta3)) * r2) / (r2**2 + z2**2)

        # Compute theta2
        theta2 = math.atan2(sin_theta2, cos_theta2)

        # Compute theta4 from the relationship given
        theta4 = phi - (theta2 + theta3)

        # Fill in the response
        response.theta2 = theta2
        response.theta3 = theta3
        response.theta4 = theta4
        return response

def main(args=None):
    rclpy.init(args=args)


    inverse_kinematics_solver = InverseKinematicsSolver()
    rclpy.spin(inverse_kinematics_solver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()