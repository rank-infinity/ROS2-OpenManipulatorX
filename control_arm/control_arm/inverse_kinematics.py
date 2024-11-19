from custom_interfaces.srv import Pose2Joint

import rclpy
from rclpy.node import Node
from .arm import Arm
from std_msgs.msg import Float32MultiArray

class InverseKinematics(Node):   

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.arm  = Arm()
        
        self.serv= self.create_service(Pose2Joint, 'calculate_joint_values', self.calculate_joint_values_cb)

    def calculate_joint_values_cb(self, request, response):
        angles = self.arm.get_joint_angles(request.pose)
        # print("Needed joint configuration- ", angles)
        
        float_array= Float32MultiArray()
        float_array.data.extend(angles)
        response.array= float_array
        
        return response
    

def main(args= None):
    rclpy.init(args=args)
    invKin = InverseKinematics()

    rclpy.spin(invKin)
    rclpy.shutdown()


if __name__=='__main__':
    main()