from custom_interfaces.srv import Move

import rclpy
from rclpy.node import Node
from .arm import Arm
from std_msgs.msg import Float32MultiArray

class MoveService(Node):
    

    def __init__(self):
        super().__init__('move_service')
        self.arm  = Arm()
        
        self.serv= self.create_service(Move, 'move_to_pose', self.move_to_pose_cb)

    def move_to_pose_cb(self, request, response):
        angles= self.arm.get_joint_angles(request.pose)
        print("Needed joint configuration- ", angles)
        
        float_array= Float32MultiArray()
        float_array.data.extend(angles)
        response.array= float_array
        return response
    

def main(args= None):
    rclpy.init(args=args)
    move_service = MoveService()

    rclpy.spin(move_service)
    rclpy.shutdown()


if __name__=='__main__':
    main()