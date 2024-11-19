import sys

from custom_interfaces.srv import Move
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class MoveClientAsync(Node):
    def __init__(self):
        super().__init__('move_client_aync')
        self.client = self.create_client(Move, 'move_to_pose')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')
        
        self.req= Move.Request()
    
    def send_request(self, pose):
        self.req.pose= pose
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    

    

def main(args=None):
    rclpy.init(args=args)

    move_client = MoveClientAsync()
    request = Move.Request()

    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    response = move_client.send_request(pose)

    print("Response- ", response)
    move_client.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()