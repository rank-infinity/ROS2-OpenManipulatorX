from custom_interfaces.srv import Pose2Joint
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.client = self.create_client(Pose2Joint, 'move_to_pose')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')
        
        self.req = Pose2Joint.Request()
    
    def send_request(self, pose):
        self.req.pose = pose
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    

def main(args=None):
    rclpy.init(args=args)

    move_client = MoveRobot()
    request = Pose2Joint.Request()

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