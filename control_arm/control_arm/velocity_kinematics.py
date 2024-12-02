from custom_interfaces.srv import Joint2Twist
from custom_interfaces.srv import Twist2Joint

import rclpy
from rclpy.node import Node
from .arm import Arm
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import numpy as np


class VeloctiyKinematics(Node):

    def __init__(self, arm_params=None):
        super().__init__('velocity_kinematics')
        self.arm = Arm(arm_params)

        self.joint_positions=[1,1,1,1]

        #need joint states to calculate jacobian
        self.joint_state_subscription = self.create_subscription( # subscribe to the topic publishing q1,q2,q3
            JointState,                         # define the type of msg
            '/joint_states',                    # define the topic name
            self.joint_state_cb,                # define the callback function
            10                                  # define the message queue size
        )  

        # joint angles- JointState to Twist
        self.calc_eef_vel= self.create_service(Joint2Twist,'calculate_eef_vel', self.calc_eef_cb)
        self.calc_joint_vel= self.create_service(Twist2Joint,'calculate_joint_vel', self.calc_joint_cb)
        
    def joint_state_cb(self, msg):
        self.joint_positions= msg.position[0:4]


    # ros2 service call /calculate_eef_vel custom_interfaces/srv/Joint2Twist "{joint_velocity: { data: [0.5, 1.0, 1.0, 1.0]}}"
    #Joint to Twist
    #Need to send joint positions
    def calc_eef_cb(self, request, response):
        if(len(self.joint_positions)==0):
            print("Joint states not initiated")
            return 
        J= self.arm.get_J(self.joint_positions)
        joint_velocity= np.array(request.joint_velocity.data)
        print(joint_velocity)

        twist= np.matmul(J,joint_velocity)        
        print(twist)

        twist_array= Twist()
        twist_array.linear.x= twist[0]
        twist_array.linear.y= twist[1]
        twist_array.linear.z= twist[2]
        twist_array.angular.x= twist[3]
        twist_array.angular.y= twist[4]
        twist_array.angular.z= twist[5]
        response.twist=twist_array
        print(response)
        return response
    
    # ros2 service call /calculate_joint_vel custom_interfaces/srv/Twist2Joint "{twist: {linear: {x: 1.0, y: 1.0, z: 1.0}, angular: {x: 1.0, y: 1.0, z: 1.0}}}"
    #Twist to Joint
    def calc_joint_cb(self, request, response):
        if(len(self.joint_positions)==0):
            print("Joint states not initiated")
            return 
        J= self.arm.get_J(self.joint_positions)
        Jinv= np.linalg.pinv(J)
        print("Jinv- ", Jinv)
        twist= request.twist
        twist_array= np.array([[twist.linear.x],
                               [twist.linear.y],
                               [twist.linear.z],
                               [twist.angular.x],
                               [twist.angular.y],
                               [twist.angular.z]])

        joint_values= np.matmul(Jinv, twist_array)
        print("joint values- ", joint_values[0])
        joint_info= JointState()
        #ensure 4 values are put in
        joint_info.velocity= joint_values[0].tolist()
        response.joint_state = joint_info
        print(response)
        return response      

        
    

def main(args=None):
    rclpy.init(args=args)

    arm_params = [[0,                   96.326,   0,         np.deg2rad(-90)],
                  [np.deg2rad(-79.38),   0,        130.2305,  0],
                  [np.deg2rad(79.38),  0,        124,       0],
                  [0,                   0,        133.4,     0]]
    
    velKin = VeloctiyKinematics(arm_params)

    rclpy.spin(velKin)
    rclpy.shutdown()


if __name__=='__main__':
    main()