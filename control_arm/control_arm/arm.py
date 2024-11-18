import numpy as np
from scipy.spatial.transform import Rotation as R

class Arm:
    def __init__(self, arm_params=None):
        self.arm_params = arm_params

    def set_arm_params(self, arm_params):
        self.arm_params = arm_params
    
    def __calculate_dh_transform(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def __get_link_values(self):
        a1 = 96.326 # Link 1 length
        a2 = 130.2305  # Link 2 length 
        a3 = 124.0  # Link 3 length 
        a4 = 133.4  # Link 4 length
        return np.array([a1, a2, a3, a4])

    def calculate_jacobian(self):
        pass
    
    def get_eef_pose(self, joint_states):
        if self.arm_params is None:
            raise TypeError("arm params for the arm is None, expected a list")
        assert(len(joint_states) == len(self.arm_params)) # the /joint_states topic has the gripper position as well, so just to be sure the correct joint states are passed
        
        # arm params are essentially dh params but with q1,q2,q3,q4 = 0. So we add the current qis to get the realtime dh params
        dh_params = np.array(self.arm_params)
        for i in range(len(joint_states)):
            dh_params[i][0] += joint_states[i]

        # print(dh_params)

        H = np.eye(len(dh_params))
        for i in range(len(dh_params)):
            H = H @ self.__calculate_dh_transform(*dh_params[i])

        return H
    
    def get_joint_angles(self, pose):
        # Solve for theta3 using the cosine law
        (a1, a2, a3, a4)= self.__get_link_values()
        
        quaternion= [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation= R.from_quat(quaternion)
        (roll, pitch, yaw)= rotation.as_euler('xyz', degrees= False)

        phi= pitch
        r2= pose.position.x - a4*np.cos(phi)
        z2= pose.position.y - a4*np.sin(phi)

        cos_theta3 = (r2**2 + z2**2 - (a2**2 + a3**2)) / (2 * a2 * a3)
        sin_theta3 = np.sqrt(1 - cos_theta3**2) # sin(theta3) to get positive or negative angle

        # Calculate theta3
        theta3 = np.arctan2(sin_theta3, cos_theta3)

        # Calculate cos(theta2) and sin(theta2) using equations 23 and 24
        cos_theta2 = ((a2 + a3 * np.cos(theta3)) * r2 + (a3 * np.sin(theta3)) * z2) / (r2**2 + z2**2)
        sin_theta2 = ((a2 + a3 * np.cos(theta3)) * z2 + (a3 * np.sin(theta3)) * r2) / (r2**2 + z2**2)

        # Compute theta2
        theta2 = np.arctan2(sin_theta2, cos_theta2)

        # Compute theta4 from the relationship given
        theta4 = phi - (theta2 + theta3)

        theta1 = np.arctan2(pose.position.y, pose.position.x)

        return [theta1, theta2, theta3, theta4]