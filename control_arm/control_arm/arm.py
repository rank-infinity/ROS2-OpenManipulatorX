import numpy as np

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