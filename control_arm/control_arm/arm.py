import numpy as np
from scipy.spatial.transform import Rotation as R

class Arm:
<<<<<<< HEAD
    def __init__(self, arm_params=None):
        self.arm_params = arm_params

    def set_arm_params(self, arm_params):
        self.arm_params = arm_params
=======
    def __init__(self, arm_params=None, frame_origins=None, z_axis=None):
        self.arm_params = arm_params
        self.frame_origins= frame_origins
        self.z_axis= z_axis

    def set_arm_params(self, arm_params, frame_origins):
        self.arm_params = arm_params
        self.frame_origins= frame_origins

    def set_frame_origins(self, frame_origins):
        self.frame_origins= frame_origins

    def set_z_axis(self, z_axis):
        self.z_axis= z_axis
>>>>>>> 55be825 (Added velocity node Assigment 2 part 1)
    
    def __calculate_dh_transform(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def __get_link_values(self):
        a1 = 96.326 # Link 1 length
        l2a = 128.0
        l2b = 24.0
        a2 = 130.2305  # Link 2 length 
        a3 = 124.0  # Link 3 length 
        a4 = 133.4  # Link 4 length
        return np.array([a1, l2a, l2b, a2,  a3, a4])

<<<<<<< HEAD
    def calculate_jacobian(self):
        pass
    
=======

# UNCOMMENT AND WORK ON THIS
    # def calculate_jacobian(self, joint_states):
    #     if self.arm_params is None:
    #         raise TypeError("arm params for the arm is None, expected a list")
    #     assert(len(joint_states) == len(self.arm_params)) # the /joint_states topic has the gripper position as well, so just to be sure the correct joint states are passed
        
    #     if self.frame_origins is None:
    #         raise TypeError("frame origins for the arm is None, expected a list")
        
    #     if self.z_axis is None:
    #         raise TypeError("z-axis orientations for the arm is None, expected a list")
        
    #     size= len(joint_states)

    #     #translated origins
    #     all_H= self.get_all_H()
    #     original_z = np.vstack([np.transpose(np.array([[0,0,1]]))]*(size-1))
    #     translated_z = 

    
    def get_H_Jacobian(self, joint_states):
        dh_params = np.array(self.arm_params)        
        for i in range(len(joint_states)):
            dh_params[i][0] += joint_states[i]

        H_list= np.array([])
        H = np.eye(len(dh_params))
        for i in range(0, till_i):
            H = H @ self.__calculate_dh_transform(*dh_params[i])

        return H
    
    def translated_origin(self, H):
        coord = np.transpose(np.array([0,0,0,1]))
        return H @ coord
    
    def translated_z(self, H):
        z_axis=np.transpose(np.array([0,0,1]))
        R= H[:3,:3]
        return R @ z_axis
        
>>>>>>> 55be825 (Added velocity node Assigment 2 part 1)
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
    
    def get_all_H(self, joint_states):
        dh_params = np.array(self.arm_params)        
        for i in range(len(joint_states)):
            dh_params[i][0] += joint_states[i]

        # print(dh_params)
        all_H = []
        H = np.eye(len(dh_params))
        for i in range(len(dh_params)):
            H = H @ self.__calculate_dh_transform(*dh_params[i])
            all_H.append(H)

        return all_H

<<<<<<< HEAD
=======
    def get_J(self, joint_states):
        all_H= self.get_all_H(joint_states)
        for i in range(len(joint_states)):
            if(i==0):
                J= self.getJi(i, all_H)
            else:
                J=np.concatenate((J,self.getJi(i, all_H)), axis=1 )
        print("J- ", J)
        return J
    
    
    def getJi(self, i, all_H):
        w= self.get_zi(i, all_H)
        v= np.transpose(np.cross(w, self.get_o_dist(i, all_H)))
        w= np.transpose(w)
        print("w- ", w, " v- ",v)
        Ji= np.concatenate((v,w), axis=0)
        Ji= np.reshape(Ji, (6,1))
        return Ji

    def get_zi(self, i, all_H):
        z=np.array([0,0,1,0])
        if i>0:
            return (all_H[i] @ z)[:3]
        return z[:3]

    def get_o_dist(self, i, all_H):
        o= np.array([0,0,0,1])
        o4= all_H[-1] @ o
        if i>0:
            o= all_H[i-1] @ o
        return (o4-o)[:3]    


>>>>>>> 55be825 (Added velocity node Assigment 2 part 1)
    def get_joint_angles(self, pose):
        print(pose)
        (l1, l2a, l2b, l2, l3, l4)= self.__get_link_values()
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        R4_0 = R.from_quat(quaternion).as_matrix()
        print(R4_0)
        o4_0 = np.array([pose.position.x, pose.position.y, pose.position.z])
        o3_0 = o4_0 - l4*(R4_0 @ np.array([[1.0], [0.0], [0.0]])).flatten()

        r = np.sqrt(o3_0[0]*o3_0[0] + o3_0[1]*o3_0[1])
        z = o3_0[2] - l1

        beta = np.arccos((r**2 + z**2 - l2**2 - l3**2)/(2*l2*l3))
        alpha = np.arctan2(z, r) + np.arctan2((l3*np.sin(beta)), (l2+l3*np.cos(beta)))

        t = np.arctan2(l2b, l2a)

        theta1 = np.arctan2(o4_0[1], o4_0[0])
        theta2 = np.pi/2.0 - t - alpha
        theta3 = beta - np.pi/2.0 + t

        joint_states = [theta1, theta2, theta3]
        all_H = self.get_all_H(joint_states)
        R3_0 = all_H[-2][0:3, 0:3]
        R4_3 = np.linalg.inv(R3_0) @ R4_0
        
        _, _, theta4 = R.from_matrix(R4_3).as_euler("XYZ", degrees=False)
        

        return [theta1, theta2, theta3, theta4]

# TESTING

# from geometry_msgs.msg import Pose

# def mat2Pose(matrix):
#     # refer to https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
#     q = R.from_matrix(matrix[0:3, 0:3]).as_quat() # Convert the Rotation matrix to quaternion
    
#     pose = Pose()

#     pose.position.x = matrix[0][-1]     # extract the position from the last column of the homogenous matrix
#     pose.position.y = matrix[1][-1] 
#     pose.position.z = matrix[2][-1] 
    
#     pose.orientation.x = q[0]
#     pose.orientation.y = q[1]
#     pose.orientation.z = q[2]
#     pose.orientation.w = q[3]

#     # print(Rotation.from_quat(q).as_euler("XYZ"))

#     return pose

# arm_params = [[0,                   96.326,   0,         np.deg2rad(-90)],
#                   [np.deg2rad(-79.38),   0,        130.2305,  0],
#                   [np.deg2rad(79.38),  0,        124,       0],
#                   [0,                   0,        133.4,     0]]
# a = Arm(arm_params)

# joint_states = [0.0, 0.0, 0.0, 1.57]
# pose = mat2Pose(a.get_eef_pose(joint_states))
# print(pose)

# pose = Pose()
# pose.position.x = 200.0
# pose.position.y = 100.0
# pose.position.z = 200.0
# pose.orientation.x = 0.5
# pose.orientation.y = 0.5
# pose.orientation.z = 0.5
# pose.orientation.w = 0.5
# print(a.get_joint_angles(pose))