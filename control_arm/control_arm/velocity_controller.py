# (3.5 pts) Write a node that provides incremental position references to the robot joints, i.e.
# q_ref = q_ref_old + delta_q * sampling_time. The node would then send the q_ref to the joint
# position controllers of the robot as joint goals. You will use this node just like a velocity
# controller as follows in the next item.

from open_manipulator_msgs.srv import SetJointPosition
