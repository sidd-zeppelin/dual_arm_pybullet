import pybullet as p
import numpy as np
from dual_arm.utils.transform import pose_to_matrix


class PandaIK:

    EE_LINK = 11

    def __init__(self, robot_id):

            self.robot_id = robot_id

            self.lower_limits = []
            self.upper_limits = []
            self.joint_ranges = []
            self.rest_poses = [
                                0.0,
                                -0.785,
                                0.0,
                                -2.356,
                                0.0,
                                1.571,
                                0.785
                            ]

            for j in range(7):

                info = p.getJointInfo(robot_id, j)

                lower = info[8]
                upper = info[9]

                self.lower_limits.append(lower)
                self.upper_limits.append(upper)
                self.joint_ranges.append(upper - lower)

    def solve(
        self,
        position,
        quaternion
    ):

        joint_values = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.EE_LINK,
            targetPosition=position,
            targetOrientation=quaternion,
            lowerLimits=self.lower_limits,
            upperLimits=self.upper_limits,
            jointRanges=self.joint_ranges,
            restPoses=self.rest_poses,
            maxNumIterations=300,
            residualThreshold=1e-6
        )

        return np.array(joint_values)
    
    def set_configuration(
        self,
        joint_values
    ):

        num_joints = min(
            7,
            len(joint_values)
        )

        for joint in range(num_joints):

            p.resetJointState(
                self.robot_id,
                joint,
                joint_values[joint]
            )
            
    def open_gripper(self):

        p.resetJointState(
            self.robot_id,
            9,
            0.04
        )

        p.resetJointState(
            self.robot_id,
            10,
            0.04
        )
    def close_gripper(self):

        p.resetJointState(
            self.robot_id,
            9,
            0.0
        )

        p.resetJointState(
            self.robot_id,
            10,
            0.0
        )
    
        
    def get_ee_transform(self):

        link_state = p.getLinkState(
            self.robot_id,
            self.EE_LINK,
            computeForwardKinematics=True
        )

        return pose_to_matrix(
            link_state[4],
            link_state[5]
        )