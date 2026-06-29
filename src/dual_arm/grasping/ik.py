
# pyrefly: ignore [missing-import]
import pybullet as p
import numpy as np
from dual_arm.utils.transform import pose_to_matrix


class PandaIK:

    EE_LINK = 11

    def __init__(self, robot_id, config):

            self.robot_id = robot_id

            self.lower_limits = []
            self.upper_limits = []
            self.joint_ranges = []
            self.rest_poses = []
            custom_lower = [config["joint1"]["limit"]["lower"],
                            config["joint2"]["limit"]["lower"],
                            config["joint3"]["limit"]["lower"],
                            config["joint4"]["limit"]["lower"],
                            config["joint5"]["limit"]["lower"],
                            config["joint6"]["limit"]["lower"],
                            config["joint7"]["limit"]["lower"]]
                            
            custom_upper = [config["joint1"]["limit"]["upper"],
                            config["joint2"]["limit"]["upper"],
                            config["joint3"]["limit"]["upper"],
                            config["joint4"]["limit"]["upper"],
                            config["joint5"]["limit"]["upper"],
                            config["joint6"]["limit"]["upper"],
                            config["joint7"]["limit"]["upper"]]
                            
            custom_rest = [0.0, -1.5, 0.0, -2.8, 0.0, 1.571, 0.785]
            
            base_pos, _ = p.getBasePositionAndOrientation(robot_id)
            if base_pos[0] < 0:
                custom_rest = [-x if i in [0, 2, 4, 6] else x for i, x in enumerate(custom_rest)]
            
            num_joints = p.getNumJoints(robot_id)
            dof_idx = 0
            
            for j in range(num_joints):
                info = p.getJointInfo(robot_id, j)
                if info[2] != p.JOINT_FIXED:
                    if dof_idx < 7:
                        self.lower_limits.append(custom_lower[dof_idx])
                        self.upper_limits.append(custom_upper[dof_idx])
                        self.joint_ranges.append(custom_upper[dof_idx] - custom_lower[dof_idx])
                        self.rest_poses.append(custom_rest[dof_idx])
                    else:
                        lower = info[8]
                        upper = info[9]
                        self.lower_limits.append(lower)
                        self.upper_limits.append(upper)
                        self.joint_ranges.append(upper - lower)
                        self.rest_poses.append(0.04) 
                    dof_idx += 1

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
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_values[joint],
                force=500
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

def compute_ik_targets(assignment, leftIK, rightIK):
    from scipy.spatial.transform import Rotation
    T_target = assignment.left_hand_T.copy()
    
    T_target[:3, 3] += (0.107 + 0.025) * T_target[:3, 2]

    left_position = T_target[:3, 3]
    left_rotation = T_target[:3, :3]
    left_quaternion = Rotation.from_matrix(left_rotation).as_quat()

    left_q = leftIK.solve(left_position, left_quaternion)

    T_target = assignment.right_hand_T.copy()
    
    T_target[:3, 3] += (0.107 + 0.025) * T_target[:3, 2]

    right_position = T_target[:3, 3]
    right_rotation = T_target[:3, :3]
    right_quaternion = Rotation.from_matrix(right_rotation).as_quat()

    right_q = rightIK.solve(right_position, right_quaternion)

    return left_q, right_q