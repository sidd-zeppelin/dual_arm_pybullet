import pybullet as p
import pybullet_data
import numpy as np

from scipy.spatial.transform import Rotation


class GhostGripper:

    def __init__(self):

        hand_visual = p.createVisualShape(
            p.GEOM_MESH,
            fileName=(
                pybullet_data.getDataPath()
                + "/franka_panda/meshes/visual/hand.obj"
            )
        )

        finger_visual = p.createVisualShape(
            p.GEOM_MESH,
            fileName=(
                pybullet_data.getDataPath()
                + "/franka_panda/meshes/visual/finger.obj"
            )
        )

        self.body_id = p.createMultiBody(
            baseMass=0,
            # baseCollisionShapeIndex=hand_collision,
            baseVisualShapeIndex=hand_visual,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],

            linkMasses=[
                0,
                0,
            ],

            linkCollisionShapeIndices=[
                -1,
                -1
            ],
            
            linkVisualShapeIndices=[
                finger_visual,
                finger_visual,
            ],

            linkPositions=[
                [0,  0.04, 0.0584],
                [0, -0.04, 0.0584],
            ],

            linkOrientations=[
                [0, 0, 0, 1],
                [0, 0, 1, 0],
            ],

            linkInertialFramePositions=[
                [0, 0, 0],
                [0, 0, 0],
            ],

            linkInertialFrameOrientations=[
                [0, 0, 0, 1],
                [0, 0, 0, 1],
            ],

            linkParentIndices=[
                0,
                0,
            ],

            linkJointTypes=[
                p.JOINT_FIXED,
                p.JOINT_FIXED,
            ],

            linkJointAxis=[
                [0, 0, 1],
                [0, 0, 1],
            ]
        )
        
        self.grasp_point_offset = np.array(
            [0.0, 0.0, 0.0584]
        )

    def set_pose(
        self,
        position,
        quaternion
    ):

        p.resetBasePositionAndOrientation(
            self.body_id,
            position,
            quaternion
        )

    def set_transform(
        self,
        T_world_hand
    ):

        position = T_world_hand[:3, 3]

        quaternion = Rotation.from_matrix(
            T_world_hand[:3, :3]
        ).as_quat()

        self.set_pose(
            position,
            quaternion
        )


    def draw_frame(
        self,
        length=0.1
    ):

        pos, quat = p.getBasePositionAndOrientation(
            self.body_id
        )

        pos = np.array(pos)

        R = Rotation.from_quat(
            quat
        ).as_matrix()

        p.addUserDebugLine(
            pos,
            pos + length * R[:, 0],
            [1, 0, 0],
            3
        )

        p.addUserDebugLine(
            pos,
            pos + length * R[:, 1],
            [0, 1, 0],
            3
        )

        p.addUserDebugLine(
            pos,
            pos + length * R[:, 2],
            [0, 0, 1],
            3
        )
    
    def hide(self):

        self.set_pose(
            [100, 100, 100],
            [0, 0, 0, 1]
        )


    def show(self, T_world_hand):
        self.set_transform(T_world_hand)
        
    def get_transform(self):

        pos, quat = p.getBasePositionAndOrientation(
            self.body_id
        )

        R = Rotation.from_quat(
            quat
        ).as_matrix()

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = pos

        return T