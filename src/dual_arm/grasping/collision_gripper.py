import pybullet as p
import pybullet_data

from scipy.spatial.transform import Rotation


class CollisionGripper:

    def __init__(self):

        hand_collision = p.createCollisionShape(
            p.GEOM_MESH,
            fileName=(
                pybullet_data.getDataPath()
                + "/franka_panda/meshes/collision/hand.obj"
            )
        )

        finger_collision = p.createCollisionShape(
            p.GEOM_MESH,
            fileName=(
                pybullet_data.getDataPath()
                + "/franka_panda/meshes/collision/finger.obj"
            )
        )

        self.body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=hand_collision,

            linkMasses=[0, 0],

            linkCollisionShapeIndices=[
                finger_collision,
                finger_collision
            ],

            linkVisualShapeIndices=[
                -1,
                -1
            ],

            linkPositions=[
                [0, 0.04, 0.0584],
                [0, -0.04, 0.0584]
            ],

            linkOrientations=[
                [0, 0, 0, 1],
                [0, 0, 1, 0]
            ],

            linkInertialFramePositions=[
                [0, 0, 0],
                [0, 0, 0]
            ],

            linkInertialFrameOrientations=[
                [0, 0, 0, 1],
                [0, 0, 0, 1]
            ],

            linkParentIndices=[
                0,
                0
            ],

            linkJointTypes=[
                p.JOINT_FIXED,
                p.JOINT_FIXED
            ],

            linkJointAxis=[
                [0, 0, 1],
                [0, 0, 1]
            ]
            
        )
        
        p.changeVisualShape(
            self.body_id,
            -1,
            rgbaColor=[0, 0, 0, 0]
        )

        for link_idx in range(
            p.getNumJoints(self.body_id)
        ):
            p.changeVisualShape(
                self.body_id,
                link_idx,
                rgbaColor=[0, 0, 0, 0]
            )

    def set_transform(
        self,
        T_world_hand
    ):

        position = T_world_hand[:3, 3]

        quaternion = Rotation.from_matrix(
            T_world_hand[:3, :3]
        ).as_quat()

        p.resetBasePositionAndOrientation(
            self.body_id,
            position,
            quaternion
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
    
    def hide(self):

        self.set_pose(
            [100, 100, 100],
            [0, 0, 0, 1]
        )

    def show(self, T_world_hand):
        self.set_transform(T_world_hand)