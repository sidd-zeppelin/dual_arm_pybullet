import time
import pybullet as p
from scipy.spatial.transform import Rotation
import numpy as np

from dual_arm.utils.config import load_config
from dual_arm.simulator.world import World
from dual_arm.simulator.robots import load_robot
from dual_arm.simulator.objects import load_object
from dual_arm.simulator.gripper import GhostGripper
from dual_arm.dataset.dg16m import DG16MLoader
from dual_arm.utils.transform import (create_transform,
                                      pose_to_matrix,)
from dual_arm.grasping.visualizer import GraspVisualizer
from dual_arm.grasping.collision_checker import CollisionChecker
from dual_arm.grasping.collision_gripper import CollisionGripper
from dual_arm.grasping.arm_assignment import AssignmentManager
from dual_arm.simulator.highlight import highlight_robot
from dual_arm.grasping.ik import PandaIK
from dual_arm.utils.visualization import draw_frame

def main():

    config = load_config(
        "../configs/scene.yaml"
    )
    
    joint_limits = load_config(
        "../configs/joint_limits.yaml"
    )
    
    
    world = World(config)

    left_robot = load_robot(
        config["robots"]["left"]
    )

    right_robot = load_robot(
        config["robots"]["right"]
    )

    rightIK = PandaIK(right_robot,
                      joint_limits)
    leftIK = PandaIK(left_robot,
                     joint_limits)
    leftIK.open_gripper()
    rightIK.open_gripper()
              
    
    object_id = load_object(
        config["object"]
    )
    
    ghost = GhostGripper()
    collision_gripper = CollisionGripper()
    ghost_pos, ghost_quat = p.getBasePositionAndOrientation(ghost.body_id)
    p.resetBasePositionAndOrientation(collision_gripper.body_id, ghost_pos, ghost_quat)
        
    checker = CollisionChecker(
        collision_gripper.body_id,
        object_id
    )

    ghost.set_pose(
        [0, 0, 0.5],
        [0, 0, 0, 1]
    )
    
    loader = DG16MLoader(
        config["object"]["grasps"]
    )

    T_grasp_hand = create_transform(
        config["gripper"]["offset_pos"],
        config["gripper"]["offset_rpy_deg"]
    )

    visualizer = GraspVisualizer(
        loader,
        object_id,
        ghost,
        collision_gripper,
        T_grasp_hand
    )
    
    assignment = AssignmentManager()

    highlight_robot(
        left_robot,
        (0, 1, 0, 1)
    )
    
    highlight_robot(
        right_robot,
        (1, 1, 1, 1)
    )
    
    visualizer.show(
        config["object"]["grasp_idx"]
    )
    
    dist = checker.minimum_distance()
    if dist < 0:
        print(
            f"REJECT | penetration = {-dist:.4f}"
        )
    else:
        print(
            f"ACCEPT | clearance = {dist:.4f}"
        )
    
    while True:

        world.step()
        keys = p.getKeyboardEvents()
        
        if (
            ord('a') in keys
            and keys[ord('a')] & p.KEY_WAS_TRIGGERED
        ):

            assignment.accept(
                visualizer.current_idx,
                visualizer.get_current_grasp_transform(),
                visualizer.get_current_hand_transform()
            )
            
            if assignment.mode == "right":

                highlight_robot(
                    left_robot,
                    [1, 1, 1, 1]
                )

                highlight_robot(
                    right_robot,
                    [0, 1, 0, 1]
                )
                
                        
            if assignment.mode == "ik":
                print('=' * 80)
                print('Arm Assignment is done.\n')
                print(f'Left Grasp Index = {assignment.left_grasp_idx}\n')
                print(f'Right Grasp Index = {assignment.right_grasp_idx}')
                print('=' * 80)

                ghost.hide()
                collision_gripper.hide()

        if (
            ord('l') in keys
            and keys[ord('l')] & p.KEY_WAS_TRIGGERED
        ):
            visualizer.show_random()
            dist = checker.minimum_distance()

            if dist < 0:
                print(
                    f"REJECT | penetration = {-dist:.4f}"
                )
            else:
                print(
                    f"ACCEPT | clearance = {dist:.4f}"
                )

        if (
            ord('k') in keys
            and keys[ord('k')] & p.KEY_WAS_TRIGGERED
        ):
            visualizer.show_next_valid(checker)
            dist = checker.minimum_distance()
            if dist < 0:
                print(
                    f"REJECT | penetration = {-dist:.4f}"
                )
            else:
                print(
                    f"ACCEPT | clearance = {dist:.4f}"
                )
            
        if (
            ord('j') in keys
            and keys[ord('j')] & p.KEY_WAS_TRIGGERED
        ):
            visualizer.show_prev_valid(checker)
            dist = checker.minimum_distance()
            if dist < 0:
                print(
                    f"REJECT | penetration = {-dist:.4f}"
                )
            else:
                print(
                    f"ACCEPT | clearance = {dist:.4f}"
                )
                
        if(
            assignment.mode == 'ik'
        ):
      
            # ---------------- Left ----------------

            T_target = assignment.left_hand_T.copy()

            # move 2 cm along the hand's local Y-axis
            T_target[:3, 3] += 0.107 * T_target[:3, 2]

            left_position = T_target[:3, 3]
            left_rotation = T_target[:3, :3]

            left_quaternion = Rotation.from_matrix(
                left_rotation
            ).as_quat()

            left_q = leftIK.solve(
                left_position,
                left_quaternion
            )

            # ---------------- Right ----------------

            T_target = assignment.right_hand_T.copy()

            T_target[:3, 3] += 0.107 * T_target[:3, 2]

            right_position = T_target[:3, 3]
            right_rotation = T_target[:3, :3]

            right_quaternion = Rotation.from_matrix(
                right_rotation
            ).as_quat()

            right_q = rightIK.solve(
                right_position,
                right_quaternion
            )
            
            leftIK.set_configuration(left_q)
            rightIK.set_configuration(right_q)

            assignment.mode = "stop"
                    
                    
        time.sleep(
            config["simulation"]["timestep"]
        )
        
    # debug 
    # print(grasp.shape)
    # print(grasp)

if __name__ == "__main__":
    main()