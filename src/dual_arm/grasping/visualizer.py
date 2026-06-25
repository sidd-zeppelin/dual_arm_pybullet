import random

import pybullet as p

from dual_arm.utils.transform import pose_to_matrix
from dual_arm.utils.visualization import draw_frame


class GraspVisualizer:

    def __init__(
        self,
        loader,
        object_id,
        ghost,
        collision_gripper,
        T_grasp_hand,
    ):

        self.loader = loader
        self.object_id = object_id
        self.ghost = ghost
        self.collision_gripper = collision_gripper
        self.T_grasp_hand = T_grasp_hand

        self.num_grasps = 4000

        self.current_idx = 0

        self.current_T_world_grasp = None
        self.current_T_world_hand = None

        self.frame_ids = []

    def show(
        self,
        grasp_idx
    ):

        T_object_grasp = self.loader.get_grasp(
            grasp_idx
        )

        obj_pos, obj_quat = (
            p.getBasePositionAndOrientation(
                self.object_id
            )
        )

        T_world_object = pose_to_matrix(
            obj_pos,
            obj_quat
        )

        self.current_T_world_grasp = (
            T_world_object
            @
            T_object_grasp
        )

        self.current_T_world_hand = (
            self.current_T_world_grasp
            @
            self.T_grasp_hand
        )

        self.ghost.set_transform(
            self.current_T_world_hand
        )

        self.collision_gripper.set_transform(
            self.current_T_world_hand
        )

        self.clear_frame()

        # self.frame_ids = draw_frame(
        #     self.current_T_world_grasp,
        #     scale=0.05
        # )

        self.current_idx = grasp_idx

        print(
            f"Showing grasp {grasp_idx}"
        )

    def show_random(self):

        self.show(
            random.randint(
                0,
                self.num_grasps - 1
            )
        )

    def show_next(self):

        self.current_idx = (
            self.current_idx + 1
        ) % self.num_grasps

        self.show(
            self.current_idx
        )

    def show_previous(self):

        self.current_idx = (
            self.current_idx - 1
        ) % self.num_grasps

        self.show(
            self.current_idx
        )

    def _show_valid(
        self,
        checker,
        step
    ):

        for _ in range(self.num_grasps):

            self.current_idx = (
                self.current_idx + step
            ) % self.num_grasps

            self.show(
                self.current_idx
            )

            if not checker.is_penetrating():

                print(
                    f"Accepted grasp {self.current_idx}"
                )

                return self.current_idx

        print(
            "No valid grasp found."
        )

        return None

    def show_next_valid(
        self,
        checker
    ):

        return self._show_valid(
            checker,
            1
        )

    def show_prev_valid(
        self,
        checker
    ):

        return self._show_valid(
            checker,
            -1
        )

    def clear_frame(self):

        for frame_id in self.frame_ids:

            p.removeUserDebugItem(
                frame_id
            )

        self.frame_ids = []

    def get_current_grasp_transform(self):

        return self.current_T_world_grasp

    def get_current_hand_transform(self):

        return self.current_T_world_hand