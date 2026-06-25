import pybullet as p


class CollisionChecker:

    def __init__(
        self,
        gripper_id,
        object_id
    ):
        self.gripper_id = gripper_id
        self.object_id = object_id

    def object_collision(
        self,
        distance=0.0
    ):

        contacts = p.getClosestPoints(
            self.gripper_id,
            self.object_id,
            distance
        )

        return len(contacts) > 0

    def minimum_distance(self):

        contacts = p.getClosestPoints(
            self.gripper_id,
            self.object_id,
            10.0
        )

        if len(contacts) == 0:
            return None

        return min(
            c[8]
            for c in contacts
        )
    def is_penetrating(self):

        contacts = p.getClosestPoints(
            self.gripper_id,
            self.object_id,
            0.05
        )

        for c in contacts:
            if c[8] < 0:
                return True

        return False