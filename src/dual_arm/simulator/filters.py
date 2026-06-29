import pybullet as p


def setup_self_collision_filters(robot_id):
    """
    Sets up self-collision filters for a specific robot.
    While p.URDF_USE_SELF_COLLISION enables collisions, it can cause the robot
    to violently explode if adjacent (or near-adjacent) links overlap.
    This function explicitly disables collisions between parent-child, 
    grandparent-child, and great-grandparent-child links to stabilize the arm.
    """
    num_joints = p.getNumJoints(robot_id)

    # Disable collision between specific close links
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        parent_index = info[16]

        if parent_index != -1:
            # 1. Disable Parent-Child collision
            p.setCollisionFilterPair(robot_id, robot_id, i, parent_index, 0)

            # 2. Disable Grandparent-Child collision
            parent_info = p.getJointInfo(robot_id, parent_index)
            grandparent_index = parent_info[16]
            if grandparent_index != -1:
                p.setCollisionFilterPair(robot_id, robot_id, i, grandparent_index, 0)

                # 3. Disable Great-Grandparent-Child collision (especially useful for Panda wrists)
                gp_info = p.getJointInfo(robot_id, grandparent_index)
                ggp_index = gp_info[16]
                if ggp_index != -1:
                    p.setCollisionFilterPair(robot_id, robot_id, i, ggp_index, 0)
                    
    # The Panda hand (link 8) often overlaps with earlier links in tight configurations, 
    # the depth-3 filter above handles most of it, keeping the rest of the arm safely self-colliding.
