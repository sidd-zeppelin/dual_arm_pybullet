# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased] - Initial Development

### Added
- Implemented robust self-collision filters for both arms to prevent the arms from physics-clipping into themselves.
- Setup complete dual-arm PyBullet simulation environment with left and right Franka Panda robots.
- Ghost gripper and collision gripper for visual and physical validation of grasps.
- Interactive PyBullet controls:
  - `J`: Previous valid grasp.
  - `K`: Next valid grasp.
  - `L`: Random grasp (valid or rejected).
  - `A`: Assign grasp to highlighted active arm.
- Grasp index assignment pipeline for individual arms (alternating left, then right).
- Full Inverse Kinematics (IK) solving for both arms to smoothly reach assigned grasps.
- Configurable simulation scene, object parameters, and grasp datasets via `scene.yaml`.

### Fixed
- Fixed Inverse Kinematics (IK) null-space array length mismatch (PyBullet silently ignored constraints due to arrays of size 14/7 instead of exactly 9 DOFs).
- Fixed the pre-grasp target offset direction by subtracting the Z-axis vector (`-=`) instead of adding it, preventing the robot from colliding violently into the floor/object.
- Fixed collision checker to accurately detect object-gripper penetration, showing only valid grasps when iterating next/previous.
- Synced the ghost gripper and collision collider behavior.
- Resets right arm highlight back to default white color once dual-arm assignment is complete.
- Hardcoded translation offset between ghost gripper and actual end-effector pose for perfect visual alignment.
- Resolved IK issues to ensure joints remain within defined limits and avoid extremums.
- Fixed an issue where the End-Effector (EE) pose was lagging behind the ghost gripper visualization.

### Changed
- Arms now move to a predefined "pre-grasp" position first before reaching out to the actual grasp transform.
- Refined the interactive visualization logic for smoother UX during manual grasp exploration.
