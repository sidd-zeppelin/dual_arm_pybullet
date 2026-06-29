# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased] - Initial Development

### Added
- Modularized the monolithic `main.py` into distinct functional modules: `setup.py`, `objects.py`, `animation.py`, and `ik.py` for production readiness.
- Implemented a discrete state machine (`init`, `assignment`, `wait_for_ik`, `done`) to clearly sequence the interactive workflow.
- Bound the `Y` key to act as a universal confirmation/transition button between phases.
- Bound the `R` key to cleanly restart the PyBullet simulation environment on the fly.
- Replaced instantaneous IK "teleporting" with a smooth, 1-second S-curve motor interpolation loop (`animation.py`) for physically realistic arm execution.
- Added visual media (Setup Image, Working Demo Video) directly to the documentation.
- Finalized a compact, symmetrical humanoid torso configuration with both arm bases mounted closely together (`X = +/- 0.1`, `Z = 1.5`, `Y = -0.6`) facing outward.
- Perfectly mirrored base quaternions in `scene.yaml` to achieve symmetrical kinematic reachability for the left and right arms, preventing awkward elbow configurations.
- Implemented a custom "curled up" resting posture for the robots to keep them naturally out of collision bounds when idle.
- Added a 2.5cm "plunge" offset to the final Inverse Kinematics targets in `main.py`. This allows the arm to aggressively reach deeper into the object for the final grasp, while keeping the interactive ghost gripper preview completely untouched.
- Implemented robust self-collision filters for both arms to prevent the arms from physics-clipping into themselves.
- Added realistic physical environment: enabled gravity (`-9.81` on Z) and spawned a table (`table/table.urdf`).
- Spawned visual geometric primitives (a central pillar and shoulder beams) to represent the physical torso mount.
- Added a 2-second physics stabilization phase at startup. The object dynamically calculates its bounding box and spawns precisely 5cm above the table, naturally falling into place before the interactive grasp visualizer kicks in.
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
- Fixed Inverse Kinematics (IK) logic to target the correct end-effector link (`panda_grasptarget`, Link 11), ensuring the robot TCP flawlessly matches the dataset's grasp pose.
- Fixed Inverse Kinematics (IK) null-space array length mismatch (PyBullet silently ignored constraints due to arrays of size 14/7 instead of exactly 9 DOFs).
- Removed the 10.7cm pre-grasp backward offset from `main.py` so that the arms snap exactly to the object grasp position instead of stopping 10cm away.
- Fixed a physics engine bug where arms would instantaneously snap back to their vertical `rest_poses` (candle pose) after assignment. `p.setJointMotorControl2` is now used alongside `resetJointState` to actively hold the solved IK joint positions during `world.step()`.
- Fixed collision checker to accurately detect object-gripper penetration, showing only valid grasps when iterating next/previous.
- Synced the ghost gripper and collision collider behavior.
- Resets right arm highlight back to default white color once dual-arm assignment is complete.
- Hardcoded translation offset between ghost gripper and actual end-effector pose for perfect visual alignment.
- Resolved IK issues to ensure joints remain within defined limits and avoid extremums.
- Fixed an issue where the End-Effector (EE) pose was lagging behind the ghost gripper visualization.

### Changed
- Disabled the default PyBullet GUI sidebar and global world axis for a cleaner visualization and recording environment.
- Stripped all legacy inline comments across the core modules to prepare the codebase for a centralized technical design document.
- Arms now move to a predefined "pre-grasp" position first before reaching out to the actual grasp transform.
- Refined the interactive visualization logic for smoother UX during manual grasp exploration.
- Cleaned up development and prototyping scripts (`test_torso.py`, `test_gripper_offset.py`, etc.) from the workspace.
