# Dual Arm Pybullet Simulation Environment for Dual Arm Grasping

## File Structure:
```
dual_arm_pybullet/

    - assets/
        - grasps/ (contains decoupled grasp files)
        - meshes/ (contains .obj files for the meshes)
        - urdf/ 

    - configs/
        -scene.yaml (settings for the scene in simulator)

    - src/
        - dual_arm/

            - dataset/
                - dg16m.py  (contains function to load grasps)
            
            -grasping/
                - arm_assignment.py (Class and helpers for arm assignment)
                - collision_checker.py (Class for generating a collision checker)
                - collision_gripper.py (class for generating an invisible gripper for collision checks)
                - visualizer.py (grasp visualisation helpers)
                - ik.py (Class for IK Solver)

            - simulator/
                - objects.py (function to load the object in the simulator)
                - robots.py  (function to load the robot in the simulator)
                - world.py   (class for the world to load in the simulator)
                - gripper.py (for visualising the ghost gripper)
                - highlight.py (helper for highlighting an object)

            - utils/
                - config.py     (connects the settings from config.yaml to the code)
                - transform.py (contains helps for pose to matrix and viceversa trasnformations)
                - visualisation.py (adds axes to the given frame.)

        - main.py
    
```

## How to run:
- clone the repo first.
- cd `dual_arm_pybullet`
- run `uv venv`
- run `uv sync`
- run `cd src`
- run `python main.py`
- The Pybullet window appears. 
    ```
    press k for next acceptable grasp.
    press j for prev acceptable grasp.
    press l for random grasp (acceptable and rejected).
    press a to assign the grasp to the arm highlighted.

    ```


## Pipeline Overview

1. Load the simulation configuration from `scene.yaml`.

2. Create the PyBullet world and initialize the simulation.

3. Load the left and right Panda robots at their respective base positions.

4. Initialize inverse kinematics solvers and open both robot grippers.

5. Spawn the target object into the scene.

6. Create a ghost gripper for visualization and a collision gripper for collision checking.

7. Initialize the collision checker between the collision gripper and the object.

8. Load all candidate grasps from the DG16M grasp dataset.

9. Create the grasp-to-hand transform that aligns the DG16M grasp frame with the Panda hand frame.

10. Initialize the grasp visualizer using the dataset, object, and gripper models.

11. Initialize the arm assignment manager.

12. Highlight the left robot as the active robot for grasp assignment.

13. Display the initial grasp selected in the configuration file.

14. Evaluate the displayed grasp for object penetration and report whether it is valid.

15. Enter the simulation loop and continuously advance the physics simulation.

16. Listen for keyboard inputs to browse candidate grasps.

    * **L** : Show a random grasp.
    * **K** : Show the next collision-free grasp.
    * **J** : Show the previous collision-free grasp.

17. Press **A** to assign the currently displayed grasp to the active robot.

18. After the first assignment, switch the active robot from the left arm to the right arm.

19. After both grasps have been assigned, hide the visualization grippers and begin inverse kinematics.

20. Extract the desired hand pose for each robot from the stored assignment.

21. Convert each desired hand orientation from a rotation matrix to a quaternion.

22. Solve inverse kinematics for both Panda robots to obtain joint configurations.

23. Apply the computed joint configurations to both robots, moving them to their assigned grasp poses.

24. Stop the assignment pipeline after both robots reach their target configurations.



## what all has been done:
- the environment has been setup. Ghost gripper and colliding gripper has been setup.
- press j for previous grasp, k for next grasp and l for random grasp in pybullet
- fixed collision checker.
- added when pressed next, it will show valid next grasp, same for previous, random  can show anything.
- grasp index assignment to individual arm done.
- IK solving is done and arms perfectly reach the grasp
- The offset that was between ghost gripper and end effector, ive fixed it by hardcoding the translation.

## current bugs / todo:
- ~~i suspect that the ghost and collider is not synced up, because i can see that the gripper does not collide but in console it rejects it.~~
- ~~solve IK and make sure that the joint limits are not reaching extremums.~~
- m~~ove to a pregrasp position and then attempt to have the arms reachout.~~
- ~~for some reason, the EE pose is behind the ghost gripper visualisation, fix this.~~

## further developments:

- ~~next, to assign which grasp for which arm ~~
- ~~solve IK~~
- ~~make sure that the arm reaches the grasp effectively~~
- establish that the arms can hold the object.
- introduce coefficient of friction for the object and the gripper.