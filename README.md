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
    


## what all has been done:
- the environment has been setup. Ghost gripper and colliding gripper has been setup.
- press j for previous grasp, k for next grasp and l for random grasp in pybullet
- fixed collision checker.
- added when pressed next, it will show valid next grasp, same for previous, random  can show anything.
- grasp index assignment to individual arm done.
- IK solving is done and arms perfectly reach the grasp

## current bugs / todo:
- ~~i suspect that the ghost and collider is not synced up, because i can see that the gripper does not collide but in console it rejects it.~~
- ~~solve IK and make sure that the joint limits are not reaching extremums.~~
- m~~ove to a pregrasp position and then attempt to have the arms reachout.~~
- for some reason, the EE pose is behind the ghost gripper visualisation, fix this.

## further developments:

- ~~next, to assign which grasp for which arm ~~
- ~~solve IK~~
- ~~make sure that the arm reaches the grasp effectively~~
- establish that the arms can hold the object.
- introduce coefficient of friction for the object and the gripper.