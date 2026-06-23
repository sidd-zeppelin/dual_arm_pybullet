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

            - simulator/
                - objects.py (function to load the object in the simulator)
                - robots.py  (function to load the robot in the simulator)
                - world.py   (class for the world to load in the simulator)

            - utils/
                - config.py     (connects the settings from config.yaml to the code)

        - main.py
    
```

## what all has been done:
- the environment has been setup. Ghost gripper and colliding gripper has been setup.
- press j for previous grasp, k for next grasp and l for random grasp in pybullet
- fixed collision checker.
- added when pressed next, it will show valid next grasp, same for previous, random  can show anything.
- grasp index assignment to individual arm done.

## current bugs / todo:
- ~~i suspect that the ghost and collider is not synced up, because i can see that the gripper does not collide but in console it rejects it.~~
- solve IK and make sure that the joint limits are not reaching extremums.
- move to a pregrasp position and then attempt to have the arms reachout.

## further developments:

- ~~next, to assign which grasp for which arm ~~
- solve IK
- make sure that the arm reaches the grasp effectively
- establish that the arms can hold the object.
- introduce coefficient of friction for the object and the gripper.