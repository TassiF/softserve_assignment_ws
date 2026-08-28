# Pick-and-place planning and control

This package runs a collision-aware motion planner and controller using MoveIt and the Isaac Sim simulation environment with ROS2 bridges:

1. Isaac spawns the integrated UR10e manipulator + Robotiq 2F-140 gripper usd, a dynamic cube at a random pose, three open-top bins and a fixed obstacle. (The framework is generic for any manipulator model of Universal Robots, however currently using UR10e since it is integrated with the Robotiq gripper. A differen model might require attaching the gripper urdf separately).

2. Isaac publishes `/isaac_joint_states`, `/clock`, the measured object pose, object generation, and grasp state.

3. `isaac_joint_trajectory_controller` exposes MoveIt `FollowJointTrajectory` action, interpolates trajectories in sim time, and sends the position targets to Isaac.

4. The C++ planner mirrors the ground, bins, obstacle, object, and a conservative 2F-140 collision proxy into the MoveIt planning scene.

5. For each cycle, the planner generates a trajectory to first approach the object and then grasps itl. It waits for Isaac's attachment confirmation, chooses one of the three bins randomly, plans around the obstacle while carrying the object and
   places it into the selected bin.


## Motion Planner

The motion is planned by random_pick_place_planner.cpp.

The complete pick and place of the box is considered a full cycle.

For each phase of the cycle (pick-up approach, grasp, release approach, release), in case of potential failures or negative success of the trajectory planner, this retries for `planning_retries` iterations.
If still not successful after `planning_retries` iterations, `recover_from_failure()` is triggered and the robot goes to the homing position.
From there, it tries again to complete the same phase for other `planning_retries` iterations.
Ultimately, if this is again not feasible,  the object is respawned, the robot is homed and the gripper is reset. The full cycle is restarted.  
All of the above is repeated for `cycle_retries` iterations, after which, if no satisfactory solution is obtained, the task is interrupted.

### Additional parameters in `pick_place.yaml`

If `prefer_current_state_ = false`, it calls:
```bash
setPoseTarget(...)
```
MoveIt keeps the target as a Cartesian pose and lets the planner choose a valid joint configuration.

Instead, if `prefer_current_state_ = true`, it calls:
```bash
setJointValueTarget(...)
```
MoveIt computes IK using the current state and creates a joint-space target.
This uses the current robot state as the IK seed and solves the target pose from that state, generating the goal joint configuration.
The trajectory is generated from the current state to the generated joint configuration goal.
This prioritizes a nearby configuration during IK selection, but it is not a strict joint-distance optimization. The selected IK solution is close-biased, however OMPL may still generate a non-shortest path to reach it.

#### Future additions
This is a UR10e robot with 6DoF, however, if a redundant robot is to be used, it is essential to enforce a nullspace task to ensure that the best IK nullspace solution is chosen (among infinite solutions).
Hence, for stronger preference, I would add a joint-space nullspace target and enforce a
**minimum distance task:**\
$$ d \left( q_{target},q_{measured} \right) $$
 
In particular, with redundant robots with more than one degree of redundancy (DoF>task-space degrees of freedom (which is typically 6 or less)), it becomes highly important to constraint these nullspaces in order to obtain a solution that converges towards the global optimum. To do this, an optimal control scheme is needed, and even better a hierarchical optimal control scheme.

I am specialized in these type of controllers, for which you can check some of my works:
- [1] F. Tassi and A. Ajoudani, "Decoupled Multi-Robot Control for Coupled Bimanual Manipulation," 2026 IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM), Genova, Italy, 2026, pp. 1-8, doi: [10.1109/AIM65483.2026.11658072](https://ieeexplore.ieee.org/document/11658072)
- [2] Tassi F, Zhao J, Lahr GJ, et al. "IMA-catcher: An IMpact-aware nonprehensile catching framework based on combined optimization and learning". The International Journal of Robotics Research. 2026;45(1):100-127. doi: [10.1177/02783649251345851](https://journals.sagepub.com/doi/abs/10.1177/02783649251345851)
- [3] Tassi, F., "Hierarchical control for optimal human-robot collaboration", 2022. [hdl.handle.net/10589/207581](https://hdl.handle.net/10589/207581)
  

### Observations and Conclusions
moveit sucks



## Motion Controller


## Build and run

Run inside the Isaac Lab ROS 2 container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

reset; ros2 launch manipulator_sim ur_isaac_spawn.launch.py randomize_orientation:=true
```

For a single reproducible headless cycle:

```bash
reset; ros2 launch manipulator_sim ur_isaac_spawn.launch.py use_rviz:=true random_seed:=42 max_cycles:=1 randomize_orientation:=true
```

Please consider that, depending on your hardware, the proper startup of Isaac simulator might take a while.
However, after the first run, the caching volumes defined in the docker-compose file enable a faster startup.

The Robotiq assembly requires `ur_type:=ur10e`, which is the launch default. Isaac Sim 5.1 provides this gripper as the `Robotiq_2f_140` variant of its UR10e
asset. The ROS URDF remains the official six-axis UR model; MoveIt represents the attached gripper with conservative collision primitives configured in `config/pick_place.yaml`.

## Configuration and interfaces

`config/pick_place.yaml` is shared by the C++ planner and Isaac scene. It holds all the parameters for the particular task,
such as: object spawn region, three bin physical properties, obstacle geometry, grasp transform, gripper positions, 
collision proxy, timeouts, and motion scaling. `config/moveit_controllers.yaml` maps MoveIt execution to the custom
trajectory action.

Scene coordination topics:

- `/pick_place/object_pose` (`geometry_msgs/msg/PoseStamped`): object pose measured in Isaac.
- `/pick_place/object_generation` (`std_msgs/msg/UInt32`): increments of random object respawn.
- `/pick_place/reset_object` (`std_msgs/msg/Empty`): to request a new random object position.
- `/pick_place/gripper_close` (`std_msgs/msg/Bool`): actuates the Robotiq 2F-140 gripper.
- `/pick_place/grasp_attached` (`std_msgs/msg/Bool`): confirms physical pickup or release.
- `/pick_place/selected_bin` (`std_msgs/msg/Int32`): zero-based chosen bin.
