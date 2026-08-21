#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import re

import numpy as np


headless_from_environment = os.environ.get("UR_HEADLESS", "").strip().lower() in {
    "1",
    "true",
    "native",
    "webrtc",
}

parser = argparse.ArgumentParser()
parser.add_argument(
    "--headless",
    action="store_true",
    default=headless_from_environment,
)
args, _ = parser.parse_known_args()


# This must be created before importing omni.* or other isaacsim modules.
from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "headless": args.headless,
    }
)


# Isaac Sim imports must occur after SimulationApp creation.
import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils import extensions, viewports
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics


extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()


ROBOT_TYPE = os.environ.get("UR_ROBOT_TYPE", "ur30").strip().lower()
if not re.fullmatch(r"[a-z0-9_]+", ROBOT_TYPE):
    raise RuntimeError(f"Invalid UR_ROBOT_TYPE: {ROBOT_TYPE!r}")

ROBOT_PRIM_PATH = f"/World/{ROBOT_TYPE.upper()}"

INPUT_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def read_initial_joint_positions() -> np.ndarray:
    raw_value = os.environ.get(
        "UR_INITIAL_JOINTS",
        os.environ.get(
            "UR30_INITIAL_JOINTS",
            "0.0,-1.5708,1.5708,-1.5708,-1.5708,0.0",
        ),
    )

    try:
        values = np.asarray(
            [
                float(component.strip())
                for component in raw_value.split(",")
            ],
            dtype=np.float32,
        )
    except ValueError as exception:
        raise RuntimeError(
            "UR_INITIAL_JOINTS must contain six numeric, "
            "comma-separated values."
        ) from exception

    if values.shape != (6,):
        raise RuntimeError(
            "UR_INITIAL_JOINTS must contain exactly six values. "
            f"Received: {raw_value!r}"
        )

    if not np.all(np.isfinite(values)):
        raise RuntimeError(
            "UR_INITIAL_JOINTS contains NaN or infinity."
        )

    return values


def find_articulation_root_path() -> str:
    """Return the descendant prim carrying ArticulationRootAPI."""
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")

    matching_paths = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if (
            str(prim.GetPath()) == ROBOT_PRIM_PATH
            or str(prim.GetPath()).startswith(ROBOT_PRIM_PATH + "/")
        )
        and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(matching_paths) != 1:
        raise RuntimeError(
            "Expected exactly one articulation root below "
            f"{ROBOT_PRIM_PATH}, found {matching_paths}."
        )
    return matching_paths[0]


def create_ros_action_graph(articulation_root_path: str) -> None:
    """Bridge Isaac articulation state and commands through ROS 2."""
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                (
                    "ArticulationController.inputs:robotPath",
                    articulation_root_path,
                ),
                ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
                (
                    "PublishJointState.inputs:targetPrim",
                    [usdrt.Sdf.Path(articulation_root_path)],
                ),
            ],
        },
    )


try:
    input_positions = read_initial_joint_positions()

    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError(
            "Isaac Sim asset root could not be resolved."
        )

    robot_usd_path = (
        assets_root
        + f"/Isaac/Robots/UniversalRobots/{ROBOT_TYPE}/{ROBOT_TYPE}.usd"
    )

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    viewports.set_camera_view(
        eye=np.array([2.8, 2.8, 2.2]),
        target=np.array([0.0, 0.0, 0.7]),
    )

    add_reference_to_stage(
        usd_path=robot_usd_path,
        prim_path=ROBOT_PRIM_PATH,
    )

    robot = world.scene.add(
        SingleArticulation(
            prim_path=ROBOT_PRIM_PATH,
            name="ur_robot",
        )
    )

    simulation_app.update()
    articulation_root_path = find_articulation_root_path()
    print("USD articulation root:", articulation_root_path, flush=True)
    create_ros_action_graph(articulation_root_path)
    simulation_app.update()

    # First reset initializes the articulation physics handles.
    world.reset()

    print(f"{ROBOT_TYPE} DOF names:", robot.dof_names, flush=True)
    print(f"{ROBOT_TYPE} number of DOFs:", robot.num_dof, flush=True)

    if robot.num_dof != 6:
        raise RuntimeError(
            f"Expected six {ROBOT_TYPE} DOFs, found {robot.num_dof}. "
            f"DOFs: {robot.dof_names}"
        )

    # Convert the launch-file ordering to the actual USD DOF ordering.
    positions_by_name = dict(
        zip(INPUT_JOINT_NAMES, input_positions)
    )

    missing_names = [
        joint_name
        for joint_name in INPUT_JOINT_NAMES
        if joint_name not in robot.dof_names
    ]

    if missing_names:
        raise RuntimeError(
            "The following joint names were not found in the "
            f"{ROBOT_TYPE} USD: {missing_names}\n"
            f"Available DOFs: {robot.dof_names}"
        )

    initial_positions = np.asarray(
        [
            positions_by_name[joint_name]
            for joint_name in robot.dof_names
        ],
        dtype=np.float32,
    )

    # In Isaac Sim 5.1, joint defaults are configured separately
    # from the articulation root pose.
    robot.set_joints_default_state(
        positions=initial_positions,
        velocities=np.zeros(
            shape=(robot.num_dof,),
            dtype=np.float32,
        ),
    )

    # The second reset applies set_joints_default_state().
    world.reset()

    # Teleport to the exact state immediately.
    robot.set_joint_positions(initial_positions)

    # Set the position-drive targets so the robot holds the pose.
    robot.apply_action(
        ArticulationAction(
            joint_positions=initial_positions,
        )
    )

    print(
        f"{ROBOT_TYPE} initialized successfully with ROS 2 joint bridge",
        flush=True,
    )
    print("USD DOF order:", robot.dof_names, flush=True)
    print("Initial positions:", initial_positions.tolist(), flush=True)
    print("Publishing: /isaac_joint_states and /clock", flush=True)
    print("Subscribing: /isaac_joint_commands", flush=True)

    while simulation_app.is_running():
        # render=True is required even for a headless SimulationApp: in Isaac
        # Sim 5.1, render=False advances PhysX directly but does not call the
        # application update that evaluates this OmniGraph.  NVIDIA's bundled
        # standalone ROS/MoveIt example uses the same stepping mode.
        world.step(render=True)
        # Tick state publishing, command subscription, actuation, and clock once
        # per physics frame, matching NVIDIA's standalone MoveIt bridge sample.
        og.Controller.set(
            og.Controller.attribute(
                "/ActionGraph/OnImpulseEvent.state:enableImpulse"
            ),
            True,
        )

except KeyboardInterrupt:
    pass

finally:
    simulation_app.close()
