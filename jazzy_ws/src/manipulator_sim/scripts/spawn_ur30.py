#!/usr/bin/env python3

"""Spawn a UR10e/Robotiq pick-and-place cell and bridge it to ROS 2."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
from typing import Any

import numpy as np
import yaml


headless_from_environment = os.environ.get('UR_HEADLESS', '').strip().lower() in {
    '1',
    'true',
    'native',
    'webrtc',
}

parser = argparse.ArgumentParser()
parser.add_argument(
    '--headless',
    action='store_true',
    default=headless_from_environment,
)
args, _ = parser.parse_known_args()


# SimulationApp must exist before importing omni.*, pxr, or Isaac Sim modules.
from isaacsim import SimulationApp  # noqa: E402, I100


simulation_app = SimulationApp({'headless': args.headless})


# Isaac Sim and ROS imports must occur after SimulationApp creation.
import omni.graph.core as og  # noqa: E402, I100
import omni.usd  # noqa: E402, I100
import rclpy  # noqa: E402, I100
import usdrt.Sdf  # noqa: E402, I100
from geometry_msgs.msg import PoseStamped  # noqa: E402, I100
from isaacsim.core.api import World  # noqa: E402, I100
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid  # noqa: E402, I100
from isaacsim.core.prims import SingleArticulation  # noqa: E402, I100
from isaacsim.core.utils import extensions, viewports  # noqa: E402, I100
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402, I100
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402, I100
from isaacsim.storage.native import get_assets_root_path  # noqa: E402, I100
from pxr import Gf, Sdf, Usd, UsdPhysics  # noqa: E402, I100
from rclpy.qos import (  # noqa: E402, I100
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Empty, UInt32  # noqa: E402, I100


extensions.enable_extension('isaacsim.ros2.bridge')
simulation_app.update()


ROBOT_TYPE = os.environ.get('UR_ROBOT_TYPE', 'ur10e').strip().lower()
if ROBOT_TYPE != 'ur10e':
    raise RuntimeError(
        "The Robotiq 2F-140 assembly uses Isaac Sim's integrated ur10e "
        f'asset; received UR_ROBOT_TYPE={ROBOT_TYPE!r}.'
    )

ROBOT_PRIM_PATH = '/World/UR10E'
OBJECT_PRIM_PATH = '/World/PickObject'
ROBOT_VARIANT_SET = 'Gripper'
ROBOT_VARIANT = 'Robotiq_2f_140'
ARM_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
GRIPPER_JOINT_NAMES = [
    'finger_joint',
    'right_outer_knuckle_joint',
    'left_outer_finger_joint',
    'right_outer_finger_joint',
    'left_inner_finger_joint',
    'right_inner_finger_joint',
    'left_inner_finger_pad_joint',
    'right_inner_finger_pad_joint',
]
GRIPPER_DRIVE_JOINT_NAME = 'finger_joint'


def require_vector(
    parameters: dict[str, Any], name: str, length: int
) -> np.ndarray:
    value = np.asarray(parameters.get(name, []), dtype=np.float64)
    if value.shape != (length,) or not np.all(np.isfinite(value)):
        raise RuntimeError(
            f'Scene parameter {name!r} must contain {length} finite values.'
        )
    return value


def require_positive_vector(
    parameters: dict[str, Any], name: str, length: int
) -> np.ndarray:
    value = require_vector(parameters, name, length)
    if np.any(value <= 0.0):
        raise RuntimeError(f'Scene parameter {name!r} must be positive.')
    return value


def load_scene_parameters() -> dict[str, Any]:
    config_path = Path(
        os.environ.get(
            'UR_SCENE_CONFIG',
            Path(__file__).resolve().parents[1] / 'config' / 'pick_place.yaml',
        )
    )
    try:
        with config_path.open('r', encoding='utf-8') as config_file:
            document = yaml.safe_load(config_file)
        parameters = document['random_pick_place_planner']['ros__parameters']
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exception:
        raise RuntimeError(
            f'Could not load pick/place scene configuration from {config_path}.'
        ) from exception
    if not isinstance(parameters, dict):
        raise RuntimeError('The pick/place ROS parameter block must be a mapping.')
    return parameters


def read_initial_joint_positions() -> np.ndarray:
    raw_value = os.environ.get(
        'UR_INITIAL_JOINTS',
        '-1.5708,-1.5708,1.5708,-1.5708,-1.5708,0.0',
    )
    try:
        values = np.asarray(
            [float(component.strip()) for component in raw_value.split(',')],
            dtype=np.float32,
        )
    except ValueError as exception:
        raise RuntimeError(
            'UR_INITIAL_JOINTS must contain six numeric comma-separated values.'
        ) from exception
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise RuntimeError(
            'UR_INITIAL_JOINTS must contain exactly six finite values.'
        )
    return values


def find_unique_descendant(
    stage: Usd.Stage, root_path: str, predicate: Any, description: str
) -> str:
    matching_paths = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if (
            str(prim.GetPath()) == root_path
            or str(prim.GetPath()).startswith(root_path + '/')
        )
        and predicate(prim)
    ]
    if len(matching_paths) != 1:
        raise RuntimeError(
            f'Expected one {description} below {root_path}, found {matching_paths}.'
        )
    return matching_paths[0]


def create_ros_action_graph(articulation_root_path: str) -> None:
    """Bridge the combined articulation state and six-DOF arm commands."""
    og.Controller.edit(
        {'graph_path': '/ActionGraph', 'evaluator_name': 'execution'},
        {
            og.Controller.Keys.CREATE_NODES: [
                ('OnImpulseEvent', 'omni.graph.action.OnImpulseEvent'),
                ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
                (
                    'PublishJointState',
                    'isaacsim.ros2.bridge.ROS2PublishJointState',
                ),
                (
                    'SubscribeJointState',
                    'isaacsim.ros2.bridge.ROS2SubscribeJointState',
                ),
                (
                    'ArticulationController',
                    'isaacsim.core.nodes.IsaacArticulationController',
                ),
                ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
            ],
            og.Controller.Keys.CONNECT: [
                (
                    'OnImpulseEvent.outputs:execOut',
                    'PublishJointState.inputs:execIn',
                ),
                (
                    'OnImpulseEvent.outputs:execOut',
                    'SubscribeJointState.inputs:execIn',
                ),
                (
                    'OnImpulseEvent.outputs:execOut',
                    'ArticulationController.inputs:execIn',
                ),
                (
                    'OnImpulseEvent.outputs:execOut',
                    'PublishClock.inputs:execIn',
                ),
                ('Context.outputs:context', 'PublishJointState.inputs:context'),
                ('Context.outputs:context', 'SubscribeJointState.inputs:context'),
                ('Context.outputs:context', 'PublishClock.inputs:context'),
                (
                    'ReadSimTime.outputs:simulationTime',
                    'PublishJointState.inputs:timeStamp',
                ),
                (
                    'ReadSimTime.outputs:simulationTime',
                    'PublishClock.inputs:timeStamp',
                ),
                (
                    'SubscribeJointState.outputs:jointNames',
                    'ArticulationController.inputs:jointNames',
                ),
                (
                    'SubscribeJointState.outputs:positionCommand',
                    'ArticulationController.inputs:positionCommand',
                ),
                (
                    'SubscribeJointState.outputs:velocityCommand',
                    'ArticulationController.inputs:velocityCommand',
                ),
                (
                    'SubscribeJointState.outputs:effortCommand',
                    'ArticulationController.inputs:effortCommand',
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                (
                    'ArticulationController.inputs:robotPath',
                    articulation_root_path,
                ),
                ('PublishJointState.inputs:topicName', 'isaac_joint_states'),
                ('SubscribeJointState.inputs:topicName', 'isaac_joint_commands'),
                (
                    'PublishJointState.inputs:targetPrim',
                    [usdrt.Sdf.Path(articulation_root_path)],
                ),
            ],
        },
    )


def add_fixed_cuboid(
    world: World,
    name: str,
    position: np.ndarray,
    size: np.ndarray,
    color: np.ndarray,
) -> None:
    world.scene.add(
        FixedCuboid(
            prim_path=f'/World/{name}',
            name=name,
            position=position,
            scale=size,
            size=1.0,
            color=color,
        )
    )


def add_bin(
    world: World,
    index: int,
    center: np.ndarray,
    outer_size: np.ndarray,
    wall_thickness: float,
    floor_thickness: float,
    color: np.ndarray,
) -> None:
    x_size, y_size, height = outer_size
    x_center, y_center, base_z = center
    parts = [
        (
            'floor',
            np.array([x_center, y_center, base_z + floor_thickness / 2.0]),
            np.array([x_size, y_size, floor_thickness]),
        ),
        (
            'left',
            np.array(
                [
                    x_center - (x_size - wall_thickness) / 2.0,
                    y_center,
                    base_z + height / 2.0,
                ]
            ),
            np.array([wall_thickness, y_size, height]),
        ),
        (
            'right',
            np.array(
                [
                    x_center + (x_size - wall_thickness) / 2.0,
                    y_center,
                    base_z + height / 2.0,
                ]
            ),
            np.array([wall_thickness, y_size, height]),
        ),
        (
            'front',
            np.array(
                [
                    x_center,
                    y_center - (y_size - wall_thickness) / 2.0,
                    base_z + height / 2.0,
                ]
            ),
            np.array([x_size - 2.0 * wall_thickness, wall_thickness, height]),
        ),
        (
            'back',
            np.array(
                [
                    x_center,
                    y_center + (y_size - wall_thickness) / 2.0,
                    base_z + height / 2.0,
                ]
            ),
            np.array([x_size - 2.0 * wall_thickness, wall_thickness, height]),
        ),
    ]
    for part_name, position, size in parts:
        add_fixed_cuboid(
            world,
            f'Bin{index}_{part_name}',
            position,
            size,
            color,
        )


def quaternion_conjugate(quaternion: np.ndarray) -> np.ndarray:
    result = quaternion.copy()
    result[1:] *= -1.0
    return result


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )


def rotate_vector(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    vector_quaternion = np.array([0.0, *vector], dtype=np.float64)
    return quaternion_multiply(
        quaternion_multiply(quaternion, vector_quaternion),
        quaternion_conjugate(quaternion),
    )[1:]


def prim_world_pose(prim: Usd.Prim) -> tuple[np.ndarray, np.ndarray]:
    matrix = omni.usd.get_world_transform_matrix(prim)
    translation = matrix.ExtractTranslation()
    rotation = matrix.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    return (
        np.array(translation, dtype=np.float64),
        np.array(
            [rotation.GetReal(), imaginary[0], imaginary[1], imaginary[2]],
            dtype=np.float64,
        ),
    )


class SceneRosBridge:
    """Synchronize the physical object and Robotiq command with the planner."""

    def __init__(
        self,
        world: World,
        robot: SingleArticulation,
        pick_object: DynamicCuboid,
        wrist_prim: Usd.Prim,
        parameters: dict[str, Any],
    ) -> None:
        self.world = world
        self.robot = robot
        self.pick_object = pick_object
        self.wrist_prim = wrist_prim
        self.object_size = require_positive_vector(parameters, 'object_size', 3)
        self.spawn_min = require_vector(parameters, 'object_spawn_min', 3)
        self.spawn_max = require_vector(parameters, 'object_spawn_max', 3)
        self.object_to_tcp_offset = require_vector(
            parameters, 'object_to_tcp_offset', 3
        )
        self.tcp_offset_from_wrist = require_vector(
            parameters, 'tcp_offset_from_wrist', 3
        )
        self.expected_grasp_distance = float(
            np.linalg.norm(self.object_to_tcp_offset)
        )
        if np.any(self.spawn_min[:2] >= self.spawn_max[:2]):
            raise RuntimeError('object_spawn_min must precede object_spawn_max in x/y.')
        self.gripper_open_position = float(
            parameters.get('gripper_open_position', 0.0)
        )
        self.gripper_closed_position = float(
            parameters.get('gripper_closed_position', 0.65)
        )
        self.max_grasp_distance = float(
            parameters.get('max_grasp_distance', 0.10)
        )
        if (
            not math.isfinite(self.gripper_open_position)
            or not math.isfinite(self.gripper_closed_position)
            or self.gripper_closed_position <= self.gripper_open_position
            or self.max_grasp_distance <= 0.0
        ):
            raise RuntimeError('Invalid gripper positions or max_grasp_distance.')

        seed = int(os.environ.get('UR_RANDOM_SEED', '-1'))
        self.random_generator = np.random.default_rng(None if seed < 0 else seed)
        self.gripper_indices = np.asarray(
            [self.robot.dof_names.index(GRIPPER_DRIVE_JOINT_NAME)],
            dtype=np.int32,
        )
        self.requested_close = False
        self.previous_requested_close = False
        self.reset_pending = True
        self.object_attached = False
        self.object_generation = 0
        self.relative_position = np.zeros(3, dtype=np.float64)
        self.relative_orientation = np.array(
            [1.0, 0.0, 0.0, 0.0], dtype=np.float64
        )
        self.frames_since_publish = 0

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('isaac_pick_place_scene')
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.object_pose_publisher = self.node.create_publisher(
            PoseStamped, '/pick_place/object_pose', latched_qos
        )
        self.generation_publisher = self.node.create_publisher(
            UInt32, '/pick_place/object_generation', latched_qos
        )
        self.grasp_publisher = self.node.create_publisher(
            Bool, '/pick_place/grasp_attached', latched_qos
        )
        self.node.create_subscription(
            Bool, '/pick_place/gripper_close', self.gripper_callback, 10
        )
        self.node.create_subscription(
            Empty, '/pick_place/reset_object', self.reset_callback, 10
        )

        rigid_body = UsdPhysics.RigidBodyAPI(
            omni.usd.get_context().get_stage().GetPrimAtPath(OBJECT_PRIM_PATH)
        )
        self.grasp_joint: UsdPhysics.FixedJoint | None = None
        object_rigid_view = self.pick_object._rigid_prim_view
        if not object_rigid_view.is_physics_handle_valid():
            raise RuntimeError('Pick-object PhysX view was not initialized.')
        # DynamicCuboid.set_world_pose() teleports the actor, but it does not
        # update a kinematic actor's target. Keep a dedicated one-body PhysX
        # view so the attached cube is advanced by the simulation and renderer.
        self.object_physics_view = object_rigid_view._physics_view
        self.object_backend_utils = object_rigid_view._backend_utils
        self.object_physics_device = object_rigid_view._device
        self.object_indices = self.object_backend_utils.resolve_indices(
            None, object_rigid_view.count, self.object_physics_device
        )

    def gripper_callback(self, message: Bool) -> None:
        self.requested_close = bool(message.data)

    def reset_callback(self, _: Empty) -> None:
        self.reset_pending = True

    def gripper_targets(self) -> np.ndarray:
        value = (
            self.gripper_closed_position
            if self.requested_close
            else self.gripper_open_position
        )
        return np.asarray([value], dtype=np.float32)

    def publish_state(self) -> None:
        position, orientation = self.pick_object.get_world_pose()
        pose_message = PoseStamped()
        pose_message.header.frame_id = 'world'
        pose_message.header.stamp = self.node.get_clock().now().to_msg()
        pose_message.pose.position.x = float(position[0])
        pose_message.pose.position.y = float(position[1])
        pose_message.pose.position.z = float(position[2])
        pose_message.pose.orientation.w = float(orientation[0])
        pose_message.pose.orientation.x = float(orientation[1])
        pose_message.pose.orientation.y = float(orientation[2])
        pose_message.pose.orientation.z = float(orientation[3])
        self.object_pose_publisher.publish(pose_message)

        generation_message = UInt32()
        generation_message.data = self.object_generation
        self.generation_publisher.publish(generation_message)

        grasp_message = Bool()
        grasp_message.data = self.object_attached
        self.grasp_publisher.publish(grasp_message)

    def release_object(self) -> None:
        if self.object_attached:
            self.object_attached = False
            if self.grasp_joint is not None:
                self.grasp_joint.GetPrim().SetActive(False)
                self.grasp_joint = None
            self.pick_object.set_linear_velocity(np.zeros(3, dtype=np.float32))
            self.pick_object.set_angular_velocity(np.zeros(3, dtype=np.float32))

    def reset_object(self) -> None:
        self.requested_close = False
        self.previous_requested_close = False
        self.release_object()
        position = np.array(
            [
                self.random_generator.uniform(self.spawn_min[0], self.spawn_max[0]),
                self.random_generator.uniform(self.spawn_min[1], self.spawn_max[1]),
                self.spawn_min[2],
            ],
            dtype=np.float32,
        )
        self.pick_object.set_world_pose(
            position=position,
            orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        )
        self.pick_object.set_linear_velocity(np.zeros(3, dtype=np.float32))
        self.pick_object.set_angular_velocity(np.zeros(3, dtype=np.float32))
        self.object_generation += 1
        self.reset_pending = False
        self.publish_state()
        print(
            f'Spawned object generation {self.object_generation} at '
            f'{position.tolist()}',
            flush=True,
        )

    def try_attach_object(self) -> None:
        tcp_position, tcp_orientation = self.tcp_world_pose()
        object_position, object_orientation = self.pick_object.get_world_pose()
        distance = float(np.linalg.norm(object_position - tcp_position))
        if abs(distance - self.expected_grasp_distance) > self.max_grasp_distance:
            print(
                'Grasp rejected: object/TCP distance '
                f'{distance:.3f} m (expected {self.expected_grasp_distance:.3f} m)',
                flush=True,
            )
            return

        inverse_tcp = quaternion_conjugate(tcp_orientation)
        self.relative_position = rotate_vector(
            inverse_tcp, np.asarray(object_position) - tcp_position
        )
        self.relative_orientation = quaternion_multiply(
            inverse_tcp, np.asarray(object_orientation)
        )
        stage = omni.usd.get_context().get_stage()
        joint_path = Sdf.Path('/World/PickObjectGraspJoint')
        if stage.GetPrimAtPath(joint_path).IsValid():
            stage.RemovePrim(joint_path)
        self.grasp_joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
        self.grasp_joint.CreateBody0Rel().SetTargets(
            [self.wrist_prim.GetPath()]
        )
        self.grasp_joint.CreateBody1Rel().SetTargets(
            [Sdf.Path(OBJECT_PRIM_PATH)]
        )
        wrist_position, wrist_orientation = prim_world_pose(self.wrist_prim)
        inverse_wrist = quaternion_conjugate(wrist_orientation)
        local_position = rotate_vector(
            inverse_wrist, np.asarray(object_position) - wrist_position
        )
        local_orientation = quaternion_multiply(
            inverse_wrist, np.asarray(object_orientation)
        )
        self.grasp_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_position))
        self.grasp_joint.CreateLocalRot0Attr().Set(
            Gf.Quatf(
                float(local_orientation[0]),
                Gf.Vec3f(*local_orientation[1:]),
            )
        )
        self.object_attached = True
        print('Robotiq grasp attached the object', flush=True)

    def tcp_world_pose(self) -> tuple[np.ndarray, np.ndarray]:
        wrist_position, wrist_orientation = prim_world_pose(self.wrist_prim)
        tcp_position = wrist_position + rotate_vector(
            wrist_orientation, self.tcp_offset_from_wrist
        )
        return tcp_position, wrist_orientation

    def update(self, _: float) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.reset_pending:
            self.reset_object()

        self.robot.apply_action(
            ArticulationAction(
                joint_positions=self.gripper_targets(),
                joint_indices=self.gripper_indices,
            )
        )

        if self.requested_close and not self.previous_requested_close:
            self.try_attach_object()
        elif not self.requested_close and self.previous_requested_close:
            self.release_object()
            print('Robotiq released the object', flush=True)
        self.previous_requested_close = self.requested_close

        self.frames_since_publish += 1
        if self.frames_since_publish >= 6:
            self.frames_since_publish = 0
            self.publish_state()

    def close(self) -> None:
        self.release_object()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


scene_bridge: SceneRosBridge | None = None

try:
    parameters = load_scene_parameters()
    input_positions = read_initial_joint_positions()
    object_size = require_positive_vector(parameters, 'object_size', 3)
    object_spawn_min = require_vector(parameters, 'object_spawn_min', 3)
    object_mass = float(parameters.get('object_mass', 0.08))
    bin_centers_flat = require_vector(parameters, 'bin_centers', 9)
    bin_centers = bin_centers_flat.reshape((3, 3))
    bin_outer_size = require_positive_vector(parameters, 'bin_outer_size', 3)
    bin_wall_thickness = float(parameters.get('bin_wall_thickness', 0.015))
    bin_floor_thickness = float(parameters.get('bin_floor_thickness', 0.02))
    obstacle_position = require_vector(parameters, 'obstacle_position', 3)
    obstacle_size = require_positive_vector(parameters, 'obstacle_size', 3)
    if (
        not math.isfinite(object_mass)
        or object_mass <= 0.0
        or bin_wall_thickness <= 0.0
        or bin_floor_thickness <= 0.0
        or 2.0 * bin_wall_thickness >= min(bin_outer_size[0:2])
        or bin_floor_thickness >= bin_outer_size[2]
    ):
        raise RuntimeError('Invalid object mass or bin dimensions.')

    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError('Isaac Sim asset root could not be resolved.')
    robot_usd_path = (
        assets_root + '/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd'
    )

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0)
    world.scene.add_default_ground_plane()
    viewports.set_camera_view(
        eye=np.array([2.4, 2.2, 1.8]),
        target=np.array([0.55, 0.05, 0.35]),
    )

    add_reference_to_stage(robot_usd_path, ROBOT_PRIM_PATH)
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    variants = robot_prim.GetVariantSets()
    if not variants.HasVariantSet(ROBOT_VARIANT_SET):
        raise RuntimeError(
            f'UR10e asset has no {ROBOT_VARIANT_SET!r} variant set.'
        )
    gripper_variant = variants.GetVariantSet(ROBOT_VARIANT_SET)
    if ROBOT_VARIANT not in gripper_variant.GetVariantNames():
        raise RuntimeError(
            f'UR10e asset has no {ROBOT_VARIANT!r} gripper variant.'
        )
    gripper_variant.SetVariantSelection(ROBOT_VARIANT)
    simulation_app.update()

    robot = world.scene.add(
        SingleArticulation(prim_path=ROBOT_PRIM_PATH, name='ur10e_robotiq')
    )
    pick_object = world.scene.add(
        DynamicCuboid(
            prim_path=OBJECT_PRIM_PATH,
            name='pick_object',
            position=object_spawn_min.astype(np.float32),
            scale=object_size.astype(np.float32),
            size=1.0,
            color=np.array([0.85, 0.15, 0.10]),
            mass=object_mass,
        )
    )

    bin_colors = [
        np.array([0.10, 0.35, 0.90]),
        np.array([0.15, 0.70, 0.25]),
        np.array([0.90, 0.65, 0.10]),
    ]
    for bin_index, bin_center in enumerate(bin_centers):
        add_bin(
            world,
            bin_index,
            bin_center,
            bin_outer_size,
            bin_wall_thickness,
            bin_floor_thickness,
            bin_colors[bin_index],
        )
    add_fixed_cuboid(
        world,
        'StaticObstacle',
        obstacle_position,
        obstacle_size,
        np.array([0.55, 0.20, 0.65]),
    )

    simulation_app.update()
    articulation_root_path = find_unique_descendant(
        stage,
        ROBOT_PRIM_PATH,
        lambda prim: prim.HasAPI(UsdPhysics.ArticulationRootAPI),
        'articulation root',
    )
    wrist_path = find_unique_descendant(
        stage,
        ROBOT_PRIM_PATH,
        lambda prim: prim.GetName() == 'wrist_3_link',
        'wrist_3_link prim',
    )
    wrist_prim = stage.GetPrimAtPath(wrist_path)
    print('USD articulation root:', articulation_root_path, flush=True)
    print('Robotiq wrist body:', wrist_path, flush=True)
    create_ros_action_graph(articulation_root_path)
    simulation_app.update()

    world.reset()
    print('UR10e/Robotiq DOF names:', robot.dof_names, flush=True)
    missing_arm = [name for name in ARM_JOINT_NAMES if name not in robot.dof_names]
    missing_gripper = [
        name for name in GRIPPER_JOINT_NAMES if name not in robot.dof_names
    ]
    if missing_arm or missing_gripper:
        raise RuntimeError(
            f'Missing arm joints {missing_arm} or gripper joints {missing_gripper}; '
            f'available DOFs: {robot.dof_names}'
        )

    initial_by_name = dict(zip(ARM_JOINT_NAMES, input_positions))
    initial_positions = np.asarray(
        [initial_by_name.get(name, 0.0) for name in robot.dof_names],
        dtype=np.float32,
    )
    robot.set_joints_default_state(
        positions=initial_positions,
        velocities=np.zeros(robot.num_dof, dtype=np.float32),
    )
    world.reset()
    robot.set_joint_positions(initial_positions)
    robot.apply_action(ArticulationAction(joint_positions=initial_positions))

    scene_bridge = SceneRosBridge(
        world, robot, pick_object, wrist_prim, parameters
    )
    world.add_physics_callback('pick_place_scene_bridge', scene_bridge.update)
    print(
        'UR10e with Robotiq 2F-140 initialized with ROS 2 arm and scene bridge',
        flush=True,
    )
    print('Publishing: /isaac_joint_states, /clock, and object/grasp state', flush=True)
    print('Subscribing: /isaac_joint_commands and pick/place scene commands', flush=True)

    while simulation_app.is_running():
        # render=True also evaluates the OmniGraph in headless Isaac Sim 5.1.
        world.step(render=True)
        og.Controller.set(
            og.Controller.attribute(
                '/ActionGraph/OnImpulseEvent.state:enableImpulse'
            ),
            True,
        )

except KeyboardInterrupt:
    pass

finally:
    if scene_bridge is not None:
        scene_bridge.close()
    simulation_app.close()
