#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

#define ANSI_COLOR_ERROR    "\x1b[31m" //red
#define ANSI_COLOR_INFO   "\x1b[32m" //green
#define ANSI_COLOR_WARN  "\x1b[33m"  //yellow
#define ANSI_COLOR_RESET   "\x1b[0m"  //reset

namespace manipulator_sim
{

class RandomPickPlacePlanner
{
public:
  explicit RandomPickPlacePlanner(const rclcpp::Node::SharedPtr & node)
  : node_(node), random_engine_(std::random_device{}())
  {
    planning_group_ = parameter<std::string>("planning_group", "ur_manipulator");
    end_effector_link_ = parameter<std::string>("end_effector_link", "gripper_tcp");
    gripper_mount_link_ = parameter<std::string>("gripper_mount_link", "wrist_3_link");
    planner_id_ = parameter<std::string>("planner_id", "");
    planning_time_s_ = parameter<double>("planning_time", 8.0);
    planning_attempts_ = static_cast<int>(parameter<int64_t>("planning_attempts", 8));
    planning_retries_ = static_cast<int>(parameter<int64_t>("planning_retries", 4));
    velocity_scaling_ = parameter<double>("velocity_scaling", 0.2);
    acceleration_scaling_ = parameter<double>("acceleration_scaling", 0.2);
    position_tolerance_m_ = parameter<double>("position_tolerance", 0.01);
    orientation_tolerance_rad_ = parameter<double>("orientation_tolerance", 0.05);
    approach_height_m_ = parameter<double>("approach_height", 0.18);
    cartesian_step_m_ = parameter<double>("cartesian_step", 0.005);
    dwell_time_s_ = parameter<double>("dwell_time", 1.0);
    retry_delay_s_ = parameter<double>("retry_delay", 2.0);
    scene_timeout_s_ = parameter<double>("scene_timeout", 8.0);
    placement_timeout_s_ = parameter<double>("placement_timeout", 4.0);
    max_cycles_ = static_cast<int>(parameter<int64_t>("max_cycles", 0));
    const auto seed = parameter<int64_t>("random_seed", -1);

    joint_state_topic_ = parameter<std::string>("joint_state_topic", "/joint_states");
    joint_state_timeout_s_ = parameter<double>("joint_state_timeout", 1.0);
    minimum_state_samples_ = static_cast<int>(
      parameter<int64_t>("minimum_state_samples", 3));
    joint_names_ = parameter<std::vector<std::string>>(
      "joint_names",
      {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
      });

    grasp_orientation_ = quaternion_parameter(
      "grasp_orientation_xyzw", {0.70710678, 0.70710678, 0.0, 0.0});
    object_to_tcp_offset_ = vector3_parameter(
      "object_to_tcp_offset", {0.0, 0.0, 0.0});
    object_id_ = parameter<std::string>("object_id", "pick_object");
    object_size_ = positive_vector3_parameter("object_size", {0.055, 0.055, 0.055});
    bin_centers_ = bin_centers_parameter();
    bin_outer_size_ = positive_vector3_parameter("bin_outer_size", {0.22, 0.20, 0.14});
    bin_wall_thickness_ = parameter<double>("bin_wall_thickness", 0.015);
    bin_floor_thickness_ = parameter<double>("bin_floor_thickness", 0.02);
    bin_drop_height_ = parameter<double>("bin_drop_height", 0.22);
    obstacle_position_ = vector3_parameter("obstacle_position", {0.62, 0.05, 0.25});
    obstacle_size_ = positive_vector3_parameter("obstacle_size", {0.22, 0.16, 0.50});
    ground_size_ = positive_vector3_parameter("ground_size", {2.4, 2.4, 0.05});
    gripper_body_size_ = positive_vector3_parameter(
      "gripper_body_size", {0.14, 0.14, 0.18});
    gripper_body_position_ = vector3_parameter(
      "gripper_body_position", {0.0, 0.0, 0.10});
    gripper_finger_size_ = positive_vector3_parameter(
      "gripper_finger_size", {0.025, 0.035, 0.07});
    gripper_finger_positions_ = parameter<std::vector<double>>(
      "gripper_finger_positions", {-0.055, 0.0, 0.185, 0.055, 0.0, 0.185});
    if (gripper_finger_positions_.size() != 6 ||
      !all_finite(gripper_finger_positions_))
    {
      throw std::invalid_argument(
              "gripper_finger_positions must contain two finite xyz positions");
    }

    object_pose_topic_ = parameter<std::string>(
      "object_pose_topic", "/pick_place/object_pose");
    object_generation_topic_ = parameter<std::string>(
      "object_generation_topic", "/pick_place/object_generation");
    reset_object_topic_ = parameter<std::string>(
      "reset_object_topic", "/pick_place/reset_object");
    gripper_command_topic_ = parameter<std::string>(
      "gripper_command_topic", "/pick_place/gripper_close");
    grasp_state_topic_ = parameter<std::string>(
      "grasp_state_topic", "/pick_place/grasp_attached");

    touch_links_ = parameter<std::vector<std::string>>(
      "gripper_touch_links",
      {
        "base_link", "base_link_inertia", "shoulder_link", "upper_arm_link",
        "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link",
        "flange", "tool0",
      });

    validate_parameters();
    if (seed >= 0) {
      random_engine_.seed(static_cast<std::mt19937::result_type>(seed));
    }

    goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pick_place/goal_pose", rclcpp::QoS(10).reliable());
    gripper_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      gripper_command_topic_, rclcpp::QoS(10).reliable());
    reset_object_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(
      reset_object_topic_, rclcpp::QoS(10).reliable());
    selected_bin_publisher_ = node_->create_publisher<std_msgs::msg::Int32>(
      "/pick_place/selected_bin", rclcpp::QoS(1).transient_local().reliable());
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/pick_place/status", rclcpp::QoS(10).transient_local().reliable());

    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr message) {
        joint_state_callback(message);
      });
    object_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      object_pose_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        object_pose_callback(message);
      });
    object_generation_subscription_ = node_->create_subscription<std_msgs::msg::UInt32>(
      object_generation_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::UInt32::ConstSharedPtr message) {
        object_generation_callback(message);
      });
    grasp_state_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      grasp_state_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
        grasp_state_callback(message);
      });
  }

  void run()
  {
    publish_status("Waiting for MoveIt, Isaac feedback, and the physical scene");
    moveit::planning_interface::MoveGroupInterface move_group(node_, planning_group_);
    configure_move_group(move_group);

    move_group.startStateMonitor();
    while (rclcpp::ok()) {
      if (move_group.getCurrentState(1.0) && has_fresh_joint_state_stream() &&
        wait_for_initial_scene(0.1))
      {
        break;
      }
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        ANSI_COLOR_WARN "Waiting for complete arm state and Isaac object/grasp state" ANSI_COLOR_RESET);
    }
    if (!rclcpp::ok()) {
      return;
    }

    initialize_fixed_planning_scene();
    RCLCPP_INFO(
      node_->get_logger(),
      ANSI_COLOR_INFO "Physical pick/place planner ready (group=%s, wrist=%s, frame=%s)" ANSI_COLOR_RESET,
      planning_group_.c_str(), end_effector_link_.c_str(), move_group.getPlanningFrame().c_str());

    int completed_cycles = 0;
    while (rclcpp::ok() && (max_cycles_ <= 0 || completed_cycles < max_cycles_)) {
      if (execute_cycle(move_group, completed_cycles + 1)) {
        ++completed_cycles;
        RCLCPP_INFO(node_->get_logger(), ANSI_COLOR_INFO "Completed physical pick/place cycle %d" ANSI_COLOR_RESET,
            completed_cycles);
      } else if (rclcpp::ok()) {
        recover_from_failure(move_group);
        publish_status("Cycle failed; respawning the object after retry delay");
        interruptible_sleep(retry_delay_s_);
      }
    }

    recover_from_failure(move_group);
    publish_status("Pick/place planner stopped");
  }

private:
  using Vector3 = std::array<double, 3>;

  struct Bin
  {
    Vector3 center;
  };

  template<typename T>
  T parameter(const std::string & name, const T & default_value)
  {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
    T value{};
    if (!node_->get_parameter(name, value)) {
      return default_value;
    }
    return value;
  }

  static bool all_finite(const std::vector<double> & values)
  {
    return std::all_of(
      values.begin(), values.end(), [](double value) {return std::isfinite(value);});
  }

  Vector3 vector3_parameter(
    const std::string & name, const std::vector<double> & default_value)
  {
    const auto values = parameter<std::vector<double>>(name, default_value);
    if (values.size() != 3 || !all_finite(values)) {
      throw std::invalid_argument(name + " must contain three finite values");
    }
    return {values[0], values[1], values[2]};
  }

  Vector3 positive_vector3_parameter(
    const std::string & name, const std::vector<double> & default_value)
  {
    const auto values = vector3_parameter(name, default_value);
    if (std::any_of(values.begin(), values.end(), [](double value) {return value <= 0.0;})) {
      throw std::invalid_argument(name + " must contain positive values");
    }
    return values;
  }

  geometry_msgs::msg::Quaternion quaternion_parameter(
    const std::string & name, const std::vector<double> & default_value)
  {
    const auto values = parameter<std::vector<double>>(name, default_value);
    if (values.size() != 4 || !all_finite(values)) {
      throw std::invalid_argument(name + " must contain four finite xyzw values");
    }
    const double norm = std::sqrt(
      values[0] * values[0] + values[1] * values[1] +
      values[2] * values[2] + values[3] * values[3]);
    if (norm <= 1.0e-9) {
      throw std::invalid_argument(name + " must have a non-zero norm");
    }
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = values[0] / norm;
    quaternion.y = values[1] / norm;
    quaternion.z = values[2] / norm;
    quaternion.w = values[3] / norm;
    return quaternion;
  }

  std::vector<Bin> bin_centers_parameter()
  {
    const auto values = parameter<std::vector<double>>(
      "bin_centers", {0.36, 0.48, 0.0, 0.64, 0.48, 0.0, 0.92, 0.48, 0.0});
    if (values.size() != 9 || !all_finite(values)) {
      throw std::invalid_argument("bin_centers must contain exactly three xyz positions");
    }
    std::vector<Bin> bins;
    for (std::size_t offset = 0; offset < values.size(); offset += 3) {
      bins.push_back({{values[offset], values[offset + 1], values[offset + 2]}});
    }
    return bins;
  }

  void validate_parameters() const
  {
    if (planning_time_s_ <= 0.0 || planning_attempts_ <= 0 || planning_retries_ <= 0) {
      throw std::invalid_argument(
              "planning_time, planning_attempts, and planning_retries must be positive");
    }
    if (approach_height_m_ <= 0.0 || cartesian_step_m_ <= 0.0 ||
      scene_timeout_s_ <= 0.0 ||
      placement_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("approach, Cartesian step, and scene timeouts must be positive");
    }
    if (joint_state_timeout_s_ <= 0.0 || minimum_state_samples_ <= 0 || joint_names_.empty()) {
      throw std::invalid_argument(
              "joint_state_timeout, minimum_state_samples, and joint_names must be valid");
    }
    if (velocity_scaling_ <= 0.0 || velocity_scaling_ > 1.0 ||
      acceleration_scaling_ <= 0.0 || acceleration_scaling_ > 1.0)
    {
      throw std::invalid_argument("velocity and acceleration scaling must be in (0, 1]");
    }
    if (bin_wall_thickness_ <= 0.0 || bin_floor_thickness_ <= 0.0 ||
      2.0 * bin_wall_thickness_ >= std::min(bin_outer_size_[0], bin_outer_size_[1]) ||
      bin_floor_thickness_ >= bin_outer_size_[2] || bin_drop_height_ <= bin_outer_size_[2])
    {
      throw std::invalid_argument("bin wall, floor, or drop dimensions are invalid");
    }
    if (object_id_.empty() || gripper_mount_link_.empty() || touch_links_.empty()) {
      throw std::invalid_argument("object_id, gripper_mount_link, and touch_links are required");
    }
  }

  void configure_move_group(
    moveit::planning_interface::MoveGroupInterface & move_group) const
  {
    move_group.setEndEffectorLink(end_effector_link_);
    move_group.setPlanningPipelineId("ompl");
    if (!planner_id_.empty()) {
      move_group.setPlannerId(planner_id_);
    }
    move_group.setPlanningTime(planning_time_s_);
    move_group.setNumPlanningAttempts(planning_attempts_);
    move_group.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group.setGoalPositionTolerance(position_tolerance_m_);
    move_group.setGoalOrientationTolerance(orientation_tolerance_rad_);
    move_group.allowReplanning(false);
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr & message)
  {
    if (message->name.size() != message->position.size()) {
      return;
    }
    for (const auto & required_name : joint_names_) {
      const auto iterator = std::find(
        message->name.begin(), message->name.end(), required_name);
      if (iterator == message->name.end()) {
        return;
      }
      const auto index = static_cast<std::size_t>(
        std::distance(message->name.begin(), iterator));
      if (!std::isfinite(message->position[index])) {
        return;
      }
    }

    const auto received_at = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (valid_joint_state_samples_ > 0 &&
      std::chrono::duration<double>(received_at - last_joint_state_time_).count() >
      joint_state_timeout_s_)
    {
      valid_joint_state_samples_ = 0;
    }
    ++valid_joint_state_samples_;
    last_joint_state_time_ = received_at;
  }

  void object_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & message)
  {
    const auto & position = message->pose.position;
    if (message->header.frame_id.empty() || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(position.z))
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    object_pose_ = *message;
    have_object_pose_ = true;
    object_pose_received_at_ = std::chrono::steady_clock::now();
  }

  void object_generation_callback(const std_msgs::msg::UInt32::ConstSharedPtr & message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool generation_changed =
      !have_object_generation_ || message->data != object_generation_;
    object_generation_ = message->data;
    have_object_generation_ = message->data > 0;
    if (generation_changed) {
      object_generation_received_at_ = std::chrono::steady_clock::now();
    }
  }

  void grasp_state_callback(const std_msgs::msg::Bool::ConstSharedPtr & message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    grasp_attached_ = message->data;
    have_grasp_state_ = true;
  }

  bool has_fresh_joint_state_stream() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (valid_joint_state_samples_ < minimum_state_samples_) {
      return false;
    }
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_joint_state_time_).count() <=
           joint_state_timeout_s_;
  }

  bool wait_for_initial_scene(double timeout_s) const
  {
    return wait_until(
      [this]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return have_object_pose_ && have_object_generation_ && have_grasp_state_;
      }, timeout_s);
  }

  template<typename Predicate>
  bool wait_until(Predicate predicate, double timeout_s) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(0.0, timeout_s)));
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return predicate();
  }

  geometry_msgs::msg::PoseStamped latest_object_pose() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return object_pose_;
  }

  uint32_t current_object_generation() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return object_generation_;
  }

  bool request_object_for_cycle(geometry_msgs::msg::PoseStamped & pose)
  {
    if (last_used_object_generation_ == 0) {
      if (!wait_for_initial_scene(scene_timeout_s_)) {
        RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "Timed out waiting for the initial Isaac object pose" ANSI_COLOR_RESET);
        return false;
      }
    } else {
      const uint32_t previous_generation = current_object_generation();
      std_msgs::msg::Empty reset_message;
      reset_object_publisher_->publish(reset_message);
      publish_status("Requested a new random object position from Isaac");
      if (!wait_until(
          [this, previous_generation]() {
            std::lock_guard<std::mutex> lock(state_mutex_);
            return object_generation_ > previous_generation && have_object_pose_ &&
                   object_pose_received_at_ >= object_generation_received_at_;
          }, scene_timeout_s_))
      {
        RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "Isaac did not confirm a new object generation" ANSI_COLOR_RESET);
        return false;
      }
    }

    pose = latest_object_pose();
    last_used_object_generation_ = current_object_generation();
    return true;
  }

  static geometry_msgs::msg::Pose identity_pose(const Vector3 & position)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.w = 1.0;
    return pose;
  }

  static shape_msgs::msg::SolidPrimitive box_primitive(const Vector3 & size)
  {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {size[0], size[1], size[2]};
    return primitive;
  }

  static moveit_msgs::msg::CollisionObject box_object(
    const std::string & id,
    const std::string & frame,
    const Vector3 & position,
    const Vector3 & size)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame;
    object.id = id;
    object.primitives.push_back(box_primitive(size));
    object.primitive_poses.push_back(identity_pose(position));
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
  }

  static void append_box(
    moveit_msgs::msg::CollisionObject & object,
    const Vector3 & position,
    const Vector3 & size)
  {
    object.primitives.push_back(box_primitive(size));
    object.primitive_poses.push_back(identity_pose(position));
  }

  moveit_msgs::msg::CollisionObject bin_collision_object(
    std::size_t index, const Bin & bin) const
  {
    const double x_size = bin_outer_size_[0];
    const double y_size = bin_outer_size_[1];
    const double height = bin_outer_size_[2];
    const double x = bin.center[0];
    const double y = bin.center[1];
    const double z = bin.center[2];
    const double wall = bin_wall_thickness_;

    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "world";
    object.id = "bin_" + std::to_string(index);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    append_box(
      object, {x, y, z + bin_floor_thickness_ / 2.0},
      {x_size, y_size, bin_floor_thickness_});
    append_box(
      object, {x - (x_size - wall) / 2.0, y, z + height / 2.0},
      {wall, y_size, height});
    append_box(
      object, {x + (x_size - wall) / 2.0, y, z + height / 2.0},
      {wall, y_size, height});
    append_box(
      object, {x, y - (y_size - wall) / 2.0, z + height / 2.0},
      {x_size - 2.0 * wall, wall, height});
    append_box(
      object, {x, y + (y_size - wall) / 2.0, z + height / 2.0},
      {x_size - 2.0 * wall, wall, height});
    return object;
  }

  void initialize_fixed_planning_scene()
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(
      box_object(
        "ground", "world", {0.0, 0.0, -ground_size_[2] / 2.0}, ground_size_));
    objects.push_back(
      box_object("static_obstacle", "world", obstacle_position_, obstacle_size_));
    for (std::size_t index = 0; index < bin_centers_.size(); ++index) {
      objects.push_back(bin_collision_object(index, bin_centers_[index]));
    }
    if (!planning_scene_interface_.applyCollisionObjects(objects)) {
      throw std::runtime_error("MoveIt rejected the fixed collision scene");
    }

    moveit_msgs::msg::AttachedCollisionObject gripper;
    gripper.link_name = gripper_mount_link_;
    gripper.touch_links = touch_links_;
    gripper.object.header.frame_id = gripper_mount_link_;
    gripper.object.id = "robotiq_2f_140_collision_proxy";
    gripper.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    append_box(gripper.object, gripper_body_position_, gripper_body_size_);
    append_box(
      gripper.object,
      {gripper_finger_positions_[0], gripper_finger_positions_[1],
        gripper_finger_positions_[2]},
      gripper_finger_size_);
    append_box(
      gripper.object,
      {gripper_finger_positions_[3], gripper_finger_positions_[4],
        gripper_finger_positions_[5]},
      gripper_finger_size_);
    if (!planning_scene_interface_.applyAttachedCollisionObject(gripper)) {
      throw std::runtime_error("MoveIt rejected the Robotiq collision proxy");
    }
    publish_status("MoveIt scene contains the ground, three bins, Robotiq, and obstacle");
  }

  bool update_object_collision(const geometry_msgs::msg::PoseStamped & pose)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header = pose.header;
    object.id = object_id_;
    object.primitives.push_back(box_primitive(object_size_));
    object.primitive_poses.push_back(pose.pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return planning_scene_interface_.applyCollisionObject(object);
  }

  bool remove_object_collision()
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "world";
    object.id = object_id_;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!planning_scene_interface_.applyCollisionObject(object)) {
      return false;
    }
    return wait_until(
      [this]() {
        return planning_scene_interface_.getObjects({object_id_}).empty();
      }, scene_timeout_s_);
  }

  static Vector3 rotate_vector(
    const geometry_msgs::msg::Quaternion & quaternion, const Vector3 & vector)
  {
    const Vector3 q_vector{quaternion.x, quaternion.y, quaternion.z};
    const Vector3 cross_one{
      q_vector[1] * vector[2] - q_vector[2] * vector[1],
      q_vector[2] * vector[0] - q_vector[0] * vector[2],
      q_vector[0] * vector[1] - q_vector[1] * vector[0]};
    const Vector3 cross_two{
      q_vector[1] * cross_one[2] - q_vector[2] * cross_one[1],
      q_vector[2] * cross_one[0] - q_vector[0] * cross_one[2],
      q_vector[0] * cross_one[1] - q_vector[1] * cross_one[0]};
    return {
      vector[0] + 2.0 * (quaternion.w * cross_one[0] + cross_two[0]),
      vector[1] + 2.0 * (quaternion.w * cross_one[1] + cross_two[1]),
      vector[2] + 2.0 * (quaternion.w * cross_one[2] + cross_two[2])};
  }

  geometry_msgs::msg::PoseStamped tcp_pose_for_object(
    const geometry_msgs::msg::PoseStamped & object_pose) const
  {
    geometry_msgs::msg::PoseStamped wrist_pose;
    wrist_pose.header = object_pose.header;
    wrist_pose.header.stamp = node_->now();
    wrist_pose.pose.orientation = grasp_orientation_;
    const auto offset = rotate_vector(grasp_orientation_, object_to_tcp_offset_);
    wrist_pose.pose.position.x = object_pose.pose.position.x + offset[0];
    wrist_pose.pose.position.y = object_pose.pose.position.y + offset[1];
    wrist_pose.pose.position.z = object_pose.pose.position.z + offset[2];
    return wrist_pose;
  }

  bool plan_and_execute(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const geometry_msgs::msg::PoseStamped & target,
    const std::string & phase)
  {
    target_log(phase, target);
    goal_publisher_->publish(target);
    publish_status("Planning " + phase);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= planning_retries_ && rclcpp::ok(); ++attempt) {
      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(target, end_effector_link_);
      const auto planning_result = move_group.plan(plan);
      move_group.clearPoseTargets();
      if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        node_->get_logger(), ANSI_COLOR_WARN "Planning attempt %d/%d failed for phase '%s'" ANSI_COLOR_RESET,
        attempt, planning_retries_, phase.c_str());
    }
    if (!planned) {
      return false;
    }

    publish_status("Executing " + phase);
    const auto execution_result = move_group.execute(plan);
    if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "Execution failed for phase '%s'" ANSI_COLOR_RESET, phase.c_str());
      return false;
    }
    return true;
  }

  bool cartesian_execute(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const geometry_msgs::msg::PoseStamped & target,
    const std::string & phase)
  {
    target_log(phase, target);
    goal_publisher_->publish(target);
    publish_status("Computing Cartesian " + phase);

    move_group.setPoseReferenceFrame(target.header.frame_id);
    move_group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory_message;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    const double fraction = move_group.computeCartesianPath(
      {target.pose}, cartesian_step_m_, trajectory_message, true, &error_code);
    if (fraction < 0.999) {
      RCLCPP_WARN(
        node_->get_logger(),
        ANSI_COLOR_WARN "Cartesian phase '%s' reached %.1f%% (MoveIt code %d)" ANSI_COLOR_RESET,
        phase.c_str(), 100.0 * std::max(0.0, fraction), error_code.val);
      return false;
    }

    const auto current_state = move_group.getCurrentState(1.0);
    if (!current_state) {
      RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "No current state for Cartesian phase '%s'" ANSI_COLOR_RESET, phase.c_str());
      return false;
    }
    robot_trajectory::RobotTrajectory timed_trajectory(move_group.getRobotModel(), planning_group_);
    timed_trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        timed_trajectory, velocity_scaling_, acceleration_scaling_))
    {
      RCLCPP_ERROR(
        node_->get_logger(), ANSI_COLOR_ERROR "Could not time-parameterize Cartesian phase '%s'" ANSI_COLOR_RESET,
        phase.c_str());
      return false;
    }
    timed_trajectory.getRobotTrajectoryMsg(trajectory_message);

    publish_status("Executing Cartesian " + phase);
    const auto execution_result = move_group.execute(trajectory_message);
    if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        node_->get_logger(), ANSI_COLOR_ERROR "Execution failed for Cartesian phase '%s'" ANSI_COLOR_RESET,
        phase.c_str());
      return false;
    }
    return true;
  }

  bool command_gripper(bool close)
  {
    std_msgs::msg::Bool message;
    message.data = close;
    gripper_publisher_->publish(message);
    if (!wait_until(
        [this, close]() {
          std::lock_guard<std::mutex> lock(state_mutex_);
          return have_grasp_state_ && grasp_attached_ == close;
        }, scene_timeout_s_))
    {
      RCLCPP_ERROR(
        node_->get_logger(), ANSI_COLOR_ERROR "Isaac did not confirm gripper state '%s'" ANSI_COLOR_RESET,
        close ? "attached" : "released");
      return false;
    }
    return true;
  }

  bool attach_object_in_moveit(
    moveit::planning_interface::MoveGroupInterface & move_group)
  {
    if (!move_group.attachObject(object_id_, gripper_mount_link_, touch_links_)) {
      RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "MoveIt rejected object attachment" ANSI_COLOR_RESET);
      return false;
    }
    if (!wait_until(
        [this]() {
          return !planning_scene_interface_.getAttachedObjects({object_id_}).empty();
        }, scene_timeout_s_))
    {
      RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "MoveIt did not report the object as attached" ANSI_COLOR_RESET);
      return false;
    }
    planning_object_attached_ = true;
    return true;
  }

  bool detach_object_in_moveit(
    moveit::planning_interface::MoveGroupInterface & move_group)
  {
    if (!planning_object_attached_) {
      return true;
    }
    if (!move_group.detachObject(object_id_)) {
      RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "MoveIt rejected object detachment" ANSI_COLOR_RESET);
      return false;
    }
    if (!wait_until(
        [this]() {
          return planning_scene_interface_.getAttachedObjects({object_id_}).empty();
        }, scene_timeout_s_))
    {
      RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "MoveIt still reports the object as attached" ANSI_COLOR_RESET);
      return false;
    }
    planning_object_attached_ = false;
    return update_object_collision(latest_object_pose());
  }

  bool object_is_in_bin(const Bin & bin) const
  {
    const auto pose = latest_object_pose();
    const double inner_x_half =
      (bin_outer_size_[0] - 2.0 * bin_wall_thickness_ - object_size_[0]) / 2.0;
    const double inner_y_half =
      (bin_outer_size_[1] - 2.0 * bin_wall_thickness_ - object_size_[1]) / 2.0;
    return std::abs(pose.pose.position.x - bin.center[0]) <= inner_x_half &&
           std::abs(pose.pose.position.y - bin.center[1]) <= inner_y_half &&
           pose.pose.position.z >= bin.center[2] &&
           pose.pose.position.z <= bin.center[2] + bin_outer_size_[2] + object_size_[2];
  }

  bool execute_cycle(
    moveit::planning_interface::MoveGroupInterface & move_group, int cycle_number)
  {
    publish_status("Starting cycle " + std::to_string(cycle_number));
    if (!command_gripper(false)) {
      return false;
    }

    geometry_msgs::msg::PoseStamped object_pose;
    if (!request_object_for_cycle(object_pose)) {
      return false;
    }
    if (!update_object_collision(object_pose)) {
      RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "Could not add the randomized object to MoveIt" ANSI_COLOR_RESET);
      return false;
    }
    target_log("measured object", object_pose);

    auto pick_contact = tcp_pose_for_object(object_pose);
    auto pick_approach = pick_contact;
    pick_approach.pose.position.z += approach_height_m_;
    if (!plan_and_execute(move_group, pick_approach, "pick approach")) {
      return false;
    }
    if (!remove_object_collision()) {
      RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "Could not open the grasp collision allowance" ANSI_COLOR_RESET);
      return false;
    }
    if (!cartesian_execute(move_group, pick_contact, "pick contact")) {
      return false;
    }

    publish_status("Closing the Robotiq 2F-140 around the object");
    if (!command_gripper(true)) {
      command_gripper(false);
      return false;
    }
    interruptible_sleep(dwell_time_s_);
    if (!update_object_collision(latest_object_pose())) {
      RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "Could not restore the grasped object collision" ANSI_COLOR_RESET);
      command_gripper(false);
      return false;
    }
    if (!attach_object_in_moveit(move_group)) {
      command_gripper(false);
      return false;
    }

    auto pick_retreat = pick_contact;
    pick_retreat.pose.position.z += approach_height_m_;
    if (!cartesian_execute(move_group, pick_retreat, "pick retreat")) {
      return false;
    }

    std::uniform_int_distribution<std::size_t> select_bin(0, bin_centers_.size() - 1);
    const std::size_t bin_index = select_bin(random_engine_);
    const Bin & selected_bin = bin_centers_[bin_index];
    std_msgs::msg::Int32 bin_message;
    bin_message.data = static_cast<int32_t>(bin_index);
    selected_bin_publisher_->publish(bin_message);
    publish_status(ANSI_COLOR_INFO "Selected bin " + std::to_string(bin_index) + ANSI_COLOR_RESET);

    geometry_msgs::msg::PoseStamped drop_object_pose;
    drop_object_pose.header.frame_id = move_group.getPlanningFrame();
    drop_object_pose.header.stamp = node_->now();
    drop_object_pose.pose.position.x = selected_bin.center[0];
    drop_object_pose.pose.position.y = selected_bin.center[1];
    drop_object_pose.pose.position.z = selected_bin.center[2] + bin_drop_height_;
    drop_object_pose.pose.orientation.w = 1.0;
    auto place_contact = tcp_pose_for_object(drop_object_pose);
    auto place_approach = place_contact;
    place_approach.pose.position.z += approach_height_m_;
    if (!plan_and_execute(move_group, place_approach, "place approach") ||
      !cartesian_execute(move_group, place_contact, "place contact"))
    {
      return false;
    }

    publish_status("Opening the Robotiq 2F-140 over bin " + std::to_string(bin_index));
    if (!command_gripper(false) || !detach_object_in_moveit(move_group)) {
      return false;
    }
    interruptible_sleep(dwell_time_s_);
    if (!update_object_collision(latest_object_pose())) {
      RCLCPP_ERROR(node_->get_logger(), ANSI_COLOR_ERROR "Could not update the released object collision" ANSI_COLOR_RESET);
      return false;
    }

    auto place_retreat = place_contact;
    place_retreat.pose.position.z += approach_height_m_;
    if (!cartesian_execute(move_group, place_retreat, "place retreat")) {
      return false;
    }

    if (!wait_until(
        [this, &selected_bin]() {return object_is_in_bin(selected_bin);},
        placement_timeout_s_))
    {
      RCLCPP_ERROR(
        node_->get_logger(), ANSI_COLOR_ERROR "The measured object pose did not settle inside bin %zu" ANSI_COLOR_RESET,
        bin_index);
      return false;
    }

    publish_status(
      "Completed cycle " + std::to_string(cycle_number) +
      " in bin " + std::to_string(bin_index));
    return true;
  }

  void recover_from_failure(
    moveit::planning_interface::MoveGroupInterface & move_group)
  {
    std_msgs::msg::Bool open_message;
    open_message.data = false;
    gripper_publisher_->publish(open_message);
    wait_until(
      [this]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return have_grasp_state_ && !grasp_attached_;
      }, std::min(2.0, scene_timeout_s_));
    if (!detach_object_in_moveit(move_group)) {
      RCLCPP_WARN(node_->get_logger(), ANSI_COLOR_ERROR "Failure recovery could not detach the MoveIt object" ANSI_COLOR_RESET);
    }
  }

  void publish_status(const std::string & status)
  {
    std_msgs::msg::String message;
    message.data = status;
    status_publisher_->publish(message);
    RCLCPP_INFO(node_->get_logger(), "%s", status.c_str());
  }

  void target_log(
    const std::string & phase,
    const geometry_msgs::msg::PoseStamped & target) const
  {
    RCLCPP_INFO(
      node_->get_logger(), "%s in %s: [%.3f, %.3f, %.3f]",
      phase.c_str(), target.header.frame_id.c_str(),
      target.pose.position.x, target.pose.position.y, target.pose.position.z);
  }

  void interruptible_sleep(double duration_s) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(0.0, duration_s)));
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  std::string end_effector_link_;
  std::string gripper_mount_link_;
  std::string planner_id_;
  double planning_time_s_{8.0};
  int planning_attempts_{8};
  int planning_retries_{4};
  double velocity_scaling_{0.2};
  double acceleration_scaling_{0.2};
  double position_tolerance_m_{0.01};
  double orientation_tolerance_rad_{0.05};
  double approach_height_m_{0.18};
  double cartesian_step_m_{0.005};
  double dwell_time_s_{1.0};
  double retry_delay_s_{2.0};
  double scene_timeout_s_{8.0};
  double placement_timeout_s_{4.0};
  int max_cycles_{0};

  std::string joint_state_topic_;
  double joint_state_timeout_s_{1.0};
  int minimum_state_samples_{3};
  std::vector<std::string> joint_names_;
  geometry_msgs::msg::Quaternion grasp_orientation_;
  Vector3 object_to_tcp_offset_{};
  std::string object_id_;
  Vector3 object_size_{};
  std::vector<Bin> bin_centers_;
  Vector3 bin_outer_size_{};
  double bin_wall_thickness_{0.015};
  double bin_floor_thickness_{0.02};
  double bin_drop_height_{0.22};
  Vector3 obstacle_position_{};
  Vector3 obstacle_size_{};
  Vector3 ground_size_{};
  Vector3 gripper_body_size_{};
  Vector3 gripper_body_position_{};
  Vector3 gripper_finger_size_{};
  std::vector<double> gripper_finger_positions_;
  std::vector<std::string> touch_links_;

  std::string object_pose_topic_;
  std::string object_generation_topic_;
  std::string reset_object_topic_;
  std::string gripper_command_topic_;
  std::string grasp_state_topic_;
  std::mt19937 random_engine_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  bool planning_object_attached_{false};
  uint32_t last_used_object_generation_{0};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_object_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr selected_bin_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr object_generation_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp_state_subscription_;

  mutable std::mutex state_mutex_;
  std::chrono::steady_clock::time_point last_joint_state_time_{};
  int valid_joint_state_samples_{0};
  geometry_msgs::msg::PoseStamped object_pose_;
  uint32_t object_generation_{0};
  bool have_object_pose_{false};
  bool have_object_generation_{false};
  bool have_grasp_state_{false};
  bool grasp_attached_{false};
  std::chrono::steady_clock::time_point object_pose_received_at_{};
  std::chrono::steady_clock::time_point object_generation_received_at_{};
};

}  // namespace manipulator_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("random_pick_place_planner", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int exit_code = 0;
  try {
    manipulator_sim::RandomPickPlacePlanner planner(node);
    planner.run();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), ANSI_COLOR_ERROR "Planner terminated: %s" ANSI_COLOR_RESET, exception.what());
    exit_code = 1;
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
