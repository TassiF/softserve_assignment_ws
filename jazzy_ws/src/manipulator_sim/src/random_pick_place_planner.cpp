#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace manipulator_sim
{

class RandomPickPlacePlanner
{
public:
  explicit RandomPickPlacePlanner(const rclcpp::Node::SharedPtr & node)
  : node_(node), random_engine_(std::random_device{}())
  {
    planning_group_ = parameter<std::string>("planning_group", "ur_manipulator");
    end_effector_link_ = parameter<std::string>("end_effector_link", "tool0");
    planner_id_ = parameter<std::string>("planner_id", "RRTConnectkConfigDefault");
    planning_time_s_ = parameter<double>("planning_time", 5.0);
    planning_attempts_ = static_cast<int>(
      parameter<int64_t>("planning_attempts", 5));
    random_samples_ = static_cast<int>(
      parameter<int64_t>("random_samples", 10));
    velocity_scaling_ = parameter<double>("velocity_scaling", 0.25);
    acceleration_scaling_ = parameter<double>("acceleration_scaling", 0.25);
    position_tolerance_m_ = parameter<double>("position_tolerance", 0.01);
    orientation_tolerance_rad_ = parameter<double>("orientation_tolerance", 0.05);
    approach_height_m_ = parameter<double>("approach_height", 0.15);
    dwell_time_s_ = parameter<double>("dwell_time", 1.0);
    retry_delay_s_ = parameter<double>("retry_delay", 2.0);
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

    pick_bounds_ = read_bounds(
      "pick_min", "pick_max",
      {0.35, -0.70, 0.20}, {0.95, -0.15, 0.65});
    place_bounds_ = read_bounds(
      "place_min", "place_max",
      {0.35, 0.15, 0.20}, {0.95, 0.70, 0.65});

    if (seed >= 0) {
      random_engine_.seed(static_cast<std::mt19937::result_type>(seed));
    }
    if (planning_time_s_ <= 0.0 || planning_attempts_ <= 0 || random_samples_ <= 0) {
      throw std::invalid_argument(
              "planning_time, planning_attempts, and random_samples must be positive");
    }
    if (approach_height_m_ <= 0.0) {
      throw std::invalid_argument("approach_height must be positive");
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

    goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pick_place/goal_pose", rclcpp::QoS(10).reliable());
    gripper_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/pick_place/gripper_close", rclcpp::QoS(10).reliable());
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/pick_place/status", rclcpp::QoS(10).transient_local().reliable());
    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr message) {
        joint_state_callback(message);
      });
  }

  void run()
  {
    publish_status("Waiting for MoveIt and Isaac joint-state feedback");
    moveit::planning_interface::MoveGroupInterface move_group(
      node_, planning_group_);
    move_group.setEndEffectorLink(end_effector_link_);
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId(planner_id_);
    move_group.setPlanningTime(planning_time_s_);
    move_group.setNumPlanningAttempts(planning_attempts_);
    move_group.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group.setGoalPositionTolerance(position_tolerance_m_);
    move_group.setGoalOrientationTolerance(orientation_tolerance_rad_);
    move_group.allowReplanning(false);

    move_group.startStateMonitor();
    while (rclcpp::ok()) {
      if (move_group.getCurrentState(1.0) && has_fresh_joint_state_stream()) {
        break;
      }
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Waiting for complete /joint_states from the Isaac controller");
    }
    if (!rclcpp::ok()) {
      return;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Random pick/place planner ready (group=%s, end_effector=%s, frame=%s)",
      planning_group_.c_str(), end_effector_link_.c_str(),
      move_group.getPlanningFrame().c_str());

    int completed_cycles = 0;
    while (rclcpp::ok() && (max_cycles_ <= 0 || completed_cycles < max_cycles_)) {
      if (execute_cycle(move_group, completed_cycles + 1)) {
        ++completed_cycles;
        RCLCPP_INFO(node_->get_logger(), "Completed pick/place cycle %d", completed_cycles);
      } else if (rclcpp::ok()) {
        publish_status("Cycle failed; resampling after retry delay");
        interruptible_sleep(retry_delay_s_);
      }
    }

    publish_status("Pick/place planner stopped");
  }

private:
  struct Bounds
  {
    std::array<double, 3> minimum;
    std::array<double, 3> maximum;
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
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (valid_joint_state_samples_ > 0 &&
      std::chrono::duration<double>(received_at - last_joint_state_time_).count() >
      joint_state_timeout_s_)
    {
      valid_joint_state_samples_ = 0;
    }
    ++valid_joint_state_samples_;
    last_joint_state_time_ = received_at;
  }

  bool has_fresh_joint_state_stream() const
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (valid_joint_state_samples_ < minimum_state_samples_) {
      return false;
    }
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_joint_state_time_).count() <=
           joint_state_timeout_s_;
  }

  Bounds read_bounds(
    const std::string & minimum_name,
    const std::string & maximum_name,
    const std::vector<double> & default_minimum,
    const std::vector<double> & default_maximum)
  {
    const auto minimum = parameter<std::vector<double>>(minimum_name, default_minimum);
    const auto maximum = parameter<std::vector<double>>(maximum_name, default_maximum);
    if (minimum.size() != 3 || maximum.size() != 3) {
      throw std::invalid_argument(minimum_name + " and " + maximum_name + " must have three values");
    }

    Bounds bounds{
      {minimum[0], minimum[1], minimum[2]},
      {maximum[0], maximum[1], maximum[2]}};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(bounds.minimum[axis]) ||
        !std::isfinite(bounds.maximum[axis]) ||
        bounds.minimum[axis] >= bounds.maximum[axis])
      {
        throw std::invalid_argument(
                minimum_name + " must be finite and smaller than " + maximum_name);
      }
    }
    return bounds;
  }

  geometry_msgs::msg::PoseStamped sample_pose(
    const Bounds & bounds,
    const geometry_msgs::msg::Quaternion & orientation,
    const std::string & frame)
  {
    std::uniform_real_distribution<double> x(bounds.minimum[0], bounds.maximum[0]);
    std::uniform_real_distribution<double> y(bounds.minimum[1], bounds.maximum[1]);
    std::uniform_real_distribution<double> z(bounds.minimum[2], bounds.maximum[2]);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = frame;
    pose.pose.position.x = x(random_engine_);
    pose.pose.position.y = y(random_engine_);
    pose.pose.position.z = z(random_engine_);
    pose.pose.orientation = orientation;
    return pose;
  }

  bool plan_and_execute(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const geometry_msgs::msg::PoseStamped & target,
    const std::string & phase)
  {
    target_log(phase, target);
    goal_publisher_->publish(target);
    publish_status("Planning " + phase);

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target, end_effector_link_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group.plan(plan);
    move_group.clearPoseTargets();
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(node_->get_logger(), "Planning failed for phase '%s'", phase.c_str());
      return false;
    }

    publish_status("Executing " + phase);
    const auto execution_result = move_group.execute(plan);
    if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Execution failed for phase '%s'", phase.c_str());
      return false;
    }
    return true;
  }

  std::optional<geometry_msgs::msg::PoseStamped> reach_random_contact(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const Bounds & bounds,
    const std::string & operation)
  {
    for (int sample = 1; sample <= random_samples_ && rclcpp::ok(); ++sample) {
      const auto current = move_group.getCurrentPose(end_effector_link_);
      auto contact = sample_pose(
        bounds, current.pose.orientation, move_group.getPlanningFrame());
      auto approach = contact;
      approach.pose.position.z += approach_height_m_;

      RCLCPP_INFO(
        node_->get_logger(), "Trying random %s sample %d/%d",
        operation.c_str(), sample, random_samples_);
      if (!plan_and_execute(move_group, approach, operation + " approach")) {
        continue;
      }
      if (plan_and_execute(move_group, contact, operation + " contact")) {
        return contact;
      }
    }

    RCLCPP_ERROR(
      node_->get_logger(), "No executable random %s target found after %d samples",
      operation.c_str(), random_samples_);
    return std::nullopt;
  }

  bool execute_cycle(moveit::planning_interface::MoveGroupInterface & move_group, int cycle_number)
  {
    publish_status("Starting cycle " + std::to_string(cycle_number));

    const auto pick = reach_random_contact(move_group, pick_bounds_, "pick");
    if (!pick) {
      return false;
    }
    publish_gripper(true);
    publish_status("Pick contact reached; closing gripper interface");
    interruptible_sleep(dwell_time_s_);

    auto pick_retreat = *pick;
    pick_retreat.pose.position.z += approach_height_m_;
    if (!plan_and_execute(move_group, pick_retreat, "pick retreat")) {
      publish_gripper(false);
      publish_status("Pick retreat failed; opening gripper interface");
      return false;
    }

    const auto place = reach_random_contact(move_group, place_bounds_, "place");
    if (!place) {
      publish_gripper(false);
      publish_status("No place target found; opening gripper interface");
      return false;
    }
    publish_gripper(false);
    publish_status("Place contact reached; opening gripper interface");
    interruptible_sleep(dwell_time_s_);

    auto place_retreat = *place;
    place_retreat.pose.position.z += approach_height_m_;
    if (!plan_and_execute(move_group, place_retreat, "place retreat")) {
      return false;
    }

    publish_status("Completed cycle " + std::to_string(cycle_number));
    return true;
  }

  void publish_gripper(bool close)
  {
    std_msgs::msg::Bool message;
    message.data = close;
    gripper_publisher_->publish(message);
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
      node_->get_logger(), "%s target in %s: [%.3f, %.3f, %.3f]",
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
  std::string planner_id_;
  double planning_time_s_{5.0};
  int planning_attempts_{5};
  int random_samples_{10};
  double velocity_scaling_{0.25};
  double acceleration_scaling_{0.25};
  double position_tolerance_m_{0.01};
  double orientation_tolerance_rad_{0.05};
  double approach_height_m_{0.15};
  double dwell_time_s_{1.0};
  double retry_delay_s_{2.0};
  int max_cycles_{0};
  std::string joint_state_topic_;
  double joint_state_timeout_s_{1.0};
  int minimum_state_samples_{3};
  std::vector<std::string> joint_names_;
  Bounds pick_bounds_{};
  Bounds place_bounds_{};
  std::mt19937 random_engine_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;
  mutable std::mutex joint_state_mutex_;
  std::chrono::steady_clock::time_point last_joint_state_time_{};
  int valid_joint_state_samples_{0};
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
    RCLCPP_FATAL(node->get_logger(), "Planner terminated: %s", exception.what());
    exit_code = 1;
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
