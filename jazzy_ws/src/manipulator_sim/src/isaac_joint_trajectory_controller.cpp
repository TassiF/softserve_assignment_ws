// Developer: Francesco Tassi
// Email: francesco.tassi@iit.it
//
// Joint trajectory controller for a robotic manipulator in IsaacSim. The controller receives joint trajectory goals from MoveIt and publishes joint commands to the simulator. 
// It subscribes to the joint states published by the simulator to monitor execution progress and provide feedback to MoveIt.
// 
// Topics:
//  - /isaac_joint_commands (sensor_msgs/JointState): publishes the joint commands to the simulator
//  - /joint_states (sensor_msgs/JointState): publishes the joint states to MoveIt
// Subscriptions:
//  - /isaac_joint_states (sensor_msgs/JointState): subscribes to the joint states published by the simulator

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#define ANSI_COLOR_ERROR    "\x1b[31m" //red
#define ANSI_COLOR_INFO   "\x1b[32m" //green
#define ANSI_COLOR_WARN  "\x1b[33m"  //yellow
#define ANSI_COLOR_RESET   "\x1b[0m"  //reset

namespace manipulator_sim
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class IsaacJointTrajectoryController : public rclcpp::Node
{
public:
  explicit IsaacJointTrajectoryController(const rclcpp::NodeOptions & options)
  : Node("isaac_joint_trajectory_controller", options)
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
      });
    state_topic_ = declare_parameter<std::string>("state_topic", "/isaac_joint_states");
    command_topic_ = declare_parameter<std::string>("command_topic", "/isaac_joint_commands");
    action_name_ = declare_parameter<std::string>("action_name", "/isaac_joint_trajectory_controller/follow_joint_trajectory");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 100.0);
    state_timeout_s_ = declare_parameter<double>("state_timeout", 1.0);
    path_tolerance_rad_ = declare_parameter<double>("path_tolerance", 0.5);
    goal_tolerance_rad_ = declare_parameter<double>("goal_tolerance", 0.02);
    goal_time_tolerance_s_ = declare_parameter<double>("goal_time_tolerance", 3.0);
    stable_samples_required_ = declare_parameter<int>("stable_samples", 5);

    if (joint_names_.empty()) {
      throw std::invalid_argument("joint_names must not be empty");
    }
    if (control_rate_hz_ <= 0.0) {
      throw std::invalid_argument("control_rate_hz must be positive");
    }
    if (state_timeout_s_ <= 0.0) {
      throw std::invalid_argument("state_timeout must be positive");
    }
    if (path_tolerance_rad_ <= 0.0 || goal_tolerance_rad_ <= 0.0 || goal_time_tolerance_s_ <= 0.0 || stable_samples_required_ <= 0) {
      throw std::invalid_argument(
              "trajectory tolerances and stable_samples must be positive");
    }

    command_publisher_ = create_publisher<sensor_msgs::msg::JointState>(command_topic_, rclcpp::QoS(10).reliable());
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10).reliable());
    state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(state_topic_, rclcpp::SensorDataQoS(), std::bind(&IsaacJointTrajectoryController::joint_state_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(this, action_name_,
      std::bind(
        &IsaacJointTrajectoryController::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &IsaacJointTrajectoryController::handle_cancel, this,
        std::placeholders::_1),
      std::bind(
        &IsaacJointTrajectoryController::handle_accepted, this,
        std::placeholders::_1));

    RCLCPP_INFO(get_logger(), ANSI_COLOR_INFO "Isaac trajectory controller ready: %s -> %s (feedback: %s)" ANSI_COLOR_RESET, action_name_.c_str(), command_topic_.c_str(), state_topic_.c_str());
  }

  ~IsaacJointTrajectoryController() override
  {
    std::lock_guard<std::mutex> lock(execution_thread_mutex_);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

private:
  static double duration_seconds(const builtin_interfaces::msg::Duration & duration) {
    return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
  }

  static builtin_interfaces::msg::Duration seconds_to_duration(double seconds) {
    seconds = std::max(0.0, seconds);
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(std::floor(seconds));
    duration.nanosec = static_cast<uint32_t>(std::round((seconds - static_cast<double>(duration.sec)) * 1.0e9));
    if (duration.nanosec >= 1000000000U) {
      ++duration.sec;
      duration.nanosec -= 1000000000U;
    }
    return duration;
  }

  static double position_error(double desired, double actual) {
    return desired - actual;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message) {
    if (message->name.size() != message->position.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, ANSI_COLOR_WARN "Ignoring malformed Isaac JointState: %zu names, %zu positions" ANSI_COLOR_RESET, message->name.size(), message->position.size());
      return;
    }
    std::unordered_map<std::string, double> positions;
    positions.reserve(message->name.size());
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      if (std::isfinite(message->position[index])) {
        positions[message->name[index]] = message->position[index];
      }
    }
    for (const auto & joint_name : joint_names_) {
      if (positions.find(joint_name) == positions.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, ANSI_COLOR_WARN "Isaac JointState does not yet contain required joint '%s'" ANSI_COLOR_RESET, joint_name.c_str());
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_positions_ = std::move(positions);
      latest_state_time_ = std::chrono::steady_clock::now();
      have_state_ = true;
    }

    // MoveIt and robot_state_publisher use the conventional topic. The Isaac-specific topic remains the source.
    auto forwarded = *message;
    if (forwarded.header.stamp.sec == 0 && forwarded.header.stamp.nanosec == 0) {
      forwarded.header.stamp = now();
    }
    joint_state_publisher_->publish(forwarded);
  }

  bool read_positions(const std::vector<std::string> & names, std::vector<double> & positions, double & state_age_s) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_state_) {
      return false;
    }
    state_age_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_state_time_).count();
    positions.clear();
    positions.reserve(names.size());
    for (const auto & name : names) {
      const auto iterator = latest_positions_.find(name);
      if (iterator == latest_positions_.end()) {
        return false;
      }
      positions.push_back(iterator->second);
    }
    return true;
  }

  bool validate_goal(const FollowJointTrajectory::Goal & goal, std::string & reason) const {
    const auto & trajectory = goal.trajectory;
    if (trajectory.joint_names.empty()) {
      reason = "trajectory has no joint names";
      return false;
    }
    if (trajectory.points.empty()) {
      reason = "trajectory has no points";
      return false;
    }

    std::vector<std::string> expected = joint_names_;
    std::vector<std::string> received = trajectory.joint_names;
    std::sort(expected.begin(), expected.end());
    std::sort(received.begin(), received.end());
    if (received != expected) {
      reason = "trajectory joint set does not match the configured UR joints";
      return false;
    }

    double previous_time = -1.0;
    for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
      const auto & point = trajectory.points[index];
      if (point.positions.size() != trajectory.joint_names.size()) {
        reason = "point " + std::to_string(index) + " has an invalid position vector";
        return false;
      }
      if (!point.velocities.empty() && point.velocities.size() != trajectory.joint_names.size()) {
        reason = "point " + std::to_string(index) + " has an invalid velocity vector";
        return false;
      }
      if (!point.accelerations.empty() && point.accelerations.size() != trajectory.joint_names.size()) {
        reason = "point " + std::to_string(index) + " has an invalid acceleration vector";
        return false;
      }
      if (!point.effort.empty() && point.effort.size() != trajectory.joint_names.size()) {
        reason = "point " + std::to_string(index) + " has an invalid effort vector";
        return false;
      }
      const double point_time = duration_seconds(point.time_from_start);
      if (point_time < 0.0 || point_time < previous_time) {
        reason = "trajectory times are not monotonic";
        return false;
      }
      const std::array<std::pair<const std::vector<double> *, const char *>, 4> fields{{
        {&point.positions, "position"},
        {&point.velocities, "velocity"},
        {&point.accelerations, "acceleration"},
        {&point.effort, "effort"},
      }};
      for (const auto & [values, field_name] : fields) {
        if (!std::all_of(
            values->begin(), values->end(),
            [](double value) {return std::isfinite(value);}))
        {
          reason = "trajectory contains a non-finite " + std::string(field_name);
          return false;
        }
      }
      previous_time = point_time;
    }

    const auto tolerances_are_valid = [&trajectory](const std::vector<control_msgs::msg::JointTolerance> & tolerances) {
        for (const auto & tolerance : tolerances) {
          if (std::find(
              trajectory.joint_names.begin(), trajectory.joint_names.end(),
              tolerance.name) == trajectory.joint_names.end() ||
            !std::isfinite(tolerance.position) ||
            !std::isfinite(tolerance.velocity) ||
            !std::isfinite(tolerance.acceleration)) {
            return false;
          }
        }
        return true;
      };
    if (!tolerances_are_valid(goal.path_tolerance) || !tolerances_are_valid(goal.goal_tolerance)) {
      reason = "trajectory contains an invalid joint tolerance";
      return false;
    }
    return true;
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
    std::string reason;
    if (!validate_goal(*goal, reason)) {
      RCLCPP_ERROR(get_logger(), ANSI_COLOR_ERROR "Rejecting trajectory: %s" ANSI_COLOR_RESET, reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    bool expected = false;
    if (!executing_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), ANSI_COLOR_WARN "Rejecting trajectory while another goal is active" ANSI_COLOR_RESET);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowJointTrajectory>) {
    RCLCPP_INFO(get_logger(), ANSI_COLOR_INFO "Trajectory cancellation requested" ANSI_COLOR_RESET);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
    std::lock_guard<std::mutex> lock(execution_thread_mutex_);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread(&IsaacJointTrajectoryController::execute, this, goal_handle);
  }

  std::vector<double> interpolate(const FollowJointTrajectory::Goal & goal, const std::vector<double> & initial_positions, double elapsed_s) const {
    const auto & points = goal.trajectory.points;
    const double first_time = duration_seconds(points.front().time_from_start);
    if (elapsed_s <= first_time) {
      if (first_time <= std::numeric_limits<double>::epsilon()) {
        return points.front().positions;
      }
      const double alpha = std::clamp(elapsed_s / first_time, 0.0, 1.0);
      std::vector<double> output(initial_positions.size());
      for (std::size_t index = 0; index < output.size(); ++index) {
        output[index] = initial_positions[index] + alpha * (points.front().positions[index] - initial_positions[index]);
      }
      return output;
    }

    for (std::size_t index = 1; index < points.size(); ++index) {
      const double segment_end = duration_seconds(points[index].time_from_start);
      if (elapsed_s <= segment_end) {
        const double segment_start = duration_seconds(points[index - 1].time_from_start);
        const double duration = segment_end - segment_start;
        const double alpha = duration <= std::numeric_limits<double>::epsilon() ? 1.0 : std::clamp((elapsed_s - segment_start) / duration, 0.0, 1.0);
        std::vector<double> output(initial_positions.size());
        for (std::size_t joint_index = 0; joint_index < output.size(); ++joint_index) {
          output[joint_index] = points[index - 1].positions[joint_index] + alpha * (points[index].positions[joint_index] - points[index - 1].positions[joint_index]);
        }
        return output;
      }
    }
    return points.back().positions;
  }

  std::vector<double> tolerances_for(const std::vector<control_msgs::msg::JointTolerance> & requested, double default_tolerance, const std::vector<std::string> & trajectory_joint_names) const {
    std::map<std::string, double> requested_by_name;
    for (const auto & tolerance : requested) {
      if (!tolerance.name.empty()) {
        requested_by_name[tolerance.name] = tolerance.position;
      }
    }
    std::vector<double> result;
    result.reserve(trajectory_joint_names.size());
    for (const auto & name : trajectory_joint_names) {
      const auto iterator = requested_by_name.find(name);
      if (iterator == requested_by_name.end() || iterator->second == 0.0) {
        result.push_back(default_tolerance);
      } else if (iterator->second < 0.0) {
        result.push_back(std::numeric_limits<double>::infinity());
      } else {
        result.push_back(iterator->second);
      }
    }
    return result;
  }

  void publish_command(const std::vector<std::string> & names, const std::vector<double> & positions) {
    sensor_msgs::msg::JointState command;
    command.header.stamp = now();
    command.name = names;
    command.position = positions;
    command_publisher_->publish(command);
  }

  void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
    struct ExecutionFlagReset {
      explicit ExecutionFlagReset(std::atomic<bool> & flag)
      : flag_(flag) {}
      ~ExecutionFlagReset() {flag_.store(false);}
      std::atomic<bool> & flag_;
    } reset_execution_flag(executing_);

    const auto goal = goal_handle->get_goal();
    const auto & names = goal->trajectory.joint_names;
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    std::vector<double> initial_positions;
    double state_age_s = 0.0;
    if (!read_positions(names, initial_positions, state_age_s) || state_age_s > state_timeout_s_) {
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      result->error_string = "No fresh, complete joint state is available from Isaac Sim";
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), ANSI_COLOR_ERROR "%s" ANSI_COLOR_RESET, result->error_string.c_str());
      return;
    }

    const auto path_tolerances = tolerances_for(goal->path_tolerance, path_tolerance_rad_, names);
    const auto goal_tolerances = tolerances_for(goal->goal_tolerance, goal_tolerance_rad_, names);
    const double requested_goal_time = duration_seconds(goal->goal_time_tolerance);
    const double goal_time_tolerance = requested_goal_time > 0.0 ? requested_goal_time : goal_time_tolerance_s_;
    const double final_trajectory_time = duration_seconds(goal->trajectory.points.back().time_from_start);

    RCLCPP_INFO(get_logger(), ANSI_COLOR_INFO "Executing %zu-point trajectory over %.3f seconds" ANSI_COLOR_RESET,goal->trajectory.points.size(), final_trajectory_time);

    const auto start = now();
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    auto next_update = std::chrono::steady_clock::now();
    int stable_samples = 0;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        std::vector<double> actual;
        if (read_positions(names, actual, state_age_s)) {
          publish_command(names, actual);
        }
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "Trajectory canceled; current position is being held";
        goal_handle->canceled(result);
        return;
      }

      const auto current_time = now();
      const double elapsed_s = std::max(0.0, (current_time - start).seconds());
      const auto desired = interpolate(*goal, initial_positions, elapsed_s);
      publish_command(names, desired);

      std::vector<double> actual;
      if (!read_positions(names, actual, state_age_s) || state_age_s > state_timeout_s_) {
        if (!actual.empty()) {
          publish_command(names, actual);
        }
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "Joint-state feedback from Isaac Sim became stale";
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), ANSI_COLOR_ERROR "%s" ANSI_COLOR_RESET, result->error_string.c_str());
        return;
      }

      std::vector<double> errors(actual.size());
      bool path_within_tolerance = true;
      bool goal_within_tolerance = true;
      for (std::size_t index = 0; index < actual.size(); ++index) {
        errors[index] = position_error(desired[index], actual[index]);
        path_within_tolerance = path_within_tolerance &&
          std::abs(errors[index]) <= path_tolerances[index];
        const double final_error = position_error(goal->trajectory.points.back().positions[index], actual[index]);
        goal_within_tolerance = goal_within_tolerance &&
          std::abs(final_error) <= goal_tolerances[index];
      }

      auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
      feedback->header.stamp = now();
      feedback->joint_names = names;
      feedback->desired.positions = desired;
      feedback->desired.time_from_start = seconds_to_duration(elapsed_s);
      feedback->actual.positions = actual;
      feedback->actual.time_from_start = seconds_to_duration(elapsed_s);
      feedback->error.positions = errors;
      feedback->error.time_from_start = seconds_to_duration(elapsed_s);
      goal_handle->publish_feedback(feedback);

      if (elapsed_s < final_trajectory_time && !path_within_tolerance) {
        publish_command(names, actual);
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "Isaac Sim tracking error exceeded the path tolerance";
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), ANSI_COLOR_ERROR "%s" ANSI_COLOR_RESET, result->error_string.c_str());
        return;
      }

      if (elapsed_s >= final_trajectory_time) {
        if (goal_within_tolerance) {
          ++stable_samples;
          if (stable_samples >= stable_samples_required_) {
            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            result->error_string = "Trajectory completed";
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), ANSI_COLOR_INFO "Trajectory completed successfully" ANSI_COLOR_RESET);
            return;
          }
        } else {
          stable_samples = 0;
        }

        if (elapsed_s > final_trajectory_time + goal_time_tolerance) {
          publish_command(names, actual);
          result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
          result->error_string = "Isaac Sim did not reach the final pose before the goal timeout";
          goal_handle->abort(result);
          RCLCPP_ERROR(get_logger(), ANSI_COLOR_ERROR "%s" ANSI_COLOR_RESET, result->error_string.c_str());
          return;
        }
      }

      next_update += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next_update);
    }

    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = "ROS shutdown interrupted trajectory execution";
    goal_handle->abort(result);
  }

  std::vector<std::string> joint_names_;
  std::string state_topic_;
  std::string command_topic_;
  std::string action_name_;
  double control_rate_hz_{100.0};
  double state_timeout_s_{1.0};
  double path_tolerance_rad_{0.5};
  double goal_tolerance_rad_{0.02};
  double goal_time_tolerance_s_{3.0};
  int stable_samples_required_{5};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  mutable std::mutex state_mutex_;
  std::unordered_map<std::string, double> latest_positions_;
  std::chrono::steady_clock::time_point latest_state_time_{};
  bool have_state_{false};
  std::atomic<bool> executing_{false};
  std::mutex execution_thread_mutex_;
  std::thread execution_thread_;
};

}  // namespace manipulator_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false);
  auto node = std::make_shared<manipulator_sim::IsaacJointTrajectoryController>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
