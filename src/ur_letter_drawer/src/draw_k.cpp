#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
constexpr char kPlanningGroup[] = "ur_manipulator";
constexpr char kPlanningFrame[] = "base_link";
constexpr double kCartesianStep = 0.002;
constexpr double kJumpThreshold = 0.0;  // Avoid false rejection near the wrist singularity.
constexpr double kFullPathFraction = 0.999;
constexpr double kEefLift = 0.04;

struct Stroke
{
  std::vector<std::pair<double, double>> points;
};

// Capital K in base_link's XY plane: 14 x 20 cm, visible and within UR3 reach.
std::vector<Stroke> makeLetterK()
{
  constexpr double x = 0.18;
  constexpr double y = -0.08;
  constexpr double width = 0.12;
  constexpr double height = 0.18;
  const double middle_y = y + height / 2.0;

  return {
    {{{x, y}, {x, y + height}}},
    {{{x, middle_y}, {x + width, y + height}}},
    {{{x, middle_y}, {x + width, y}}},
  };
}

geometry_msgs::msg::Pose makePose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  // Tool vertical, pointing down at the horizontal writing plane.
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  return pose;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

void publishLetterPath(
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
  const rclcpp::Clock::SharedPtr & clock,
  const std::vector<Stroke> & strokes,
  double z)
{
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;
  for (const auto & stroke : strokes) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = kPlanningFrame;
    marker.header.stamp = clock->now();
    marker.ns = "letter_K";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.008;
    marker.color.r = 0.10F;
    marker.color.g = 0.90F;
    marker.color.b = 1.00F;
    marker.color.a = 1.00F;
    for (const auto & point : stroke.points) {
      marker.points.push_back(makePoint(point.first, point.second, z));
    }
    markers.markers.push_back(std::move(marker));
  }
  publisher->publish(markers);
}

bool executeCartesian(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  const std::string & description,
  const rclcpp::Logger & logger)
{
  if (waypoints.empty()) {
    return true;
  }
  move_group.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = move_group.computeCartesianPath(
    waypoints, kCartesianStep, kJumpThreshold, trajectory, true);
  RCLCPP_INFO(logger, "Cartesian path '%s': %.1f%%", description.c_str(), fraction * 100.0);
  if (fraction < kFullPathFraction) {
    RCLCPP_ERROR(
      logger,
      "Rejecting incomplete Cartesian path '%s' (%.1f%%); no partial trajectory will be executed",
      description.c_str(), fraction * 100.0);
    return false;
  }
  const auto execute_result = move_group.execute(trajectory);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "MoveIt failed to execute Cartesian path '%s'", description.c_str());
    return false;
  }
  return true;
}

bool drawLetterK(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::vector<Stroke> & strokes,
  double z,
  const rclcpp::Logger & logger)
{
  for (std::size_t stroke_index = 0; stroke_index < strokes.size(); ++stroke_index) {
    const auto & stroke = strokes[stroke_index];
    const auto & first = stroke.points.front();
    const auto & last = stroke.points.back();
    const std::string stroke_name = "K stroke " + std::to_string(stroke_index + 1);

    const auto raised_start = makePose(first.first, first.second, z + kEefLift);
    // Follow a collision-checked Cartesian path from the current state to keep the IK branch local.
    if (!executeCartesian(
        move_group, {raised_start}, "raised travel to " + stroke_name, logger))
    {
      return false;
    }
    // Lower, draw, then lift with collision-checked Cartesian paths.
    if (!executeCartesian(
        move_group, {makePose(first.first, first.second, z)},
        "lower to " + stroke_name, logger))
    {
      return false;
    }

    std::vector<geometry_msgs::msg::Pose> drawing_waypoints;
    drawing_waypoints.reserve(stroke.points.size() - 1);
    for (std::size_t point_index = 1; point_index < stroke.points.size(); ++point_index) {
      const auto & point = stroke.points[point_index];
      drawing_waypoints.push_back(makePose(point.first, point.second, z));
    }
    if (!executeCartesian(move_group, drawing_waypoints, stroke_name, logger)) {
      return false;
    }
    if (!executeCartesian(
        move_group, {makePose(last.first, last.second, z + kEefLift)},
        "lift after " + stroke_name, logger))
    {
      return false;
    }
  }
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
    "ur_letter_drawer",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto logger = node->get_logger();
  if (!node->has_parameter("writing_plane_z")) {
    node->declare_parameter("writing_plane_z", 0.34);
  }
  const double writing_plane_z = node->get_parameter("writing_plane_z").as_double();

  // MoveGroupInterface depends on ROS callbacks for state, planning and execution.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});
  int exit_code = 1;

  try {
    {
      moveit::planning_interface::MoveGroupInterface move_group(node, kPlanningGroup);
      move_group.setPoseReferenceFrame(kPlanningFrame);
      move_group.setEndEffectorLink("tool0");
      move_group.setPlanningTime(8.0);
      move_group.setNumPlanningAttempts(10);
      move_group.setGoalPositionTolerance(0.005);
      move_group.setGoalOrientationTolerance(0.05);
      move_group.setMaxVelocityScalingFactor(0.05);
      move_group.setMaxAccelerationScalingFactor(0.05);

      if (!move_group.startStateMonitor(10.0) || !move_group.getCurrentState(10.0)) {
        RCLCPP_ERROR(logger, "Timed out waiting for the simulated robot state");
      } else {
        const auto strokes = makeLetterK();
        auto marker_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/letter_path", rclcpp::QoS(1).reliable().transient_local());
        publishLetterPath(marker_publisher, node->get_clock(), strokes, writing_plane_z);
        RCLCPP_INFO(
          logger,
          "Drawing K in base_link XY plane: width 0.12 m, height 0.18 m, z=%.2f m",
          writing_plane_z);

        const bool success = drawLetterK(move_group, strokes, writing_plane_z, logger);
        if (success) {
          RCLCPP_INFO(logger, "Finished drawing K");
          exit_code = 0;
        } else {
          move_group.stop();
          RCLCPP_ERROR(logger, "Drawing stopped after a planning or execution failure");
        }
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger, "Drawing node failed: %s", exception.what());
  }

  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
