#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <string>
#include <vector>
#include <thread>

class TrjController : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  TrjController() : Node("trajectory_controller") {
    // Initialize joint publishers
    std::vector<std::string> joint_names = {
      "shoulder_pan_joint_cmd",
      "shoulder_lift_joint_cmd",
      "elbow_joint_cmd",
      "wrist_1_joint_cmd",
      "wrist_2_joint_cmd",
      "wrist_3_joint_cmd"
    };

    for (const auto & joint_name : joint_names) {
      auto pub = this->create_publisher<std_msgs::msg::Float64>(joint_name, 10);
      joint_publishers_.push_back(pub);
    }

    // Initialize clock subscription
    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "clock", 10, std::bind(&TrjController::clock_callback, this, std::placeholders::_1));

    // Initialize action server
    trj_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "scaled_joint_trajectory_controller/follow_joint_trajectory",
      std::bind(&TrjController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrjController::handle_cancel, this, std::placeholders::_1),
      std::bind(&TrjController::handle_accepted, this, std::placeholders::_1));
  }

private:
  void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    sim_time_ns = msg->clock.sec * 1e9 + msg->clock.nanosec;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with trajectory");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&TrjController::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    rclcpp::Rate loop_rate(100);  // Control loop rate in Hz
    size_t point_index = 0;
    auto start_time = this->now();

    while (rclcpp::ok()) {
      auto current_time = this->now();
      auto elapsed_time = current_time - start_time;

      // Check if we've reached the end of the trajectory
      if (point_index >= goal->trajectory.points.size()) {
        break;
      }

      auto & current_point = goal->trajectory.points[point_index];
      auto point_duration = rclcpp::Duration(current_point.time_from_start.sec, current_point.time_from_start.nanosec);

      if (elapsed_time >= point_duration) {
        for (size_t i = 0; i < joint_publishers_.size(); ++i) {
          std_msgs::msg::Float64 position_msg;
          position_msg.data = current_point.positions[i];
          joint_publishers_[i]->publish(position_msg);
        }
        point_index++;
      }

      // Check for goal cancellation
      if (goal_handle->is_canceling()) {
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      loop_rate.sleep();
    }

    result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

  int64_t sim_time_ns;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_publishers_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr trj_server_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrjController>());
  rclcpp::shutdown();
  return 0;
}
