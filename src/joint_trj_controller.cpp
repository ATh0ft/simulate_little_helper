#include <control_msgs/action/detail/follow_joint_trajectory__struct.hpp>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rosgraph_msgs/msg/detail/clock__struct.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/float64.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <string>
#include <vector>

class TrjController : public rclcpp::Node{
  public:

  TrjController() : Node("trajectory_controller") {

    // make a vector containing the joint names 
    std::vector<std::string> joint_names = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };

    // itterate over the joints to generate a publisher for each joint 

    for (const std::string & joint_name : joint_names) {
      auto publisher = this->create_publisher<std_msgs::msg::Float64>(joint_name, 10);
      joint_publishers_.push_back(publisher);
    }

    trj_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      this,
      "scaled_joint_trajectory_controller/follow_joint_trajectory",
      std::bind(&TrjController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrjController::handle_cancel, this, std::placeholders::_1),
      std::bind(&TrjController::handle_accepted, this, std::placeholders::_1)
    );



  }
  private:

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid, 
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal
  )
  {
    RCLCPP_INFO(this->get_logger(), "recived a goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
  )
  {
    RCLCPP_INFO(this->get_logger(), "recived a request to cancel the goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
  )
  {
    std::thread{std::bind(&TrjController::execute_trj, this, std::placeholders::_1), goal_handle}.detach(); 
  }


  void execute_trj(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
  )
  {
    RCLCPP_INFO(this->get_logger(), "executing trj"); 
    const auto goal = goal_handle->get_goal(); 
    
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

    rclcpp::Rate loop_rate(100); 

    uint64_t point_index = 0;

    auto start_time = this->now(); 

    while (rclcpp::ok()) {
      auto current_time = this->now();
      auto elapsed_time = current_time - start_time; 

      if (point_index >= goal->trajectory.points.size()){
        break; //the end have been reached
      }
      auto & current_point = goal->trajectory.points[point_index];
      auto point_duration = rclcpp::Duration(current_point.time_from_start.sec, current_point.time_from_start.nanosec);

      // publish the current point when it is within the time frame of that point
      if (elapsed_time >= point_duration) { 
        for (uint i = 0; i < joint_publishers_.size(); ++i) {
          std_msgs::msg::Float64 position_msg; 
          position_msg.data = current_point.positions[i];
          joint_publishers_[i]->publish(position_msg);
        }
        point_index++; 
      }
      
      // condition if the goal is canceled 
      if (goal_handle->is_canceling()) {
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "goal was canceled"); 
        return; 

      }
      loop_rate.sleep(); 
    }
    result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "goal succeed!");
  }

  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_publishers_;
  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr trj_server_;
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrjController>());
  rclcpp::shutdown();
  return 0;
}
