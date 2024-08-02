
#include <control_msgs/action/detail/follow_joint_trajectory__struct.hpp>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rosgraph_msgs/msg/detail/clock__struct.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <string>
#include <vector>

class TrjController : public rclcpp::Node{

public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;


  TrjController() : Node("trajectory_controller"){
    
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
    
    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&TrjController::clock_callback, this, std::placeholders::_1));

    trj_server_ = rclcpp_action::create_server<FollowJointTrajectory>(this, "some_trj_service_topic", std::bind(&TrjController);

  }
    


private:
  
  void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg){
    sim_time_ns = msg->clock.nanosec;
  }


  float sim_time_ns; 

  

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJointTrajectory> goal) { 
    RCLCPP_INFO(this->get_logger(), "recived goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }



  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_publishers_;
    
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;

};


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrjController>());
  rclcpp::shutdown();
  return 0;
}
