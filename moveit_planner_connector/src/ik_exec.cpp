#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <avoid_plan_msgs/srv/make_plan.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;
using namespace std::chrono_literals;

/*
The plan:
1. Take in a set of points, start and end of the 2d path
2. Hand the points to the RRT* planner system and get a path back
3. Feed it to the MoveIT IK system
4. Feed the MoveIT IK solution to the robot and execute it

working from these sources 
http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html#cartesian-paths
https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html
*/

class IKExecutorNode: public rclcpp::Node{
  public:
  IKExecutorNode(): Node("ik_executor"){
    // Declare planning group name parameter
    declare_parameter<std::string>("planning_group", "ur_manipulator");

    // Declare end effector link name
    declare_parameter<std::string>("end_link", "tool0");
    declare_parameter<std::string>("base_link", "base_link");

    // Declare start point parameters 


    // Declare end point parameters


    // Declare table offset height parameter


    // Declare timing parameters

    // create the service client for getting the RRT* path
    planner_client_ = create_client<avoid_plan_msgs::srv::MakePlan>("make_plan");

    RCLCPP_INFO(get_logger(), "Starting second thread");

    // create and start the runner thread
    plan_runner_thread_ = std::thread{std::bind(&IKExecutorNode::planRun, this)};
    plan_runner_thread_.detach();
  }

  void planRun(){
    // need to create planner stuff first
    base_link_name_ = get_parameter("base_link").as_string();
    plan_grp_name_ = get_parameter("planning_group").as_string();

    try {
      // Create the moveit instance
      using moveit::planning_interface::MoveGroupInterface;
      mv_grp_interf_ = std::make_shared<MoveGroupInterface>(this->shared_from_this(), plan_grp_name_);
      mv_grp_interf_->setPoseReferenceFrame(base_link_name_);

    } catch (const std::exception & e){
      RCLCPP_FATAL_STREAM(get_logger(), "Failed to create plan group. Error: " << e.what());
      return;
    }

    
    try{
      // create the visualizer instance
      viz_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(this->shared_from_this(), base_link_name_);
      viz_tools_->loadRemoteControl();
      viz_tools_->deleteAllMarkers();

      // put a default visualization message in
      text_pose_.position.z = 2.5;
      viz_tools_->publishText(text_pose_, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
      viz_tools_->trigger();
    } catch (const std::exception & e){
      RCLCPP_FATAL_STREAM(get_logger(), "Failed to init vizualizer. Error: " << e.what());
      return;
    }

    // use a set of constant points
    geometry_msgs::msg::Pose startingPose;
    startingPose.orientation.w = 0.0;
    startingPose.orientation.x = 0.7071067811865476;
    startingPose.orientation.y = -0.7071067811865476;
    startingPose.orientation.z = 0.0;
    startingPose.position.x = -0.735;
    startingPose.position.y =  0.000;
    startingPose.position.z =  0.350;

    geometry_msgs::msg::Pose cupStartPose(startingPose);
    cupStartPose.position.z =  0.310;

    geometry_msgs::msg::Pose endingPose;
    endingPose.orientation.w = 0.0;
    endingPose.orientation.x = 0.7071067811865476;
    endingPose.orientation.y = -0.7071067811865476;
    endingPose.orientation.z = 0.0;
    endingPose.position.x = 0.560;
    endingPose.position.y = 0.000;
    endingPose.position.z = 0.350;

    geometry_msgs::msg::Pose cupEndPose(endingPose);
    cupEndPose.position.z =  0.315;

    // Record the starting pose of the robot to return to later
    end_link_name_ = get_parameter("end_link").as_string();
    const auto initalPose = mv_grp_interf_->getCurrentPose(end_link_name_).pose;

    viz_tools_->prompt("Proceed with planned motion");

    // exceute plan from starting pose to path starting point
    bool success = false;
    while(! success){
      success = moveToPoint(startingPose);
      if(! success){
        RCLCPP_ERROR(get_logger(), "Failed to move robot to starting point");
        auto res = viz_tools_->prompt("Retry?");
        if(! res){
          RCLCPP_WARN(get_logger(), "User requested stop, after move failure");
          return;
        }
      }
    }

    // used for testing motions indvidually
    // viz_tools_->prompt("Moving down to cup");

    // execute plan and move to cup pose
    success = false;
    while(! success){
      success = moveToPoint(cupStartPose);
      if(! success){
        RCLCPP_ERROR(get_logger(), "Failed to move robot to cup");
        auto res = viz_tools_->prompt("Retry?");
        if(! res){
          RCLCPP_WARN(get_logger(), "User requested stop, after move failure");
          return;
        }
      }
    }

    // wait for gripper to close
    viz_tools_->prompt("Please close gripper");

    // execute plan and move back to path starting point
    success = false;
    while(! success){
      success = moveToPoint(startingPose);
      if(! success){
        RCLCPP_ERROR(get_logger(), "Failed to move robot to starting point");
        auto res = viz_tools_->prompt("Retry?");
        if(! res){
          RCLCPP_WARN(get_logger(), "User requested stop, after move failure");
          return;
        }
      }
    }

    // make the three stage trajectory
    std::vector<geometry_msgs::msg::Pose> waypoints, plannerPoints;
    waypoints.push_back(startingPose);
    waypoints.push_back(endingPose);

    // make the service request to the planner
    auto request = std::make_shared<avoid_plan_msgs::srv::MakePlan::Request>();

    // wrap poses into pose stamped
    std::vector<geometry_msgs::msg::PoseStamped> stampedWps;
    for(auto pose : waypoints){
      geometry_msgs::msg::PoseStamped poseStamped;
      poseStamped.pose = pose;
      poseStamped.header.frame_id = base_link_name_;
      poseStamped.header.stamp = get_clock()->now();

      stampedWps.push_back(poseStamped);
    }

    request->path_in.poses = stampedWps;
    request->path_in.header.frame_id = base_link_name_;
    request->path_in.header.stamp = get_clock()->now();

    while (!planner_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }

    // viz_tools_->prompt("Ready to send request to RRT");
    RCLCPP_INFO(get_logger(), "Sending request to planner server");

    auto future = planner_client_->async_send_request(request);
    while(! future.valid()){
      // check that we havent shut down
      if(! rclcpp::ok()){
        return;
      }
    }

    RCLCPP_INFO(get_logger(), "Planning response recieved");
    
    // read the new points out
    auto result = future.get();
    plannerPoints.push_back(startingPose);
    for(auto poseStapmed : result->path_out.poses){
      plannerPoints.push_back(poseStapmed.pose);
    }
    plannerPoints.push_back(endingPose);

    RCLCPP_INFO_STREAM(get_logger(), "Planning joint space trajectory in coordinate frame: "
      << mv_grp_interf_->getPoseReferenceFrame());


    // Send the points into moveit
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 10.0; // this can cause big moves for physical hardware if zero
    const double eef_step = 0.01;
    double fraction = mv_grp_interf_->computeCartesianPath(plannerPoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(get_logger(), "path planned successfully (%.2f%% acheived)", fraction * 100.0);

    viz_tools_->deleteAllMarkers();
    viz_tools_->publishText(endingPose, "Joint Space Goal", rvt::WHITE, rvt::LARGE);
    viz_tools_->publishPath(plannerPoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < plannerPoints.size(); ++i)
      viz_tools_->publishAxisLabeled(plannerPoints[i], "pt" + std::to_string(i), rvt::SMALL);

    viz_tools_->trigger();

    if(fraction < 0.20){
      RCLCPP_FATAL(get_logger(), "Trajectory building failed");
      return;
    }

    // used for debugging path planning
    // viz_tools_->prompt("Execute obstacle path?");

    // execute cup move plan
    mv_grp_interf_->execute(trajectory);

    // execute plan and move to ending cup pose
    success = false;
    while(! success){
      success = moveToPoint(cupEndPose);
      if(! success){
        RCLCPP_ERROR(get_logger(), "Failed to move robot to cup dropoff");
        auto res = viz_tools_->prompt("Retry?");
        if(! res){
          RCLCPP_WARN(get_logger(), "User requested stop, after move failure");
          return;
        }
      }
    }

    // wait for gripper to open
    viz_tools_->prompt("Please open gripper");

    // execute plan and move up out of the end cup pose
    success = false;
    while(! success){
      success = moveToPoint(endingPose);
      if(! success){
        RCLCPP_ERROR(get_logger(), "Failed to move robot to post dropoff location");
        auto res = viz_tools_->prompt("Retry?");
        if(! res){
          RCLCPP_WARN(get_logger(), "User requested stop, after move failure");
          return;
        }
      }
    }

    // execute plan from path end point back to robot starting pose
    success = false;
    while(! success){
      success = moveToPoint(initalPose);
      if(! success){
        RCLCPP_ERROR(get_logger(), "Failed to move robot to inital point");
        auto res = viz_tools_->prompt("Retry?");
        if(! res){
          RCLCPP_WARN(get_logger(), "User requested stop, after move failure");
          return;
        }
      }
    }
  }

  bool moveToPoint(geometry_msgs::msg::Pose target_pose){
    // Set a target Pose into moveit
    mv_grp_interf_->setGoalPositionTolerance(0.01);
    mv_grp_interf_->setGoalOrientationTolerance(0.01);
    // mv_grp_interf_->setPoseTarget(target_pose);
    bool ikOk = mv_grp_interf_->setApproximateJointValueTarget(target_pose);
    if(! ikOk){
      RCLCPP_WARN(get_logger(), "Unable to find approximate IK solution for goal pose");
      return false;
    }

    // Create a plan to the target pose
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto planStatus = mv_grp_interf_->plan(msg);
    
    if(static_cast<bool>(planStatus)){
      const auto initalPose = mv_grp_interf_->getCurrentPose(end_link_name_).pose;

      // Display the plan in RVIZ
      const auto jmg = mv_grp_interf_->getCurrentState()->getJointModelGroup(plan_grp_name_);
      viz_tools_->deleteAllMarkers();
      viz_tools_->publishAxisLabeled(initalPose, "start");
      viz_tools_->publishAxisLabeled(target_pose, "goal");
      viz_tools_->publishTrajectoryLine(msg.trajectory_, jmg);
      viz_tools_->trigger();

      // Used for debugging path planning
      // if(! viz_tools_->prompt("Path ready. Execute?")){
      //   RCLCPP_WARN(get_logger(), "User cancelled path");
      //   return false;
      // }


      // Execute the plan if it was able to make a plan
      auto executeStatus = mv_grp_interf_->execute(msg);

      RCLCPP_INFO(get_logger(), "Trajectory executed");

      // return true if the error code is okay
      if(static_cast<bool>(executeStatus)){
        return true;
      } else {
        // Show an error if the execution failed
        RCLCPP_ERROR_STREAM(get_logger(), "Execution of path failed with status: " << executeStatus.val);
      }
    } else {
      // Show an error for a planning failure
      RCLCPP_ERROR_STREAM(get_logger(), "Planning of path failed with status: " << planStatus.val);
    }

    // we failed somewhere during the routine
    return false;
  }

  void wait(){
    plan_runner_thread_.join();
  }


  private:
    // names for key links / items
    std::string end_link_name_, base_link_name_, plan_grp_name_;

    // planner client
    rclcpp::Client<avoid_plan_msgs::srv::MakePlan>::SharedPtr planner_client_;

    // execution thread
    std::thread plan_runner_thread_;

    // moveit instance
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mv_grp_interf_;

    // moveit visualization tools
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> viz_tools_;
    geometry_msgs::msg::Pose text_pose_;

};


int main(int argc, char** argv) {
  // init ros node
  rclcpp::init(argc, argv);

  // Create the node and spin it 
  auto driver = std::make_shared<IKExecutorNode>();
  rclcpp::spin(driver);

  driver->wait();

  // the node has been called to shutdown, so rclcpp needs to be shut down as well
  rclcpp::shutdown();
  return 0;
}
