// student ID:2018102224
// name: Junpyo Lim

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <iostream>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


void go_to_pose_goal(moveit::planning_interface::MoveGroupInterface &move_group_interface,
geometry_msgs::Pose &target_pose) {
  // .. _move_group_interface-planning-to-pose-goal:

  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
  // Note that this can lead to problems if the robot moved in the meanwhile.
  move_group_interface.execute(my_plan);

}

geometry_msgs::Pose list_to_pose(double x,double y,double z,double roll,double pitch,double yaw)
{

  geometry_msgs::Pose target_pose;
  tf2::Quaternion orientation;
  orientation.setRPY(roll,pitch, yaw);
  target_pose.orientation= tf2::toMsg(orientation);
  target_pose.position.x=x;
  target_pose.position.y=y;
  target_pose.position.z=z;

  return target_pose;
}

void gripper_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,double value)
{
  ROS_INFO("gripper sample"); 
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;

  joint_group_positions=move_group_interface.getCurrentJointValues();
  joint_group_positions[0]=0;
  joint_group_positions[1]=0;
  joint_group_positions[2]=value;
  joint_group_positions[3]=0;
  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  move_group_interface.setJointValueTarget(joint_group_positions);
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.execute(my_plan);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "khu_moveit_sameple");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface gripper("gripper");
  arm.setPlanningTime(200.0);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();
  
    // place(arm,gripper);
	// pick(arm,gripper);

  geometry_msgs::Pose target_pose;

//grab block1
  target_pose=list_to_pose(0.5, 0, 0.1, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();
  
  target_pose=list_to_pose(0.5, 0, 0.029, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();  

  gripper_sample(gripper,0.35);
  ros::WallDuration(1.0).sleep();
  
  target_pose=list_to_pose(0.5, 0, 0.035, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  //drop block 1
  target_pose=list_to_pose(0.4, 0, 0.035, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();     

  target_pose=list_to_pose(0.4, 0, 0.104, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(2.0).sleep();

  //grab block6
  target_pose=list_to_pose(0.3, -0.1, 0.1, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();
  
  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();

  target_pose=list_to_pose(0.3, -0.1, 0.029, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  gripper_sample(gripper,0.35);
  ros::WallDuration(1.0).sleep();

  target_pose=list_to_pose(0.3, -0.1, 0.3, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  //drop block6
  target_pose=list_to_pose(0.4, 0, 0.099, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  gripper_sample(gripper,0.5);
  target_pose=list_to_pose(0.4, 0.1, 0.3, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  //grab block2
  target_pose=list_to_pose(0.4, 0.1, 0.1, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();
  
  target_pose=list_to_pose(0.4, 0.1, 0.028, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();  

  gripper_sample(gripper,0.35);
  ros::WallDuration(1.0).sleep();

  target_pose=list_to_pose(0.4, 0.1, 0.15, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep(); 

  //drop block 2
  target_pose=list_to_pose(0.4, 0, 0.144, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();    
  
  target_pose=list_to_pose(0.4, 0, 0.25, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  //go to block 3
  target_pose=list_to_pose(0.3, 0, 0.25, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  //grab block 3
  target_pose=list_to_pose(0.4, -0.1, 0.1, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();
  
  target_pose=list_to_pose(0.4, -0.1, 0.029, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();  

  gripper_sample(gripper,0.35);
  ros::WallDuration(1.0).sleep();

  target_pose=list_to_pose(0.4, -0.1, 0.199, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep(); 

  //drop block 3
  target_pose=list_to_pose(0.4, 0, 0.199, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();    
  
  target_pose=list_to_pose(0.4, 0, 0.3, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();
//grab block 5
  target_pose=list_to_pose(0.3, 0, 0.3, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();
  
  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();

  target_pose=list_to_pose(0.3, 0, 0.1, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();
  
  target_pose=list_to_pose(0.3, 0, 0.029, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();  

  gripper_sample(gripper,0.35);
  ros::WallDuration(1.0).sleep();

  target_pose=list_to_pose(0.3, 0, 0.249, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();  

  //drop block 5
  target_pose=list_to_pose(0.4, 0, 0.249, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();    
  
  target_pose=list_to_pose(0.4, 0, 0.35, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  //grab block4
  target_pose=list_to_pose(0.3, 0.1, 0.35, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();
  
  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();
  
  target_pose=list_to_pose(0.3, 0.1, 0.1, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();
  
  target_pose=list_to_pose(0.3, 0.1, 0.029, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();  

  gripper_sample(gripper,0.35);
  ros::WallDuration(1.0).sleep();

  target_pose=list_to_pose(0.3, 0.1, 0.1, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep(); 

  //drop block 4
  target_pose=list_to_pose(0.4, 0, 0.299, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();

  gripper_sample(gripper,0.5);
  ros::WallDuration(1.0).sleep();    
  
  target_pose=list_to_pose(0.4, 0, 0.37, -M_PI ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep();
  
  //end
  target_pose=list_to_pose(0.34, 0.2, 0.3, -M_PI/2 ,0, 0);
  go_to_pose_goal(arm,target_pose);
  ros::WallDuration(1.0).sleep(); 

  return 0;
}
