#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

#include <robotiq_s_model_articulated_msgs/SModelRobotOutput.h>
#include "nomagic_test/DetectObject.h"

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

class NomagicTask {
public:
  NomagicTask() : group("manipulator"), trajectory_client("/arm_controller/follow_joint_trajectory", true) {
    gripper_cmd = n.advertise<robotiq_s_model_articulated_msgs::SModelRobotOutput>("/left_hand/command", 10);
    sleep(2);
    group.setPlanningTime(10.0);
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
    
    command.rACT = 1;
    command.rGTO = 1;
    command.rSPA = 255;
    command.rFRA = 150;
    
    gripper_cmd.publish(command);
    
    object_detection_client =  n.serviceClient<nomagic_test::DetectObject>("/detect_object");
    object_detection_client.waitForExistence();
    
  }

  void gripperOpen() {
    command.rPRA = 0;
    gripper_cmd.publish(command);
  }

  void gripperClose() {
    command.rPRA = 255;
    gripper_cmd.publish(command);
  }

  bool ptpMove(const KDL::Frame &pose) {
    geometry_msgs::Pose pose_msg;
    tf::poseKDLToMsg(pose, pose_msg);
    
    group.setPoseTarget(pose_msg);
    return (bool)group.move();
  }

  bool cartesianMove(const KDL::Frame &pose) {
    geometry_msgs::Pose pose_msg;
    tf::poseKDLToMsg(pose, pose_msg);
  
    std::vector<geometry_msgs::Pose> waypoints;
    moveit_msgs::RobotTrajectory trajectory;
    
    waypoints.push_back(pose_msg);
    
    double fraction = group.computeCartesianPath(waypoints,
                                                 0.002,  // eef_step
                                                 1.1,   // jump_threshold
                                                 trajectory,
                                                 false);
  
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
          fraction * 100.0);
    
    if (fraction < 0.0) {
      return false;
    }
  
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    plan.trajectory_ = trajectory;
    
    return (bool)group.execute(plan);
  }
  
  bool getObjectPose(KDL::Frame &pose) {
    nomagic_test::DetectObject srv;

    if (object_detection_client.call(srv)) {
      if (srv.response.detected) {
        tf::poseMsgToKDL(srv.response.pose, pose);
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  
  void moveToInitialPose() {
    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("elbow_joint");
    goal.trajectory.joint_names.push_back("wrist_1_joint");
    goal.trajectory.joint_names.push_back("wrist_2_joint");
    goal.trajectory.joint_names.push_back("wrist_3_joint");

    goal.trajectory.points.resize(1);

    goal.trajectory.points[0].positions.resize(6);
    goal.trajectory.points[0].positions[0] = 0.0;
    goal.trajectory.points[0].positions[1] = -2.2;
    goal.trajectory.points[0].positions[2] = 2.2;
    goal.trajectory.points[0].positions[3] = -1.57;
    goal.trajectory.points[0].positions[4] = -1.57;
    goal.trajectory.points[0].positions[5] = 0.0;
    
    goal.trajectory.points[0].time_from_start = ros::Duration(5.0);
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    trajectory_client.sendGoal(goal);
    
    trajectory_client.waitForResult();
  }
  
  void attachObject(std::string id) {
    group.attachObject(id);
  }
  
  void detachObject(std::string id) {
    group.detachObject(id);
  }
private:
  moveit::planning_interface::MoveGroupInterface group;
  ros::Publisher gripper_cmd;
  ros::NodeHandle n;
  robotiq_s_model_articulated_msgs::SModelRobotOutput command;
  ros::ServiceClient object_detection_client;
  actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > trajectory_client;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nomagic_task");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  NomagicTask task;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.2;
  
  KDL::Frame pose, pose_tmp;
  KDL::Frame pregrasp_offset(KDL::Vector(-0.30, 0.0, 0.0));
  KDL::Frame grasp_offset(KDL::Vector(-0.20, 0.0, 0.0));
  KDL::Frame postgrasp_offset(KDL::Vector(-0.20, 0.0, 0.10));
  KDL::Frame bin_pose(KDL::Rotation::RotX(1.57), KDL::Vector(0.0, 1.0, 1.3));

  while(ros::ok()) {
    task.moveToInitialPose();
    task.gripperOpen();
    sleep(2);

    while (task.getObjectPose(pose) == false) {
      sleep(1);
    }

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    tf::poseKDLToMsg(pose, box_pose);
    
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "box1";
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);

    // move to pre-grasp pose
    pose_tmp = pose * pregrasp_offset * KDL::Frame(KDL::Rotation::RotX(1.57));
    task.ptpMove(pose_tmp);
  
    // move to grasp pose
    pose_tmp = pose * grasp_offset * KDL::Frame(KDL::Rotation::RotX(1.57));
    task.cartesianMove(pose_tmp);
          
    task.gripperClose();
    sleep(2);
    
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
    
    // lift object
    pose_tmp = pose * postgrasp_offset * KDL::Frame(KDL::Rotation::RotX(1.57));
    task.cartesianMove(pose_tmp);
    
    // move over the bin
    task.ptpMove(bin_pose);
    
    task.gripperOpen();
    sleep(3);
  }
  ros::shutdown();  
  return 0;
}
