// system
#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
//ROS
#include <ros/ros.h>
 
// GPG
#include <gpg/cloud_camera.h>

// this project (messages)
#include <gpd/CloudIndexed.h>
#include <gpd/CloudSamples.h>
#include <gpd/CloudSources.h>
#include <gpd/GraspConfig.h>
#include <gpd/GraspConfigList.h>
#include <gpd/SamplesMsg.h>

// moveit
#include <moveit/move_group_interface/move_group_interface.h> 
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>

// geometry_msgs::Pose target_pose; 

// bool is_start = false;
void grasps_callback(const gpd::GraspConfigList& msg)
{  
    int grasps_size = msg.grasps.size(); 
    ROS_INFO_STREAM("Received grasps list size is " << grasps_size);

    int max_score = 0;
    unsigned int max_score_index = 0;
    unsigned int index = 0;
    
    // find max score
    if(msg.grasps.size() != 0)
    {
      for(auto grasp_it = msg.grasps.begin(); grasp_it != msg.grasps.end();grasp_it++)
      {  
        std::cout << grasp_it->score << std::endl;
        if(grasp_it->score.data > max_score)
        { 
          max_score = grasp_it->score.data;
          max_score_index = index; 
        }
        index++;
      }  

      // Echo grasp message
      ROS_INFO_STREAM("Max score is" << msg.grasps[max_score_index].score.data);

      ROS_INFO_STREAM("approach is\n" << msg.grasps[max_score_index].approach);
      ROS_INFO_STREAM("binormal is\n" << msg.grasps[max_score_index].binormal);
      ROS_INFO_STREAM("axis is\n" << msg.grasps[max_score_index].axis);

      ROS_INFO_STREAM("bottom is\n" << msg.grasps[max_score_index].bottom);
      ROS_INFO_STREAM("top is\n" << msg.grasps[max_score_index].top); 
      ROS_INFO_STREAM("surface is\n" << msg.grasps[max_score_index].surface);

      ROS_INFO_STREAM("Max width is " << msg.grasps[max_score_index].width.data); 

      // test
//       Eigen::Matrix3d t_R;
//       t_R << msg.grasps[max_score_index].approach
//       <<msg.grasps[max_score_index].binormal
//       <<msg.grasps[max_score_index].axis;
//
//        std::cout << t_R;

                //        AngleAxisd t_V(M_PI / 4, Vector3d(0, 0, 1));
//        Matrix3d t_R = t_V.matrix();
//        Quaterniond t_Q(t_V);
//
//
//      Eigen::AngleAxisf rotationVector();
//      Eigen::Matrix3d rotationMatrix=Eigen::Matrix3d::Identity();
//      rotationMatrix=msg.grasps[max_score_index].approach.toRotationMatrix();
      
      // ROS_INFO_STREAM("eulerAngle roll pitch yaw\n" << rotationMatrix);  


      ROS_INFO_STREAM("----------------------------------------------------------");   
      ROS_INFO_STREAM("----------------------------------------------------------");   
    }
    
  // // end-effector.
  // //末端姿态四元数
  // target_pose.orientation.w = 1.000000;
  // target_pose.orientation.x = 0.000000;
  // target_pose.orientation.y = 0.000000;
  // target_pose.orientation.z = 0.000000; 
  // //末端姿态三维坐标
  // target_pose.position.x = msg.grasps[max_score_index].approach.x;
  // target_pose.position.y = msg.grasps[max_score_index].approach.y;
  // target_pose.position.z = msg.grasps[max_score_index].approach.z;
}


int main(int argc, char** argv)
{  
  // initialize ROS
  ros::init(argc, argv, "arm_destination_publish");
  ros::NodeHandle node("~");
     ros::AsyncSpinner spinner(1);
   spinner.start();
  bool success = false;  
   


  ros::Subscriber grasps_sub;
  grasps_sub = node.subscribe("/detect_grasps/clustered_grasps", 10, &grasps_callback);
  // static const std::string PLANNING_GROUP = "endeffector";
  // can be easily setup using just the name of the planning group you would like to control and plan for.
  // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // // use the PlanningSceneInterface class to add and remove collision objects in our “virtual world” scene
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // // Raw pointers are frequently used to refer to the planning group for improved performance.
  // // const robot_state::JointModelGroup* joint_model_group =
  // //     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  ros::Rate rate(100);
  
  // target_pose.orientation.w = 1.000000;
  // target_pose.orientation.x = 0.000000;
  // target_pose.orientation.y = 0.000000;
  // target_pose.orientation.z = 0.000000; 
  // //末端姿态三维坐标
  // target_pose.position.x = 0.0;
  // target_pose.position.y = 0.0;
  // target_pose.position.z = 0.4;

  // move_group.setPoseTarget(target_pose,"wrist_3_link"); 
  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); 
  // //运动规划输出
  // ROS_INFO("Visualizing plan (stateCatch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
  // if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  move_group.execute(my_plan);
  // else
  //   std::cout << "failer to grasp" << std::endl;

  while(ros::ok())
  {
  //   if(is_start)
  //   {
  //     std::cout << "--------------------------------------------------------------"<< std::endl;
  //     std::cout << "start to grasp"<< std::endl;
   
  //     is_start = false; 
  //     //进行运动规划


  //   }
    ros::spinOnce();
    rate.sleep(); 
  } 
  ros::shutdown();
  return 0;
}
