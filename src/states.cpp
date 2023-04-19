#include <ros/ros.h>
#include <pluginlib/class_loader.hpp>

#include <string>
#include <fstream>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <boost/scoped_ptr.hpp>


int main(int argc, char** argv){
    
    ros::init(argc, argv, "Motion_Planner");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle node_handle("~");

    const std::string PLANNING_GROUP = "panda_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    std::ofstream myfile("/home/aditya/ws_moveit/src/motion_planner/file.csv");
    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());
    myfile << "Time, "<<"X, "<<"Y, "<<"Z, "<<"\n";

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    ros::Time start = ros::Time::now();

    while(ros::ok()){
        
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
        const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("panda_link8");
        ros::Duration now = ros::Time::now() - start;
        double sec = now.sec + now.nsec * 1e-9;
        ROS_INFO_STREAM("Time: \n" << sec << "\n");
        myfile << sec <<", "<<end_effector_state.translation()[0]<<", "<<end_effector_state.translation()[1]<<", "<<end_effector_state.translation()[2]<<"\n";
            
        /* Print end-effector pose. Remember that this is in the model frame */
        ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
        // ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
        
    }
    
    myfile.close();
    ros::shutdown();
    return 0;

}
