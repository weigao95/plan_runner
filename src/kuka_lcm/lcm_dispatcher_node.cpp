//
// Created by wei on 9/9/19.
//

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "lcm_plan_dispatcher.h"


int main(int argc, char *argv[]) {
    // Check the config
    if(argc < 2) {
        ROS_INFO("Need the path to config file as the parameter");
        std::exit(1);
    }
    std::string config_file_path = argv[1];
    YAML::Node config = YAML::LoadFile(config_file_path);

    // Get the channel
    std::string lcm_plan_channel = config["lcm_plan_channel"].as<std::string>();
    std::string lcm_stop_channel = config["lcm_stop_channel"].as<std::string>();
    std::string joint_trajectory_action = "/plan_runner/JointTrajectory";
    std::string stop_plan_service = "/plan_runner/stop_plan";

    // ROS node
    using namespace plan_runner;
    ros::init(argc, argv, "lcm_command_bridge");
    ros::NodeHandle nh("lcm_command_bridge");

    // Construct the dispatcher
    KukaPlanDispatcherLCM dispatcher(
        nh,
        std::move(lcm_plan_channel), std::move(lcm_stop_channel),
        joint_trajectory_action, stop_plan_service);
    dispatcher.Start();
}