//
// Created by wei on 9/2/19.
//

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "supervisor/plan_supervisor.h"
#include "simulated_robot/common_robot_model.h"
#include "kuka_lcm.h"


std::unique_ptr<plan_runner::KukaLCMInterface> constructKukaLCMInterface(const YAML::Node& config) {
    if (!config["lcm_status_channel"] || !config["lcm_command_channel"]) {
        return nullptr;
    }

    // Construct the interface
    std::string lcm_status_channel = config["lcm_status_channel"].as<std::string>();
    std::string lcm_command_channel = config["lcm_command_channel"].as<std::string>();
    return std::make_unique<plan_runner::KukaLCMInterface>(
        std::move(lcm_status_channel),
        std::move(lcm_command_channel)
    );
}


int main(int argc, char *argv[]) {
    // Check the config
    if(argc < 2) {
        ROS_INFO("Need the path to config file as the parameter");
        std::exit(1);
    }
    std::string config_file_path = argv[1];
    YAML::Node config = YAML::LoadFile(config_file_path);

    // Construct the communication
    auto robot_arm = constructKukaLCMInterface(config);
    if(robot_arm == nullptr) {
        std::cerr << "Config file missing one or more fields." << std::endl;
        std::exit(1);
    }

    // Init the ros staff
    using namespace plan_runner;
    ros::init(argc, argv, "plan_runner");
    ros::NodeHandle nh("plan_runner");

    // The initialization
    auto tree = constructDefaultKukaRBT();
    PlanSupervisor supervisor(std::move(tree), std::move(robot_arm), nh, config);
    supervisor.Initialize();
    ROS_INFO("Kuka lcm-interface supervisor started!");

    // Rate information
    double control_rate = 200.0;
    double ros_rate = 200;
    // Read from config
    if(config["control_rate"]) {
        control_rate = config["control_rate"].as<double>();
        ros_rate = config["control_rate"].as<double>();
    }

    double control_interval = 1.0 / control_rate;
    ros::Rate rate(ros_rate); // 100 hz

    // The main loop
    while (!ros::isShuttingDown()) {
        // The iteration
        supervisor.ProcessLoopIteration(control_interval);

        // OK
        ros::spinOnce();
        rate.sleep();
    }

    // Kill the supervisor
    supervisor.Stop();
    return 0;
}