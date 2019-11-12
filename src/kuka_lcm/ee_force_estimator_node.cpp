//
// Created by wei on 11/11/19.
//

#include <ros/ros.h>
#include "simulated_robot/common_robot_model.h"
#include "robot_plan/ee_force_estimator.h"


int main(int argc, char *argv[]) {
    // Init the ros staff
    using namespace plan_runner;
    ros::init(argc, argv, "ee_force_estimator");
    ros::NodeHandle nh("ee_force_estimator");

    // Construct estimator
    auto tree = constructDefaultKukaRBT();
    EEForceTorqueEstimator estimator(nh, std::move(tree));

    // Make and spin
    estimator.Initialize();
    ros::spin();
}