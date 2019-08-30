//
// Created by wei on 8/29/19.
//

#pragma once

#include <memory>
#include <drake/multibody/rigid_body_tree.h>

#include "simulated_robot/simulated_robot.h"


namespace arm_runner {

    // The kuka model
    // Note that the RobotCommunication must send msg following the name of getKukaJointNameList()
    std::vector<std::string> getKukaJointNameList();
    std::unique_ptr<RigidBodyTree<double>> constructDefaultKukaRBT();
    std::unique_ptr<SimulatedRobotArm> constructSimulatedKukaDefault(double simulation_time_second = 10);
}