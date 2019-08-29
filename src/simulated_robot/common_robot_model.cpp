//
// Created by wei on 8/29/19.
//

#include "simulated_robot/common_robot_model.h"

#include <drake/common/find_resource.h>

std::unique_ptr<arm_runner::SimulatedRobotArm> arm_runner::constructSimulatedKukaDefault(double simulation_time_second) {
    // The rigid body tree
    const char* kModelPath =
            "drake/manipulation/models/iiwa_description/"
            "urdf/iiwa14_polytope_collision.urdf";
    const std::string urdf = drake::FindResourceOrThrow(kModelPath);
    return std::make_unique<SimulatedRobotArm>(urdf, simulation_time_second);
}