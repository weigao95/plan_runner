//
// Created by wei on 8/29/19.
//

#include "simulated_robot/common_robot_model.h"
#include <drake/common/find_resource.h>
#include <drake/multibody/parsers/urdf_parser.h>


std::unique_ptr<RigidBodyTree<double>> arm_runner::constructDefaultKukaRBT() {
    const char* kModelPath =
            "drake/manipulation/models/iiwa_description/"
            "urdf/iiwa14_polytope_collision.urdf";
    const std::string urdf = drake::FindResourceOrThrow(kModelPath);
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            urdf, drake::multibody::joints::kFixed, tree.get());
    return tree;
}


std::unique_ptr<arm_runner::SimulatedRobotArm> arm_runner::constructSimulatedKukaDefault(double simulation_time_second) {
    // The rigid body tree
    auto robot = constructDefaultKukaRBT();
    return std::make_unique<SimulatedRobotArm>(std::move(robot), simulation_time_second);
}