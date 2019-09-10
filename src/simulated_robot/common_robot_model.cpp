//
// Created by wei on 8/29/19.
//

#include "simulated_robot/common_robot_model.h"
#include <drake/common/find_resource.h>
#include <drake/multibody/parsers/urdf_parser.h>


std::vector<std::string> plan_runner::getKukaJointNameList() {
    return {
        "iiwa_joint_1",
        "iiwa_joint_2",
        "iiwa_joint_3",
        "iiwa_joint_4",
        "iiwa_joint_5",
        "iiwa_joint_6",
        "iiwa_joint_7"
    };
}


std::unique_ptr<RigidBodyTree<double>> plan_runner::constructDefaultKukaRBT() {
    const char* kModelPath =
            "drake/manipulation/models/iiwa_description/"
            "urdf/iiwa14_polytope_collision.urdf";
    const std::string urdf = drake::FindResourceOrThrow(kModelPath);
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            urdf, drake::multibody::joints::kFixed, tree.get());
    return tree;
}


std::unique_ptr<plan_runner::SimulatedRobotArm> plan_runner::constructSimulatedKukaDefault(double simulation_time_second) {
    // The rigid body tree
    auto robot = constructDefaultKukaRBT();
    return std::make_unique<SimulatedRobotArm>(std::move(robot), simulation_time_second);
}