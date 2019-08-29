//
// Created by wei on 8/29/19.
//

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <drake/common/find_resource.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/multibody/parsers/urdf_parser.h>

#include "arm_runner/plan_supervisor.h"
#include "simulated_robot/common_robot_model.h"
#include "simulated_robot/simulated_robot.h"


TEST(SimRobotTest, ConstructTest) {
    using namespace arm_runner;
    auto robot_arm = constructSimulatedKukaDefault(10);
    robot_arm->Start();
    robot_arm->Stop();
}


TEST(SimRobotTest, SupervisorConstructTest) {
    using namespace arm_runner;
    constexpr double simulation_time = 10.0;
    std::unique_ptr<RobotCommunication> robot_arm = constructSimulatedKukaDefault(10);

    // Empty init
    std::vector<std::pair<std::string, std::string>> vector_map;
    ros::init(vector_map, "plan_runner");
    ros::NodeHandle nh("plan_runner"); // sets the node's namespace

    // The rigid body tree
    const char* kModelPath =
            "drake/manipulation/models/iiwa_description/"
            "urdf/iiwa14_polytope_collision.urdf";
    const std::string urdf = drake::FindResourceOrThrow(kModelPath);
    auto tree = std::make_shared<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            urdf, drake::multibody::joints::kFixed, tree.get());

    // Construct the supervisor
    PlanSupervisor supervisor(tree, std::move(robot_arm), nh);

    // The initialization
    supervisor.Start();
    ros::Duration(0.1).sleep(); // Wait for the measurement

    // The main loop
    auto start_time = std::chrono::system_clock::now();
    ros::Rate rate(100); // 10 hz
    while (!ros::isShuttingDown()) {
        // The iteration
        supervisor.ProcessLoopIteration();

        // Check time
        auto now = std::chrono::system_clock::now();
        auto int_s = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
        if(int_s.count() > int(simulation_time))
            break;

        // OK
        rate.sleep();
    }
    supervisor.Stop();
}