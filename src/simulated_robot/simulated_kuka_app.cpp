//
// Created by wei on 8/29/19.
//

#include <string>
#include <memory>
#include <ros/ros.h>
#include <drake/common/find_resource.h>
#include <drake/multibody/rigid_body_tree.h>

#include "common/plan_supervisor.h"
#include "common/joint_publish_processor.h"
#include "simulated_robot/common_robot_model.h"
#include "simulated_robot/simulated_robot.h"


int main(int argc, char* argv[]) {
    // Get the simulation time
    double simulation_time = 20.0;
    if(argc == 2) {
        simulation_time = std::stod(argv[1]);
    }

    // The simulated robot model
    using namespace arm_runner;
    std::unique_ptr<RobotCommunication> robot_arm = constructSimulatedKukaDefault(simulation_time);

    // Empty init of node
    std::vector<std::pair<std::string, std::string>> vector_map;
    ros::init(vector_map, "plan_runner");
    ros::NodeHandle nh("plan_runner"); // sets the node's namespace

    // The publisher
    JointPublishProcessor::Ptr publisher = std::make_shared<JointPublishProcessor>(
        nh, "/iiwa/joint_states", getKukaJointNameList());
    robot_arm->AddMeasurementProcessor(publisher);

    // The rigid body tree
    auto tree = constructDefaultKukaRBT();
    PlanSupervisor supervisor(std::move(tree), std::move(robot_arm), nh);

    // The initialization
    supervisor.Start();
    ros::Duration(0.1).sleep(); // Wait for the measurement
    ROS_INFO("Simulated robot fully started!");

    // The main loop
    auto start_time = std::chrono::system_clock::now();
    ros::Rate rate(100); // 100 hz
    while (!ros::isShuttingDown()) {
        // The iteration
        supervisor.ProcessLoopIteration();

        // Check time
        auto now = std::chrono::system_clock::now();
        auto int_s = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
        if(int_s.count() > int(simulation_time)){
            break;
        }

        // OK
        rate.sleep();
    }
    supervisor.Stop();
}