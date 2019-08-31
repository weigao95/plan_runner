//
// Created by wei on 8/29/19.
//

#include <string>
#include <memory>
#include <chrono>
#include <ros/ros.h>
#include <drake/common/find_resource.h>
#include <drake/multibody/rigid_body_tree.h>

#include "common/plan_supervisor.h"
#include "common/joint_publish_processor.h"
#include "simulated_robot/common_robot_model.h"
#include "simulated_robot/simulated_robot.h"


int main(int argc, char* argv[]) {
    // Get the simulation time
    double simulation_time = 30.0;
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
    ROS_INFO("Simulated robot fully started for %f sceonds!", simulation_time);

    // The counting
    long average = 0;
    long max_time = 0;
    int iteration = 0;

    // The main loop
    auto start_time = std::chrono::system_clock::now();
    double control_rate = 100.0;
    double control_interval = 1.0 / control_rate;
    ros::Rate rate(control_rate); // 100 hz
    while (!ros::isShuttingDown()) {
        // The iteration
        auto before = std::chrono::system_clock::now();
        supervisor.ProcessLoopIteration(control_interval);
        auto after = std::chrono::system_clock::now();
        auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(after - before).count();

        // Update
        if(duration_ns > max_time)
            max_time = duration_ns;
        iteration += 1;
        average += duration_ns;

        // Debug
        if(duration_ns > 1000000) {
            ROS_INFO("SLOW Iteration!");
        }

        // Check time
        auto now = std::chrono::system_clock::now();
        auto int_s = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
        if(int_s.count() > int(simulation_time)){
            break;
        }

        // OK
        ros::spinOnce();
        rate.sleep();
    }

    // Output
    std::cout << "Max in ns is " << max_time << std::endl;
    std::cout << "Average in ns is " << double(average) / double(iteration) << std::endl;
    std::cout << "The iterations number is " << iteration << std::endl;
    std::cout << "" << average << std::endl;
    supervisor.Stop();
}