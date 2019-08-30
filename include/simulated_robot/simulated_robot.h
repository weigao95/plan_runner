//
// Created by wei on 8/28/19.
//

#pragma once

#include "simulated_robot/exchange_type.h"

#include <thread>
#include <drake/multibody/rigid_body_tree.h>

namespace arm_runner {

    // This call will start a drake simulation from another thread
    // The communication is by the exchange data struct.
    class SimulatedRobotArm : public RobotCommunication {
    public:
        SimulatedRobotArm(const std::string& model_urdf, double simulation_time_second);
        SimulatedRobotArm(std::unique_ptr<RigidBodyTree<double>> robot, double simulation_time_second)
        : tree_(std::move(robot)), simulation_time_second_(simulation_time_second) {}
        ~SimulatedRobotArm() override = default;
        void Start() override;
        void Stop() override;

    protected:
        // The data about the scene
        std::unique_ptr<RigidBodyTree<double>> tree_;

        // The shared data
        double simulation_time_second_;
        SimulationExchangeData exchanged_data_;
        void getRawMeasurement(RobotArmMeasurement& measurement) override;
        void sendRawCommand(const RobotArmCommand& command) override;

        // The threaded simulation
        void runSimulation();
        std::thread simulation_thread_;
    };
}
