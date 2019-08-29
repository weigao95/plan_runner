//
// Created by wei on 8/28/19.
//

#pragma once

#include "exchange_type.h"
#include <thread>

namespace arm_runner {

    // This call will start a drake simulation from another thread
    // The communication is by the exchange data struct.
    class SimulatedRobotArm : public RobotCommunication {
    public:
        explicit SimulatedRobotArm(double simulation_time_second = 10) : simulation_time_second_(simulation_time_second) {};
        ~SimulatedRobotArm() override = default;
        void Start() override;
        void Stop() override;

    protected:
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
