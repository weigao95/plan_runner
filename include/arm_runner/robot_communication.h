//
// Created by wei on 8/27/19.
//

#pragma once

#include "arm_runner/communication_types.h"

#include <vector>
#include <functional>
#include <boost/circular_buffer.hpp>

namespace arm_runner {

    class RobotCommunication {
    public:
        static constexpr int HISTORY_HORIZON = 100;
        RobotCommunication();
        virtual ~RobotCommunication() = default;

        // The measure and command interface
        void GetMeasurement(RobotArmMeasurement& measurement);
        void SendCommand(const RobotArmCommand& command);

    protected:
        // The actual interface that communicate with the robot
        virtual void getRawMeasurement(RobotArmMeasurement& measurement) = 0;
        virtual void sendRawCommand(const RobotArmCommand& command) = 0;

        // The buffer hold the history measurement and command
        boost::circular_buffer<RobotArmMeasurement> measurement_history_;
        boost::circular_buffer<RobotArmCommand> command_history_;

        // The processor for command and measurement
        using CommandProcessor = std::function<void(RobotArmCommand&)>;
        using MeasurementProcessor = std::function<void(RobotArmMeasurement&)>;
        std::vector<CommandProcessor> command_processor_stack_;
        std::vector<MeasurementProcessor> measurement_processor_stack_;
    };

}