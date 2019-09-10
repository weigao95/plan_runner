//
// Created by wei on 8/27/19.
//

#pragma once

#include "common/communication_types.h"
#include "common/processor_interface.h"

#include <vector>
#include <functional>
#include <boost/circular_buffer.hpp>

namespace plan_runner {

    class RobotCommunication {
    public:
        static constexpr int HISTORY_HORIZON = 100;
        RobotCommunication();
        virtual ~RobotCommunication() = default;

        // After start, the robot should accept command (setup connection)
        // After stop, the robot should not accept command (cleanup connection)
        virtual void Start() {}
        virtual void Stop() {}

        // The measure and command interface
        // These functions would only be invoked on main thread and do NOT need to be thread-safe
        void GetMeasurement(RobotArmMeasurement& measurement);
        void SendCommand(const RobotArmCommand& command);

    protected:
        // The actual interface that communicate with the robot
        // These functions would only be invoked on main thread and do NOT need to be thread-safe
        virtual void getRawMeasurement(RobotArmMeasurement& measurement) = 0;
        virtual void sendRawCommand(const RobotArmCommand& command) = 0;

        // The buffer hold the history measurement and command
        boost::circular_buffer<RobotArmMeasurement> measurement_history_;
        boost::circular_buffer<RobotArmCommand> command_history_;

        // The processor for command and measurement
        // Should be constructed before Start
    public:
        void AddMeasurementProcessor(MeasurementProcessor::Ptr processor) { measurement_processor_stack_.emplace_back(std::move(processor)); };
        void AddCommandProcessor(CommandProcessor::Ptr processor) { command_processor_stack_.emplace_back(std::move(processor)); };
    protected:
        // The stack of processor
        std::vector<CommandProcessor::Ptr> command_processor_stack_;
        std::vector<MeasurementProcessor::Ptr> measurement_processor_stack_;


        // Simple getter
        // Should only be accessed on the main thread
    public:
        const decltype(measurement_history_)& GetMeasurementHistory() const { return measurement_history_; }
        const decltype(command_history_)& GetCommandHistory() const { return command_history_; }
    };
}