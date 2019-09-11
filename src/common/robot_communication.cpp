//
// Created by wei on 8/27/19.
//

#include "common/robot_communication.h"

plan_runner::RobotCommunication::RobotCommunication()
: measurement_history_(RobotCommunication::HISTORY_HORIZON),
  command_history_(RobotCommunication::HISTORY_HORIZON) {}


void plan_runner::RobotCommunication::GetMeasurement(RobotArmMeasurement& measurement) {
    // Get the raw measurement
    getRawMeasurement(measurement);

    // Do processing
    for(const auto& processor : measurement_processor_stack_) {
        processor->ProcessMeasurement(*this, measurement);
    }

    // Save
    measurement_history_.push_back(measurement);
}


void plan_runner::RobotCommunication::SendCommand(const RobotArmCommand & command_in) {
    // Save
    command_history_.push_back(command_in);

    // Copy and add time
    command_cache = command_in;

    // Do processing
    for(const auto& processor : command_processor_stack_) {
        processor->ProcessCommand(*this, command_cache);
    }

    // Handle to robot
    sendRawCommand(command_cache);
}
