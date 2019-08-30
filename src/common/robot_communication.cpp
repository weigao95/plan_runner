//
// Created by wei on 8/27/19.
//

#include "common/robot_communication.h"

arm_runner::RobotCommunication::RobotCommunication()
: measurement_history_(RobotCommunication::HISTORY_HORIZON),
  command_history_(RobotCommunication::HISTORY_HORIZON) {}

void arm_runner::RobotCommunication::GetMeasurement(RobotArmMeasurement& measurement) {
    // Get the raw measurement
    getRawMeasurement(measurement);

    // Do processing
    for(const auto& processor : measurement_processor_stack_) {
        processor(measurement);
    }

    // Save
    measurement_history_.push_back(measurement);
}

void arm_runner::RobotCommunication::SendCommand(const RobotArmCommand & command_in) {
    // Copy and add time
    auto command = command_in;

    // Do processing
    for(const auto& processor : command_processor_stack_) {
        processor(command);
    }

    // Save
    command_history_.push_back(command);

    // Handle to robot
    sendRawCommand(command_history_.back());
}
