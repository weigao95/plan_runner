//
// Created by wei on 8/28/19.
//

#include "simulated_robot.h"

#include <drake/multibody/rigid_body_tree.h>

void arm_runner::SimulatedRobotArm::getRawMeasurement(RobotArmMeasurement &measurement) {
    std::lock_guard<std::mutex> guard(exchanged_data_.mutex);
    DRAKE_ASSERT(exchanged_data_.latest_measurement.is_valid());
    measurement = exchanged_data_.latest_measurement;
}

void arm_runner::SimulatedRobotArm::sendRawCommand(const arm_runner::RobotArmCommand &command) {
    std::lock_guard<std::mutex> guard(exchanged_data_.mutex);
    DRAKE_ASSERT(command.is_valid());
    exchanged_data_.latest_command = command;
}
