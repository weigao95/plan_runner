//
// Created by wei on 8/28/19.
//

#pragma once

#include "arm_runner/communication_types.h"
#include "arm_runner/robot_communication.h"

#include <mutex>

namespace arm_runner {

    // The exchanged data between drake simulation and robot communication
    struct SimulationExchangeData {
        std::mutex mutex;
        RobotArmMeasurement latest_measurement;
        RobotArmCommand latest_command;
    };
}