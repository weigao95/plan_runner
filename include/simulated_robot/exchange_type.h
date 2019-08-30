//
// Created by wei on 8/28/19.
//

#pragma once

#include "common/communication_types.h"
#include "common/robot_communication.h"

#include <mutex>

namespace arm_runner {

    // The exchanged data between drake simulation and robot communication
    struct SimulationExchangeData {
        std::mutex mutex;
        RobotArmMeasurement latest_measurement;
        RobotArmCommand latest_command;
    };
}