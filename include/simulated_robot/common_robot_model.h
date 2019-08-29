//
// Created by wei on 8/29/19.
//

#pragma once

#include <memory>
#include "simulated_robot/simulated_robot.h"


namespace arm_runner {

    // The "bare" kuka model
    std::unique_ptr<SimulatedRobotArm> constructSimulatedKukaDefault(double simulation_time_second = 10);
}