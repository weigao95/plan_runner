//
// Created by wei on 8/27/19.
//

#pragma once

#include "arm_runner/communication_types.h"
#include <boost/circular_buffer.hpp>

namespace arm_runner {

    class RobotCommunication {
    public:
    protected:
        virtual void getRawMeasurement(RobotArmMeasurement& measurement) = 0;
        virtual void sendRawCommand(const RobotArmCommand& command) = 0;
    };

}