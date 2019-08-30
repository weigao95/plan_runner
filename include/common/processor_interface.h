//
// Created by wei on 8/29/19.
//

#pragma once

#include <memory>
#include "common/communication_types.h"

namespace arm_runner {

    // FWD declaration
    class RobotCommunication;

    // The command processor
    class CommandProcessor {
    public:
        using Ptr = std::shared_ptr<CommandProcessor>;
        virtual ~CommandProcessor() = default;
        virtual void ProcessCommand(const RobotCommunication&, RobotArmCommand&) = 0;
    };

    // The measurement processor
    class MeasurementProcessor {
    public:
        using Ptr = std::shared_ptr<MeasurementProcessor>;
        virtual ~MeasurementProcessor() = default;
        virtual void ProcessMeasurement(const RobotCommunication&, RobotArmMeasurement&) = 0;
    };
}
