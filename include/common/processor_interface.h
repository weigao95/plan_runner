//
// Created by wei on 8/29/19.
//

#pragma once

#include <memory>

#include "common/yaml_serializable.h"
#include "common/communication_types.h"

namespace arm_runner {

    // FWD declaration
    class RobotCommunication;

    // The command processor
    class CommandProcessor : public YamlSerializableParameter {
    public:
        using Ptr = std::shared_ptr<CommandProcessor>;
        ~CommandProcessor() override = default;
        virtual void ProcessCommand(const RobotCommunication&, RobotArmCommand&) = 0;
    };

    // The measurement processor
    class MeasurementProcessor : public YamlSerializableParameter {
    public:
        using Ptr = std::shared_ptr<MeasurementProcessor>;
        ~MeasurementProcessor() override = default;
        virtual void ProcessMeasurement(const RobotCommunication&, RobotArmMeasurement&) = 0;
    };
}
