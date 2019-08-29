//
// Created by wei on 8/28/19.
//

#pragma once

#include "exchange_type.h"

namespace arm_runner {

    // This call will start a drake simulation from another thread
    // The communication is by the exchange data struct.
    class SimulatedRobotArm : public RobotCommunication {
    protected:
        // The shared data
        SimulationExchangeData exchanged_data_;
        void getRawMeasurement(RobotArmMeasurement& measurement) override;
        void sendRawCommand(const RobotArmCommand& command) override;
    };
}
