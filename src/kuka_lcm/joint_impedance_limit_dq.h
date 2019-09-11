//
// Created by wei on 9/11/19.
//

#pragma once

#include <memory>
#include "common/processor_interface.h"


namespace plan_runner {

    class JointImpedanceControllerLimitDq : public CommandProcessor {
    public:
        using Ptr = std::shared_ptr<JointImpedanceControllerLimitDq>;
        explicit JointImpedanceControllerLimitDq(double max_dq=0.02) : max_dq_(max_dq) {}

        // The processing interface
        void ProcessCommand(const RobotCommunication&, RobotArmCommand&) override;
    private:
        double max_dq_;
    };

}
