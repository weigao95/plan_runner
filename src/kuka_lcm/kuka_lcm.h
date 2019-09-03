//
// Created by wei on 9/2/19.
//

#pragma once

#include "common/robot_communication.h"


namespace arm_runner {


    class KukaLCMInterface : public RobotCommunication {
    public:

    private:
        // The communication with lcm
        std::string lcm_status_channel_;
        std::string lcm_command_channel_;
    };

}