//
// Created by wei on 8/28/19.
//

#pragma once

#include <Eigen/Eigen>

namespace arm_runner {

    // The pid gain for iiwa
    void SetPositionControlledIiwaGains(
        Eigen::VectorXd* Kp,
        Eigen::VectorXd* Ki,
        Eigen::VectorXd* Kd);
}
