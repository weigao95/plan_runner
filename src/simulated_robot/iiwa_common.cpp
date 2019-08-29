//
// Created by wei on 8/28/19.
//

#include "simulated_robot/iiwa_common.h"

void arm_runner::SetPositionControlledIiwaGains(Eigen::VectorXd *Kp, Eigen::VectorXd *Ki, Eigen::VectorXd *Kd) {
    // All the gains are for acceleration, not directly responsible for generating
    // torques. These are set to high values to ensure good tracking. These gains
    // are picked arbitrarily.
    Kp->resize(7);
    *Kp << 100, 100, 100, 100, 100, 100, 100;
    Kd->resize(Kp->size());
    for (auto i = 0; i < Kp->size(); i++) {
        // Critical damping gains.
        (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
    }
    *Ki = Eigen::VectorXd::Zero(7);
}