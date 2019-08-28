//
// Created by wei on 8/27/19.
//

#pragma once

#include <cstring>

#include "arm_runner/time_utils.h"

namespace arm_runner {

    // The max number of joints for a robot arm
    constexpr int MAX_NUM_JOINTS = 10;

    // A struct the the robot interface should write to
    // that manages the robot state information
    struct RobotArmMeasurement {
        // The time record, must be valid
        TimeStamp time_stamp;

        // The position measurement q
        double joint_position[MAX_NUM_JOINTS];
        bool position_validity;

        // The velocity measurement dq
        // Might comes from numerical difference
        double joint_velocities[MAX_NUM_JOINTS];
        bool velocity_validity;

        // The acceleration measurement ddq
        // Might comes from numerical difference
        double joint_acceleration[MAX_NUM_JOINTS];
        bool acceleration_validity;

        // The joint torque measurement
        double joint_torque[MAX_NUM_JOINTS];
        bool torque_validity;

        // Mark everything as invalid
        void SetInvalid() {
            memset(this, 0, sizeof(RobotArmMeasurement));
            position_validity = false;
            velocity_validity = false;
            acceleration_validity = false;
            torque_validity = false;
        }
    };

    // A struct the the robot interface should write to
    // that comes from external controller
    struct RobotArmCommand {
        // The time record
        TimeStamp time_stamp;

        // The position command q_desired
        double joint_position[MAX_NUM_JOINTS];
        bool position_validity;

        // The velocity command dq_desired
        double joint_velocities[MAX_NUM_JOINTS];
        bool velocity_validity;

        // The joint torque command tau_desired
        double joint_torque[MAX_NUM_JOINTS];
        bool torque_validity;

        // Mark everything as invalid
        void SetInvalid() {
            memset(this, 0, sizeof(RobotArmCommand));
            position_validity = false;
            velocity_validity = false;
            torque_validity = false;
        }
    };
}
