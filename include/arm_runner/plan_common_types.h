//
// Created by wei on 8/27/19.
//

#pragma once

#include "arm_runner/communication_types.h"
#include "arm_runner/robot_communication.h"
#include <drake/multibody/rigid_body_tree.h>
#include <drake/multibody/kinematics_cache.h>

namespace arm_runner {

    // The status of the plan
    enum class PlanStatus {
        Waiting,
        Running,
        Stopped
    };

    // The semantic of the plan
    // The download plan MUST stay at the final knot after the time of trajectory
    enum class PlanType {
        JointTrajectory,
        JointPositionStreaming,
        JointTorqueStreaming,
        EETrajectory,
        EEConfigurationStreaming,
        EEVelocityStreaming,
        EEForceStreaming
    };

    enum class ActionToCurrentPlan {
        NoAction,
        NormalStop,
        SafetyStop,
        PreemptStop
    };

    inline bool is_streaming_plan(PlanType type) {
        return type == PlanType::JointPositionStreaming 
                || type == PlanType::JointTorqueStreaming
                || type == PlanType::EEConfigurationStreaming
                || type == PlanType::EEVelocityStreaming 
                || type == PlanType::EEForceStreaming;
    }

    // The input to command
    struct CommandInput {
        const RobotArmMeasurement* latest_measurement;
        const RobotCommunication* robot_history;
        const RigidBodyTree<double>* robot_rbt;
        const KinematicsCache<double>* measured_state_cache;
    };
}
