//
// Created by wei on 8/27/19.
//

#pragma once

#include "common/communication_types.h"
#include "common/robot_communication.h"
#include <drake/multibody/rigid_body_tree.h>
#include <drake/multibody/kinematics_cache.h>

namespace arm_runner {

    // The semantic of the plan
    // The download plan MUST stay at the final knot after the time of trajectory
    enum class PlanType {
        KeepCurrentConfigurationPlan,
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

    inline bool will_plan_stop_internally(PlanType type) {
        return (!is_streaming_plan(type)) && (type != PlanType::KeepCurrentConfigurationPlan);
    }

    // The input to command
    struct CommandInput {
        double control_interval_second;
        const RobotArmMeasurement* latest_measurement;
        const RobotCommunication* robot_history;
        const RigidBodyTree<double>* robot_rbt;
        const KinematicsCache<double>* measured_state_cache;

        bool is_valid() const {
            return latest_measurement != nullptr
            && robot_history != nullptr
            && robot_rbt != nullptr
            && measured_state_cache != nullptr
            && control_interval_second > 0.0;
        }
    };
}
