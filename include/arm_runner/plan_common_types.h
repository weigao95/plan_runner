//
// Created by wei on 8/27/19.
//

#pragma once

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

    inline bool is_streaming_plan(PlanType type) {
        return type == PlanType::JointPositionStreaming 
                || type == PlanType::JointTorqueStreaming
                || type == PlanType::EEConfigurationStreaming
                || type == PlanType::EEVelocityStreaming 
                || type == PlanType::EEForceStreaming;
    }
}
