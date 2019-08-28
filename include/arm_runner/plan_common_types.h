//
// Created by wei on 8/27/19.
//

#pragma once

namespace arm_runner {

    // The status of the plan
    enum class PlanStatus {
        Waiting,
        Running,
        Preempted,
        Stopped
    };

    // The semantic of the plan
    enum class PlanType {
        KeepCurrentConfiguration,
        JointTrajectoryDownload,
        JointTrajectoryStreaming,
        JointTorqueStreaming,
        EETrajectoryDownload,
        EETrajectoryStreaming,
        EEVelocityStreaming,
        EEForceStreaming
    };
}
