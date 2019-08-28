//
// Created by wei on 8/28/19.
//

#pragma once

#include <mutex>
#include <drake/multibody/rigid_body_tree.h>

#include "arm_runner/plan_base.h"
#include "arm_runner/plan_common_types.h"
#include "arm_runner/robot_communication.h"

#include "robot_msgs/CartesianTrajectoryAction.h"
#include "robot_msgs/JointTrajectoryAction.h"
#include "robot_msgs/GetPlanNumberAction.h"
#include "robot_msgs/StartStreamingPlan.h"


namespace arm_runner {

    class PlanSupervisor {
    protected:
        // Software safety check
        bool checkCommandSafety(const CommandInput& measurement, const RobotArmCommand& command) const;

        // The processing loop
        void processLoopIter();

    private:
        // The real state
        // Can only be mutated in main thread
        std::shared_ptr<RigidBodyTree<double>> tree_;
        std::unique_ptr<RobotCommunication> rbt_communication_;
        RobotPlanBase::Ptr rbt_active_plan_;
        double plan_start_time_second_;

        // The field for switching, might be accessed by other thread
        // Protected by the lock
        std::timed_mutex switch_mutex_;
        static constexpr int LOOP_MUTEX_TIMEOUT_MS = 5;
        bool stop_current_;
        struct PlanConstructionData {
            // The flag
            bool valid;
            PlanType type;

            // The actual data
            robot_msgs::JointTrajectoryGoal::ConstPtr joint_trajectory_goal;
            robot_msgs::CartesianTrajectoryGoal::ConstPtr cartesian_trajectory_goal;
        } plan_construction_data_;

        // These method would read the construction data and can only be accessed on main thread
        bool shouldSwitchPlan(const RobotArmMeasurement& measurement, const RobotArmCommand& latest_command) const;
        void processPlanSwitch(const CommandInput& input, const RobotArmCommand& latest_command);
        RobotPlanBase::Ptr constructNewPlan(const CommandInput& input, const RobotArmCommand& latest_command);

        // The cache
        RobotArmMeasurement measurement_cache;
        RobotArmCommand command_cache;
        KinematicsCache<double> cache_measured_state;

        // The handling function
    public:
        void HandleJointTrajectoryAction(const robot_msgs::JointTrajectoryGoal::ConstPtr &goal);
    private:
        RobotPlanBase::Ptr constructJointTrajectoryPlan(const CommandInput& input);
    };
}