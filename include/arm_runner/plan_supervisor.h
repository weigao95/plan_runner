//
// Created by wei on 8/28/19.
//

#pragma once

#include <mutex>
#include <drake/multibody/rigid_body_tree.h>
#include <actionlib/server/simple_action_server.h>

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
    private:
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
        void initializeSwitchData();

        // These method would read the construction data and can only be accessed on main thread
        bool shouldSwitchPlan(const RobotArmMeasurement& measurement, const RobotArmCommand& latest_command) const;
        void processPlanSwitch(const CommandInput& input, const RobotArmCommand& latest_command);


        // The cache
    private:
        mutable RobotArmMeasurement measurement_cache;
        mutable RobotArmCommand command_cache;
        mutable KinematicsCache<double> cache_measured_state;


        // The handling function
    public:
        void HandleJointTrajectoryAction(const robot_msgs::JointTrajectoryGoal::ConstPtr &goal);
    private:
        ros::NodeHandle node_handle_;
        std::shared_ptr<actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>> joint_trajectory_action_;
        void initializeActions();


        // The plan constructor with callbacks
    private:
        RobotPlanBase::Ptr constructNewPlan(const CommandInput& input, const RobotArmCommand& latest_command);
        RobotPlanBase::Ptr constructJointTrajectoryPlan(const CommandInput& input);
    };
}