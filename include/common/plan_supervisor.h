//
// Created by wei on 8/28/19.
//

#pragma once

#include <mutex>
#include <drake/multibody/rigid_body_tree.h>
#include <actionlib/server/simple_action_server.h>

#include "common/plan_base.h"
#include "common/plan_common_types.h"
#include "common/robot_communication.h"

#include "robot_msgs/CartesianTrajectoryAction.h"
#include "robot_msgs/JointTrajectoryAction.h"
#include "robot_msgs/GetPlanNumberAction.h"
#include "robot_msgs/StartStreamingPlan.h"
#include "std_srvs/Trigger.h"


namespace arm_runner {

    class PlanSupervisor {
    public:
        PlanSupervisor(
            std::unique_ptr<RigidBodyTree<double>> tree,
            std::unique_ptr<RobotCommunication> robot_hw,
            ros::NodeHandle nh);

        // The main interface
        void Start();
        void Stop();
        void ProcessLoopIteration();


        // The real state that can only be mutated in main thread
    private:
        std::unique_ptr<RigidBodyTree<double>> tree_;
        std::unique_ptr<RobotCommunication> rbt_communication_;
        RobotPlanBase::Ptr rbt_active_plan_;
        double plan_start_time_second_;
        void initializeKinematicAndCache();


        // The members for switching, might be accessed by other thread
        // Protected by the lock
    private:
        std::timed_mutex switch_mutex_;
        ActionToCurrentPlan action_to_current_plan_;
        struct PlanConstructionData {
            // The flag
            bool valid;
            PlanType type;
            int plan_number;

            // The actual data
            robot_msgs::JointTrajectoryGoal::ConstPtr joint_trajectory_goal;
            robot_msgs::CartesianTrajectoryGoal::ConstPtr cartesian_trajectory_goal;
        } plan_construction_data_;
        void initializeSwitchData();

        // These method would read the construction data and can only be accessed on main thread
        bool shouldSwitchPlan(const RobotArmMeasurement& measurement, const RobotArmCommand& latest_command) const;
        void processPlanSwitch(const CommandInput& input, const RobotArmCommand& latest_command);

        // The member to maintain the queue of accomplished tasks
    private:
        struct FinishedPlanRecord {
            int plan_number;
            ActionToCurrentPlan action_to_plan;
        };
        struct FinishedPlanQueue {
            std::mutex mutex;
            std::vector<FinishedPlanRecord> queue;
        } finished_task_queue_;
        void lockAndCheckPlanStatus(
                int plan_number,
                bool& current_plan_in_queue,
                bool& new_plan_in_queue,
                ActionToCurrentPlan& action_to_plan);
        void lockAndEnQueue(FinishedPlanRecord record);


        // The cache
    private:
        mutable RobotArmMeasurement measurement_cache;
        mutable RobotArmCommand command_cache;
        mutable std::shared_ptr<KinematicsCache<double>> cache_measured_state;


        // Software safety check
    private:
        bool checkCommandSafety(const CommandInput& measurement, const RobotArmCommand& command) const;


        // The handling function
    public:
        void HandleJointTrajectoryAction(const robot_msgs::JointTrajectoryGoalConstPtr &goal);
        bool HandleEndPlanService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    private:
        ros::NodeHandle node_handle_;
        std::shared_ptr<actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>> joint_trajectory_action_;
        std::shared_ptr<ros::ServiceServer> plan_end_server_;
        void initializeServiceActions();


        // The plan constructor with callbacks
    private:
        RobotPlanBase::Ptr constructNewPlan(const CommandInput& input, const RobotArmCommand& latest_command);
        RobotPlanBase::Ptr constructJointTrajectoryPlan(const CommandInput& input, int plan_number);
    };
}