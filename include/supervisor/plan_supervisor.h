//
// Created by wei on 8/28/19.
//

#pragma once

#include <mutex>
#include <drake/multibody/rigid_body_tree.h>
#include <actionlib/server/simple_action_server.h>

#include "common/plan_base.h"
#include "common/plan_common_types.h"
#include "common/switch_utility.h"
#include "common/robot_communication.h"
#include "common/safety_checker_interface.h"

#include "robot_msgs/CartesianTrajectoryAction.h"
#include "robot_msgs/JointTrajectoryAction.h"
#include "robot_msgs/GetPlanNumberAction.h"
#include "robot_msgs/StartStreamingPlan.h"
#include "std_srvs/Trigger.h"


namespace plan_runner {

    class PlanSupervisor {
    public:
        PlanSupervisor(
            std::unique_ptr<RigidBodyTree<double>> tree,
            std::unique_ptr<RobotCommunication> robot_hw,
            const ros::NodeHandle& nh,
            const YAML::Node& node = YAML::Node());

        // The main interface
        void Initialize();
        void Stop();
        void ProcessLoopIteration(double control_peroid_second);


        // The state that can only be mutated in main thread
        // Constant state can be accessed everywhere, of course
    private:
        std::unique_ptr<RigidBodyTree<double>> tree_;
        std::unique_ptr<RobotCommunication> rbt_communication_;
        RobotPlanBase::Ptr rbt_active_plan_;
        const YAML::Node parameter_map_;
        void initializeKinematicAndCache();


        // The members for switching, might be accessed by other thread
        // Protected by the lock
    private:
        std::mutex switch_mutex_;
        ActionToCurrentPlan action_to_current_plan_;
        RobotPlanBase::Ptr switch_to_plan_;
        int plan_number_;

        // The queue of finished plans
        FinishedPlanQueue finished_plan_queue_;
    private:
        // These method would read the construction data and can only be accessed on main thread
        void initializeSwitchData();
        bool shouldSwitchPlan(const RobotArmMeasurement& measurement, const RobotArmCommand& latest_command) const;
        void processPlanSwitch(const CommandInput& input, const RobotArmCommand& latest_command, bool command_safety);


        // Software safety check
        // The safety checkers here are plan-independent
    public:
        void AddSafetyChecker(SafetyChecker::Ptr checker) { safety_checker_stack_.emplace_back(std::move(checker)); };
    protected:
        std::vector<SafetyChecker::Ptr> safety_checker_stack_;
        bool checkCommandSafety(const CommandInput& measurement, const RobotArmCommand& command);


        // The handling function
    public:
        void HandleJointTrajectoryAction(const robot_msgs::JointTrajectoryGoal::ConstPtr& goal);
        void HandleEETrajectoryAction(const robot_msgs::CartesianTrajectoryGoal::ConstPtr& goal);
        bool HandleEndPlanService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool HandleStartStreamingService(
            robot_msgs::StartStreamingPlan::Request& request,
            robot_msgs::StartStreamingPlan::Response& response);
    private:
        ros::NodeHandle node_handle_;
        std::shared_ptr<actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>> joint_trajectory_action_;
        std::shared_ptr<actionlib::SimpleActionServer<robot_msgs::CartesianTrajectoryAction>> ee_trajectory_action_;
        std::shared_ptr<ros::ServiceServer> plan_end_server_;
        std::shared_ptr<ros::ServiceServer> start_streaming_server_;
        void initializeServiceActions();
        void lockAndAppendPlan(const RobotPlanBase::Ptr& plan, int& plan_number);

        // Wait for the result
        template<typename ActionT, typename ResultT>
        void appendAndWaitForTrajectoryPlan(
            std::shared_ptr<actionlib::SimpleActionServer<ActionT>>& action_server,
            const RobotPlanBase::Ptr& plan,
            int wait_result_interval);

        // The cache
    private:
        mutable RobotArmMeasurement measurement_cache;
        mutable RobotArmCommand command_cache;
        mutable std::shared_ptr<KinematicsCache<double>> cache_measured_state;
    };
}

// The templated method
#include "supervisor/plan_supervisor_handler.hpp"