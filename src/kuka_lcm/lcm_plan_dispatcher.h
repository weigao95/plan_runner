//
// Created by wei on 9/9/19.
//

#pragma once

#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <lcm/lcm-cpp.hpp>
#include <robotlocomotion/robot_plan_t.hpp>

#include "robot_msgs/JointTrajectoryAction.h"


namespace plan_runner {

    class KukaPlanDispatcherLCM {
    public:
        KukaPlanDispatcherLCM(
            ros::NodeHandle& nh,
            std::string lcm_plan_channel, std::string lcm_stop_channel,
            const std::string& joint_trajectory_action, const std::string& stop_plan_service);
        ~KukaPlanDispatcherLCM() = default;
        void Start();

        // The handler for lcm plan
    public:
        void handleJointTrajectoryPlan(
            const lcm::ReceiveBuffer *,
            const std::string &,
            const robotlocomotion::robot_plan_t *tape);
        void handleStopPlan(
            const lcm::ReceiveBuffer *,
            const std::string &,
            const robotlocomotion::robot_plan_t *);
    private:
        std::string lcm_plan_channel_;
        std::string lcm_stop_channel_;

        // The ros client
    private:
        ros::NodeHandle node_handle_;
        std::mutex mutex_;
        std::shared_ptr<actionlib::SimpleActionClient<robot_msgs::JointTrajectoryAction>> joint_trajectory_client_;
        std::shared_ptr<ros::ServiceClient> stop_plan_client_;
    };
}