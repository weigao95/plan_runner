//
// Created by wei on 11/11/19.
//

#pragma once

#include <memory>
#include <ros/ros.h>
#include <drake/multibody/rigid_body_tree.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>


namespace plan_runner {

    class EEForceTorqueEstimator {
    public:
        using Ptr = std::shared_ptr<EEForceTorqueEstimator>;
        EEForceTorqueEstimator(
            ros::NodeHandle& nh,
            std::unique_ptr<RigidBodyTree<double>> tree,
            std::string estimation_publish_topic = "/plan_runner/estimated_ee_force",
            std::string reinit_service_name = "/plan_runner/ee_estimator_reset",
            std::string joint_state_topic="/joint_states",
            std::string ee_frame_id="iiwa_link_ee");
        ~EEForceTorqueEstimator();

        void Initialize();
        void onReceiveJointState(const sensor_msgs::JointState::ConstPtr& joint_state);
        bool onReinitServiceRequeat(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    private:
        // The meta info
        ros::NodeHandle node_handle_;
        std::string ee_frame_id_;
        std::string joint_state_topic_;
        std::string estimation_publish_topic_;
        std::string reinit_offset_srv_name_;

        // The topic for publishing and receiving
        std::shared_ptr<ros::Subscriber> joint_state_subscriber_;
        ros::Publisher estimation_publisher_;
        std::shared_ptr<ros::ServiceServer> reinit_offset_server_;

        // The actual computation
    protected:
        std::unique_ptr<RigidBodyTree<double>> tree_;
        void estimateEEForceTorque(
            const sensor_msgs::JointState::ConstPtr& joint_state,
            Eigen::Vector3d& force_in_world, Eigen::Vector3d& torque_in_world);

        // The offset of parameter
        std::mutex mutex_;
        Eigen::VectorXd torque_offset_;
        bool offset_valid_;

        // The low pass filtering
        Eigen::Vector3d prev_force_;
        Eigen::Vector3d prev_torque_;
        bool prev_valid_;
        void filterEstimatedForceTorque(Eigen::Vector3d& force_in_world, Eigen::Vector3d& torque_in_world);
    };
}