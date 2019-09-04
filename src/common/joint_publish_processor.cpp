//
// Created by wei on 8/29/19.
//

#include "common/joint_publish_processor.h"

#include <sensor_msgs/JointState.h>

arm_runner::JointPublishProcessor::JointPublishProcessor(
    ros::NodeHandle nh,
    const std::string &topic,
    std::vector<std::string> joint_name_list,
    int queue_size
) : node_handle_(nh), joint_name_list_(std::move(joint_name_list)) {
    joint_state_publisher_ = node_handle_.advertise<sensor_msgs::JointState>(topic, queue_size);
}

void arm_runner::JointPublishProcessor::ProcessMeasurement(
    const arm_runner::RobotCommunication &,
    arm_runner::RobotArmMeasurement & measurement
) {
    sensor_msgs::JointState joint_state;
    // The time and name
    joint_state.header.stamp = measurement.time_stamp.GetROSTimeStamp();
    joint_state.name = joint_name_list_;

    // Other properties
    joint_state.position.clear();
    joint_state.velocity.clear();
    joint_state.effort.clear();
    for(auto i = 0; i < joint_name_list_.size(); i++) {
        if(measurement.position_validity)
            joint_state.position.push_back(measurement.joint_position[i]);
        if(measurement.velocity_validity)
            joint_state.velocity.push_back(measurement.joint_velocities[i]);
        if(measurement.torque_validity)
            joint_state.effort.push_back(measurement.joint_torque[i]);
    }

    // Send the msg
    joint_state_publisher_.publish(joint_state);
}

