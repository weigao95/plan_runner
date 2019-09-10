//
// Created by wei on 8/29/19.
//

#pragma once

#include "common/processor_interface.h"

#include <ros/ros.h>

namespace plan_runner {


    class JointPublishProcessor : public MeasurementProcessor {
    public:
        using Ptr = std::shared_ptr<JointPublishProcessor>;
        JointPublishProcessor(
            ros::NodeHandle nh,
            const std::string& topic,
            std::vector<std::string> joint_name_list,
            int queue_size = 1000);
        ~JointPublishProcessor() override = default;
        void ProcessMeasurement(const RobotCommunication&, RobotArmMeasurement&) override;
    private:
        std::vector<std::string> joint_name_list_;
        ros::NodeHandle node_handle_;
        ros::Publisher joint_state_publisher_;
    };

}
