//
// Created by wei on 9/10/19.
//

#include "robot_plan/position_integrate_option.h"


const double* plan_runner::PositionIntegratorOption::GetForwardIntegrationJointPosition(
    const plan_runner::CommandInput &input
) const {
    // Use either measured or latest commanded
    const double* q_fwd_integration = nullptr;
    const auto& command_history = input.robot_history->GetCommandHistory();
    if(command_history.empty() || (!use_commanded_fwd_integration_)) {
        q_fwd_integration = input.latest_measurement->joint_position;
    } else {
        q_fwd_integration = command_history.back().joint_position;
    }

    // OK
    return q_fwd_integration;
}


plan_runner::LoadParameterStatus plan_runner::PositionIntegratorOption::LoadParameterFrom(const YAML::Node &datamap) {
    const std::string key = "use_command_position_streaming";
    if(datamap[key]) {
        use_commanded_fwd_integration_ = datamap[key].as<bool>();
        return LoadParameterStatus::Success;
    } else {
        return LoadParameterStatus::NonFatalError;
    }
}


void plan_runner::PositionIntegratorOption::SaveParameterTo(YAML::Node &datamap) const {
    const std::string key = "use_command_position_streaming";
    datamap[key] = use_commanded_fwd_integration_;
}