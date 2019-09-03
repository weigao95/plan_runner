//
// Created by wei on 8/28/19.
//

#include "common/plan_supervisor.h"
#include "common/joint_trajectory_plan.h"


arm_runner::PlanSupervisor::PlanSupervisor(
    std::unique_ptr<RigidBodyTree<double>> tree,
    std::unique_ptr<RobotCommunication> robot_hw,
    ros::NodeHandle nh
) : tree_(std::move(tree)),
    rbt_communication_(std::move(robot_hw)),
    node_handle_(nh)
{
    initializeKinematicAndCache();
    initializeSwitchData();
    initializeServiceActions();
}


void arm_runner::PlanSupervisor::initializeKinematicAndCache() {
    rbt_active_plan_ = nullptr;
    plan_start_time_second_ = 0;
    cache_measured_state = std::make_shared<KinematicsCache<double>>(tree_->CreateKinematicsCache());
    joint_name_to_idx_ = tree_->computePositionNameToIndexMap();
    num_joint_ = tree_->get_num_positions();
}


// The main interface
void arm_runner::PlanSupervisor::Initialize() {
    rbt_communication_->Start();
}

void arm_runner::PlanSupervisor::Stop() {
    rbt_communication_->Stop();
    plan_end_server_->shutdown();
}

void arm_runner::PlanSupervisor::ProcessLoopIteration(double control_peroid_second) {
    // Get measurement
    // Use the time in measurement
    rbt_communication_->GetMeasurement(measurement_cache);
    measurement_cache.time_stamp.since_plan_start_second = 
            measurement_cache.time_stamp.absolute_time_second - plan_start_time_second_;

    // Do computation
    auto q_size = tree_->get_num_positions();
    auto v_size = tree_->get_num_velocities();
    Eigen::Map<Eigen::VectorXd> q = Eigen::Map<Eigen::VectorXd>(measurement_cache.joint_position, q_size);
    Eigen::Map<Eigen::VectorXd> v = Eigen::Map<Eigen::VectorXd>(measurement_cache.joint_velocities, v_size);
    if(measurement_cache.velocity_validity) {
        cache_measured_state->initialize(q, v);
        tree_->doKinematics(*cache_measured_state);
    } else {
        cache_measured_state->initialize(q);
        tree_->doKinematics(*cache_measured_state);
    }

    // Construct input
    CommandInput input;
    input.latest_measurement = &measurement_cache;
    input.robot_history = rbt_communication_.get();
    input.robot_rbt = tree_.get();
    input.measured_state_cache = cache_measured_state.get();
    input.control_interval_second = control_peroid_second;

    // Debug
    auto utime = int64_t(command_cache.time_stamp.absolute_time_second * 1e6);
    if(utime % 100000 <= 10000) {
        ROS_INFO("The measurement position 0 is %f", measurement_cache.joint_position[0]);
    }

    // Compute command
    if(rbt_active_plan_ != nullptr) {
        rbt_active_plan_->ComputeCommand(input, command_cache);
    } else {
        RobotPlanBase::KeepCurrentConfigurationCommand(measurement_cache, command_cache);
    }

    // Software safety check
    bool command_safe = checkCommandSafety(input, command_cache);
    if(!command_safe) {
        // Keep the current pose
        RobotPlanBase::KeepCurrentConfigurationCommand(measurement_cache, command_cache);
    }

    // Send to robot
    command_cache.time_stamp = measurement_cache.time_stamp;
    rbt_communication_->SendCommand(command_cache);

    // Might need to switch the plan
    processPlanSwitch(input, command_cache, command_safe);
}


// For safety check
bool arm_runner::PlanSupervisor::checkCommandSafety(
    const arm_runner::CommandInput &input,
    const arm_runner::RobotArmCommand &command
) {
    // Plan-specific check
    if(rbt_active_plan_ != nullptr) {
        auto plan_safety = rbt_active_plan_->CheckSafety(input, command);
        if(!plan_safety)
            return false;
    }

    // General check
    for(auto& checker : safety_checker_stack_) {
        bool is_safe = true;
        if(checker->HasRequiredField(input, command)) {
            auto result = checker->CheckSafety(input, command);
            is_safe = result.is_safe;
        }
        if(!is_safe)
            return false;
    }

    // Everything OK
    return true;
}
