//
// Created by wei on 8/28/19.
//

#include "arm_runner/plan_supervisor.h"
#include "arm_runner/trajectory_plan.h"


arm_runner::PlanSupervisor::PlanSupervisor(
    std::shared_ptr<RigidBodyTree<double>> tree,
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
}


// The main interface
void arm_runner::PlanSupervisor::Start() {
    rbt_communication_->Start();
    joint_trajectory_action_->start();
}

void arm_runner::PlanSupervisor::Stop() {
    rbt_communication_->Stop();
    plan_end_server_->shutdown();
}

void arm_runner::PlanSupervisor::ProcessLoopIteration() {
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
    input.robot_rbt = tree_;
    input.measured_state_cache = cache_measured_state.get();

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
    rbt_communication_->SendCommand(command_cache);

    // Invalidate the command if not correct
    if(!command_safe && rbt_active_plan_ != nullptr) {
        action_to_current_plan_ = ActionToCurrentPlan::SafetyStop;

        //Remove these?
        rbt_active_plan_->StopPlan(action_to_current_plan_);
        rbt_active_plan_.reset();
    }

    // Might need to switch the plan
    processPlanSwitch(input, command_cache);
}


// For safety check
bool arm_runner::PlanSupervisor::checkCommandSafety(const arm_runner::CommandInput &measurement,
                                                    const arm_runner::RobotArmCommand &command) const {
    return true;
}


// The plan constructors
arm_runner::RobotPlanBase::Ptr arm_runner::PlanSupervisor::constructNewPlan(
        const CommandInput& input,
        const RobotArmCommand& latest_command
) {
    // If the plan data is valid
    if(!plan_construction_data_.valid)
        return nullptr;

    // Depends on the type
    switch (plan_construction_data_.type) {
        case PlanType::JointTrajectory: {
            return constructJointTrajectoryPlan(input);
        }
        default:
            break;
    }

    // Depends on the type
    return nullptr;
}


arm_runner::RobotPlanBase::Ptr arm_runner::PlanSupervisor::constructJointTrajectoryPlan(
    const CommandInput& input
) {
    const auto& latest_command = input.robot_history->GetCommandHistory().back();
    auto plan = ConstructJointTrajectoryPlan(
        *input.robot_rbt,
        plan_construction_data_.joint_trajectory_goal,
        *input.latest_measurement,
        latest_command);

    // The finish callback
    auto finish_callback = [this](RobotPlanBase* robot_plan, ActionToCurrentPlan action) -> void {
        robot_msgs::JointTrajectoryResult result;
        if(action == ActionToCurrentPlan::NoAction || action == ActionToCurrentPlan::NormalStop)
            result.status.status = result.status.FINISHED_NORMALLY;
        else if(action == ActionToCurrentPlan::SafetyStop)
            result.status.status = result.status.STOPPED_BY_SAFETY_CHECK;
        else
            result.status.status = result.status.ERROR;

        // Send back
        this->joint_trajectory_action_->setSucceeded(result);
    };
    plan->AddStoppedCallback(finish_callback);

    // OK
    return plan;
}
