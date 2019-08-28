//
// Created by wei on 8/28/19.
//

#include "arm_runner/plan_supervisor.h"
#include "arm_runner/time_utils.h"
#include "arm_runner/trajectory_plan.h"
#include <chrono>


void arm_runner::PlanSupervisor::processLoopIter() {
    // Get the time
    TimeStamp now;
    now.absolute_time_second = now_in_second();
    now.since_plan_start_second = now.absolute_time_second - plan_start_time_second_;

    // Get measurement
    rbt_communication_->GetMeasurement(measurement_cache, now);

    // Do computation
    auto q_size = tree_->get_num_positions();
    auto v_size = tree_->get_num_velocities();
    Eigen::Map<Eigen::VectorXd> q = Eigen::Map<Eigen::VectorXd>(measurement_cache.joint_position, q_size);
    Eigen::Map<Eigen::VectorXd> v = Eigen::Map<Eigen::VectorXd>(measurement_cache.joint_velocities, v_size);
    if(measurement_cache.velocity_validity) {
        cache_measured_state.initialize(q, v);
        tree_->doKinematics(cache_measured_state);
    } else {
        cache_measured_state.initialize(q);
        tree_->doKinematics(cache_measured_state);
    }

    // Construct input
    CommandInput input;
    input.latest_measurement = &measurement_cache;
    input.robot_history = rbt_communication_.get();
    input.robot_rbt = tree_;
    input.measured_state_cache = &cache_measured_state;

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
    rbt_communication_->SendCommand(command_cache, now);

    // Invalidate the command if not correct
    if(!command_safe && rbt_active_plan_ != nullptr) {
        rbt_active_plan_->StopPlan();
        rbt_active_plan_.reset();
    }

    // Might need to switch the plan
    processPlanSwitch(input, command_cache);
}

bool arm_runner::PlanSupervisor::shouldSwitchPlan(const RobotArmMeasurement& measurement, const RobotArmCommand& latest_command) const {
    // Commanded to stop
    if (stop_current_) return true;

    // No new plan, cannot switch
    if (!plan_construction_data_.valid) return false;

    // Now we have a new plan
    if (rbt_active_plan_ == nullptr) return true;

    // Now the active plan is not NULL
    if (is_streaming_plan(rbt_active_plan_->GetPlanType()) || rbt_active_plan_->HasFinished(measurement))
        return true;

    // Need to wait the current plan
    return false;
}


arm_runner::RobotPlanBase::Ptr arm_runner::PlanSupervisor::constructNewPlan(
    const CommandInput& input,
    const RobotArmCommand& latest_command
) {
    // If the plan data is valid
    if(!plan_construction_data_.valid)
        return nullptr;

    // Depends on the type
    switch (plan_construction_data_.type) {
        case PlanType::JointTrajectory:{
            /*auto plan = ConstructJointTrajectoryPlan(
                    *tree_,
                    plan_construction_data_.joint_trajectory_goal,
                    measurement, latest_command
            );*/
            //return plan;
        }
        default:
            break;
    }

    // Depends on the type
    return nullptr;
}


void arm_runner::PlanSupervisor::processPlanSwitch(
    const CommandInput& input,
    const RobotArmCommand& latest_command
) {
    // Use a fixed time lock
    using std::chrono::milliseconds;
    if(switch_mutex_.try_lock_for(milliseconds(LOOP_MUTEX_TIMEOUT_MS))) {
        // Now this method has lock
        // Check should I switch
        if (!shouldSwitchPlan(*input.latest_measurement, latest_command)) {
            switch_mutex_.unlock();
            return;
        }

        // Do switching
        if(rbt_active_plan_ != nullptr)
            rbt_active_plan_->StopPlan();

        // Construct and switch to the new one
        rbt_active_plan_ = constructNewPlan(input, latest_command);
        if(rbt_active_plan_ != nullptr) {
            rbt_active_plan_->InitializePlan();
        }

        // Cleanup
        plan_construction_data_.valid = false;
        stop_current_ = false;
        plan_start_time_second_ = now_in_second();

        // Release the lock and return
        switch_mutex_.unlock();
        return;
    } else {
        // No lock
        // Just keep current plan
        return;
    }
}


bool arm_runner::PlanSupervisor::checkCommandSafety(const arm_runner::CommandInput &measurement,
                                                    const arm_runner::RobotArmCommand &command) const {
    return true;
}