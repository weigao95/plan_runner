#pragma once


template<typename ActionT, typename ResultT>
void plan_runner::PlanSupervisor::appendAndWaitForTrajectoryPlan(
    std::shared_ptr<actionlib::SimpleActionServer<ActionT>>& action_server,
    const RobotPlanBase::Ptr& plan,
    int wait_result_interval
) {
    // The construction should be OK
    int current_plan_number;
    lockAndAppendPlan(plan, current_plan_number);

    // Wait for the task being accomplished
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_result_interval));

        // Read the status
        bool current_plan_in_queue = false;
        bool new_plan_in_queue = false;
        ActionToCurrentPlan action_to_current_plan = ActionToCurrentPlan::NoAction;
        finished_plan_queue_.LockAndCheckPlanStatus(
            current_plan_number,
            current_plan_in_queue,
            new_plan_in_queue,
            action_to_current_plan);

        // Determine the state
        if(current_plan_in_queue) {
            ResultT result;
            if(action_to_current_plan == ActionToCurrentPlan::NoAction
               || action_to_current_plan == ActionToCurrentPlan::NormalStop) {
                result.status.status = result.status.FINISHED_NORMALLY;
                action_server->setSucceeded(result);
            } else {
                result.status.status = result.status.STOPPED_BY_SAFETY_CHECK;
                action_server->setAborted(result);
            }
            return;
        } else if ((!current_plan_in_queue) && new_plan_in_queue) {
            ResultT result;
            result.status.status = result.status.STOPPED_BY_EXTERNAL_TRIGGER;
            action_server->setPreempted(result);
            return;
        } else
            continue;

    } while(true);
}
