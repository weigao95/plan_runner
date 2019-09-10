//
// Created by wei on 9/3/19.
//

#include "common/switch_utility.h"


void plan_runner::FinishedPlanQueue::LockAndAppend(plan_runner::FinishedPlanQueue::Record record) {
    std::lock_guard<std::mutex> guard(mutex);
    queue.emplace_back(record);
}


void plan_runner::FinishedPlanQueue::LockAndCheckPlanStatus(
    int plan_number,
    bool &current_plan_in_queue,
    bool &new_plan_in_queue,
    plan_runner::ActionToCurrentPlan &action_to_plan
) {
    std::lock_guard<std::mutex> guard(mutex);
    for(const auto& finished_plan : queue) {
        if(finished_plan.plan_number == plan_number) {
            current_plan_in_queue = true;
            action_to_plan = finished_plan.action_to_plan;
        } else if(finished_plan.plan_number > plan_number)
            new_plan_in_queue = true;
    }
}