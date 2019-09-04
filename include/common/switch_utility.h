//
// Created by wei on 9/3/19.
//

#pragma once

#include <mutex>

#include "common/plan_common_types.h"

namespace arm_runner {


    // The queue of finished plan
    class FinishedPlanQueue {
    public:
        // The record type
        struct Record {
            int plan_number;
            ActionToCurrentPlan action_to_plan;
        };

        // Simple methods
        void Initialize() { queue.clear(); }
        void LockAndAppend(Record record);
        void LockAndCheckPlanStatus(
            int plan_number,
            bool& current_plan_in_queue,
            bool& new_plan_in_queue,
            ActionToCurrentPlan& action_to_plan);

    private:
        // The member
        std::mutex mutex;
        std::vector<Record> queue;
    };
}
