#pragma once 

namespace arm_runner {

    // The record used for plan runner
    struct TimeStamp {
        double absolute_time_second;
        double since_plan_start_second;
    };

    // The time for init would be zero
    // Must be invoked before any call to now_in_second()
    void init_timer();

    // Get current time expressed in second
    double now_in_second();
}