//
// Created by wei on 8/28/19.
//

#include "arm_runner/time_utils.h"
#include <ros/timer.h>

// The starting time
ros::Time start_time;


void arm_runner::init_timer() {
    start_time = ros::Time::now();
}


double arm_runner::now_in_second() {
    auto now = ros::Time::now();
    auto duration = now - start_time;
    return duration.toSec();
}