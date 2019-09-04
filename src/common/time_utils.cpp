//
// Created by wei on 9/4/19.
//

#include "common/time_utils.h"


// The interface between second
arm_runner::TimeStamp::TimeStamp() {
    FromSecond(0);
}


double arm_runner::TimeStamp::SecondSince(const arm_runner::TimeStamp &prev_time) const {
    const auto duration = GetROSTimeStamp() - prev_time.GetROSTimeStamp();
    return duration.toSec();
}


// The interface between micro-second
void arm_runner::TimeStamp::FromMicrosecond(int64_t usecond) {
    uint64_t nanosecond = usecond * 1000;
    ros_time.fromNSec(nanosecond);
}

int64_t arm_runner::TimeStamp::ToMicrosecond() const {
    uint64_t nanosecond = ros_time.toNSec();
    return nanosecond / 1000;
}