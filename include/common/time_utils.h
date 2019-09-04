//
// Created by wei on 8/28/19.
//

#pragma once

#include <cstdint>
#include <ros/ros.h>

namespace arm_runner {

    class TimeStamp {
    public:
        // Initialize to zero
        explicit TimeStamp();

        // The conversion between second
        void FromSecond(double second) { ros_time.fromSec(second); }
        double ToSecond() const { return ros_time.toSec(); }
        double SecondSince(const TimeStamp& prev_time) const;

        // The conversion between micro-second
        void FromMicrosecond(int64_t usecond);
        int64_t ToMicrosecond() const;

        // Using the timer from ros
        const ros::Time& GetROSTimeStamp() const { return ros_time; }
    private:
        ros::Time ros_time;
    };
}
