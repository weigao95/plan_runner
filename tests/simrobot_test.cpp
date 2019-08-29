//
// Created by wei on 8/29/19.
//

#include <gtest/gtest.h>

#include "simulated_robot/simulated_robot.h"

TEST(SimRobotTest, ConstructTest) {
    using namespace arm_runner;
    SimulatedRobotArm robot_arm(1);
    robot_arm.Start();
    robot_arm.Stop();
}