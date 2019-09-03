//
// Created by wei on 9/2/19.
//

#pragma once

#include <mutex>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include <drake/lcmt_iiwa_command.hpp>
#include <drake/lcmt_iiwa_status.hpp>

#include "common/robot_communication.h"


namespace arm_runner {


    class KukaLCMInterface : public RobotCommunication {
    public:
        using lcmt_iiwa_status = drake::lcmt_iiwa_status;
        using lcmt_iiwa_command = drake::lcmt_iiwa_command;
        KukaLCMInterface(std::string lcm_status_channel, std::string lcm_command_channel);
        ~KukaLCMInterface() override = default;

        // The init function
        void Start() override;
        void Stop() override;

        // The communication interface
    protected:
        void getRawMeasurement(RobotArmMeasurement& measurement) override;
        void sendRawCommand(const RobotArmCommand& command) override;

    private:
        // The communication with lcm
        std::string lcm_status_channel_;
        std::string lcm_command_channel_;
        lcm::LCM command_publisher_lcm_;


        // The measurement exchange data
    private:
        struct {
            std::mutex mutex;
            RobotArmMeasurement measurement;
        } exchange_data_;
        bool quit_receiving_;
        std::thread receive_rbt_status_thread_;
    public:
        void handleReceiveIIWAStatus(
            const lcm::ReceiveBuffer *,
            const std::string &,
            const lcmt_iiwa_status *status);
        void receiveRobotStatusThread();


        // Caches
    private:
        RobotArmMeasurement measurement_cache;
    };
}