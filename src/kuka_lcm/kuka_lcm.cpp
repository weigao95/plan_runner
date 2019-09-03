//
// Created by wei on 9/2/19.
//

#include "kuka_lcm.h"

// This app only works for kuka iiwa arm
constexpr int KUKA_IIWA_ARM_NUM_JOINT = 7;


arm_runner::KukaLCMInterface::KukaLCMInterface(
    std::string lcm_status_channel,
    std::string lcm_command_channel
) : lcm_status_channel_(std::move(lcm_status_channel)),
    lcm_command_channel_(std::move(lcm_command_channel))
{
    // Set everything to invalid
    exchange_data_.measurement.set_invalid();
    quit_receiving_ = false;

    // Can be ignored
    measurement_cache.set_invalid();
}


void arm_runner::KukaLCMInterface::Start() {
    // Invoke the subscriber thread
    exchange_data_.measurement.set_invalid();
    quit_receiving_ = false;
    receive_rbt_status_thread_ = std::thread(&KukaLCMInterface::receiveRobotStatusThread, this);

    // Wait for one msg
    constexpr int WAIT_MSG_INTERVAL_MS = 40;
    do {
        // Wait the msg
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_MSG_INTERVAL_MS));

        // Get validity
        bool msg_valid = false;
        exchange_data_.mutex.lock();
        msg_valid = exchange_data_.measurement.is_valid();
        exchange_data_.mutex.unlock();

        // Check it
        if(msg_valid)
            break;
    } while(true);
}


void arm_runner::KukaLCMInterface::Stop() {
    quit_receiving_ = true;
    if(receive_rbt_status_thread_.joinable())
        receive_rbt_status_thread_.join();
}


// The interface for upper-level
void arm_runner::KukaLCMInterface::getRawMeasurement(
        arm_runner::RobotArmMeasurement &measurement
) {
    // Just copy the measurement
    std::lock_guard<std::mutex> guard(exchange_data_.mutex);
    measurement = exchange_data_.measurement;
}


void arm_runner::KukaLCMInterface::sendRawCommand(
    const arm_runner::RobotArmCommand &command
) {
    // The time from command
    command_cache.utime = int64_t(command.time_stamp.absolute_time_second * 1000.0);

    // Joint position should always work
    command_cache.num_joints = KUKA_IIWA_ARM_NUM_JOINT;
    command_cache.joint_position.resize(KUKA_IIWA_ARM_NUM_JOINT);
    for(auto i = 0; i < KUKA_IIWA_ARM_NUM_JOINT; i++) {
        command_cache.joint_position[i] = command.joint_position[i];
    }

    // If joint torque not available, set to zero
    command_cache.num_torques = KUKA_IIWA_ARM_NUM_JOINT;
    command_cache.joint_torque.resize(KUKA_IIWA_ARM_NUM_JOINT);
    for(auto i = 0; i < KUKA_IIWA_ARM_NUM_JOINT; i++) {
        if(command.torque_validity)
            command_cache.joint_torque[i] = command.joint_torque[i];
        else
            command_cache.joint_torque[i] = 0.0;
    }

    // Send to robot
    command_publisher_lcm_.publish(lcm_command_channel_, &command_cache);
}


// The handler for measurement
void arm_runner::KukaLCMInterface::handleReceiveIIWAStatus(
    const lcm::ReceiveBuffer *,
    const std::string &,
    const arm_runner::KukaLCMInterface::lcmt_iiwa_status *status_in
) {
    // Copy to cache
    measurement_cache.set_invalid();
    const auto& status = *status_in;
    const auto n_joint = KUKA_IIWA_ARM_NUM_JOINT;
    for(auto i = 0; i < n_joint; i++) {
        measurement_cache.joint_position[i] = status.joint_position_measured[i];
        measurement_cache.joint_velocities[i] = status.joint_velocity_estimated[i];
        measurement_cache.joint_torque[i] = status.joint_torque_measured[i];
    }
    measurement_cache.position_validity = true;
    measurement_cache.velocity_validity = true;
    measurement_cache.torque_validity = true;

    // The time of measurement
    measurement_cache.time_stamp.absolute_time_second = double(status.utime) / 1000.0;

    // Copy to exchange data
    exchange_data_.mutex.lock();
    exchange_data_.measurement = measurement_cache;
    exchange_data_.mutex.unlock();
}


void arm_runner::KukaLCMInterface::receiveRobotStatusThread() {
    // Construct the receiver
    lcm::LCM receiver_lcm;
    receiver_lcm.subscribe(
        lcm_status_channel_,
        &KukaLCMInterface::handleReceiveIIWAStatus, this);

    // Go
    while (true) {
        // Call lcm handle until at least one status message is
        // processed.
        constexpr int MESSAGE_TIMEOUT_MS = 10;
        while (0 == receiver_lcm.handleTimeout(MESSAGE_TIMEOUT_MS)) {
            // TODO: Print something here so users know no LCM message has been
            // received.
        }

        // Break if requested
        if(quit_receiving_)
            break;
    }
}