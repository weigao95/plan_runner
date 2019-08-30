//
// Created by wei on 8/28/19.
//

#include "simulated_robot/simulated_robot.h"
#include "simulated_robot/inverse_dynamic_controller.h"

#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/common/drake_assert.h>
#include <drake/common/text_logging.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/rigid_body_tree_construction.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

#include <chrono>
#include <thread>


arm_runner::SimulatedRobotArm::SimulatedRobotArm(
    const std::string &model_urdf,
    double simulation_time_second
) : simulation_time_second_(simulation_time_second)
{
    tree_ = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            model_urdf, drake::multibody::joints::kFixed, tree_.get());
    drake::multibody::AddFlatTerrainToWorld(tree_.get(), 100., 10.);

    // Clear the exchange data
    exchanged_data_.latest_measurement.set_invalid();
    exchanged_data_.latest_command.set_invalid();
}


arm_runner::SimulatedRobotArm::SimulatedRobotArm(
    std::unique_ptr<RigidBodyTree<double>> robot,
    double simulation_time_second
) : simulation_time_second_(simulation_time_second), tree_(std::move(robot))
{
    // Clear the exchange data
    exchanged_data_.latest_measurement.set_invalid();
    exchanged_data_.latest_command.set_invalid();
}


void arm_runner::SimulatedRobotArm::getRawMeasurement(RobotArmMeasurement &measurement) {
    std::lock_guard<std::mutex> guard(exchanged_data_.mutex);
    DRAKE_ASSERT(exchanged_data_.latest_measurement.is_valid());
    measurement = exchanged_data_.latest_measurement;
}


void arm_runner::SimulatedRobotArm::sendRawCommand(const arm_runner::RobotArmCommand &command) {
    std::lock_guard<std::mutex> guard(exchanged_data_.mutex);
    DRAKE_ASSERT(command.is_valid());
    exchanged_data_.latest_command = command;
}


void arm_runner::SimulatedRobotArm::Start() {
    // Launch the simulation thread
    simulation_thread_ = std::thread(&SimulatedRobotArm::runSimulation, this);

    // Wait until on measurement is active
    while(true) {
        // Read the measurement
        exchanged_data_.mutex.lock();
        bool measurement_valid = exchanged_data_.latest_measurement.is_valid();
        exchanged_data_.mutex.unlock();

        // Break if ok
        if(measurement_valid)
            break;
        else {
            constexpr int WAIT_MEASUREMENT_SLEEP_MS = 10;
            std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_MEASUREMENT_SLEEP_MS));
        }
    }
}


void arm_runner::SimulatedRobotArm::Stop() {
    if(simulation_thread_.joinable())
        simulation_thread_.join();
}


void arm_runner::SimulatedRobotArm::runSimulation() {
    // Use drake namespace
    using namespace drake;
    drake::lcm::DrakeLcm lcm;
    systems::DiagramBuilder<double> builder;
    systems::RigidBodyPlant<double>* plant = nullptr;

    // The plant
    plant = builder.template AddSystem<systems::RigidBodyPlant<double>>(std::move(tree_));
    const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();

    // The controller
    auto n_position = tree.get_num_positions();
    Eigen::VectorXd iiwa_kp, iiwa_kd; iiwa_kp.resize(n_position); iiwa_kd.resize(n_position);
    QpInverseDynamicsController::SetPositionControlledDefaultGains(&iiwa_kp, &iiwa_kd);
    auto controller = builder.template AddSystem<QpInverseDynamicsController>(
        tree.Clone(), exchanged_data_,
        iiwa_kp, iiwa_kd
    );

    // Creates and adds LCM publisher for visualization.
    // Construct the drake visualizer
    auto vis = builder.template AddSystem<systems::DrakeVisualizer>(tree, &lcm);
    vis->set_publish_period(0.005);

    // A set of connection
    builder.Connect(plant->state_output_port(),
                    controller->get_input_port_estimated_state());
    builder.Connect(controller->get_output_port_torque_commanded(),
                    plant->actuator_command_input_port());
    builder.Connect(plant->state_output_port(), vis->get_input_port(0));

    // The system
    auto sys = builder.Build();
    systems::Simulator<double> simulator(*sys);
    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(1.0);

    // Set the initial configuration
    auto& context = simulator.get_mutable_context();
    Eigen::VectorXd zero_init; zero_init.resize(n_position * 2); zero_init.setZero();
    context.SetContinuousState(zero_init);

    // Start run simulation
    simulator.AdvanceTo(simulation_time_second_);
}
