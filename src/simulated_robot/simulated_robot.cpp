//
// Created by wei on 8/28/19.
//

#include "simulated_robot/simulated_robot.h"
#include "simulated_robot/iiwa_common.h"
#include "simulated_robot/inverse_dynamic_controller.h"

#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/common/drake_assert.h>
#include <drake/common/find_resource.h>
#include <drake/common/text_logging.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/rigid_body_plant/frame_visualizer.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/multibody/rigid_body_tree_construction.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>


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
    simulation_thread_ = std::thread(&SimulatedRobotArm::runSimulation, this);
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
    const char* kModelPath =
            "drake/manipulation/models/iiwa_description/"
            "urdf/iiwa14_polytope_collision.urdf";
    const std::string urdf = FindResourceOrThrow(kModelPath);
    {
        // Construct the tree
        auto tree = std::make_unique<RigidBodyTree<double>>();
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                urdf, multibody::joints::kFixed, tree.get());
        multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);
        plant = builder.template AddSystem<systems::RigidBodyPlant<double>>(
                std::move(tree));
    }
    const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();

    // The controller
    Eigen::VectorXd iiwa_kp, iiwa_kd, iiwa_ki;
    SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
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
    simulator.Initialize();

    // Start run simulation
    simulator.StepTo(simulation_time_second_);
}