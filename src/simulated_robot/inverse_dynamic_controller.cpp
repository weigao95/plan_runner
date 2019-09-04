//
// Created by wei on 8/28/19.
//

#include "simulated_robot/inverse_dynamic_controller.h"


arm_runner::QpInverseDynamicsController::QpInverseDynamicsController(
    std::unique_ptr<RigidBodyTree<double>> tree,
    SimulationExchangeData& exchange_data,
    const drake::VectorX<double> &kp,
    const drake::VectorX<double> &kd,
    double control_period)
: exchange_data_(exchange_data), control_period_(control_period)
{
    // sanity check
    nq_ = tree->get_num_positions();
    DRAKE_DEMAND(nq_ == kp.size());
    DRAKE_DEMAND(nq_ == kd.size());

    // copy stuff.
    tree_ = std::move(tree);
    kp_ = kp;
    kd_ = kd;

    // declare ports and states.
    this->DeclarePeriodicDiscreteUpdate(control_period_);
    // estimated state input port
    idx_input_port_estimated_state_ =
            this->DeclareVectorInputPort(drake::systems::BasicVector<double>(2 * nq_))
                    .get_index();
    // commanded torque output port
    idx_output_port_commanded_torque_ =
            this->DeclareVectorOutputPort(
                            drake::systems::BasicVector<double>(nq_),
                            &QpInverseDynamicsController::CopyStateOut)
                    .get_index();
    // the system's state is the commanded torque sent to rigid body plant.
    this->DeclareDiscreteState(nq_);
}


void arm_runner::QpInverseDynamicsController::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<double> &context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<double> *> &,
    drake::systems::DiscreteValues<double> *discrete_state
) const {
    // Get the lock
    std::lock_guard<std::mutex> guard(exchange_data_.mutex);

    // Discrete state is the output (robot torque)
    const Eigen::VectorXd x = (*this->EvalVectorInput(context,
                                    get_input_port_estimated_state().get_index())).CopyToVector();
    const Eigen::VectorXd q = x.head(nq_);
    const Eigen::VectorXd v = x.tail(nq_);
    auto current_time_second = context.get_time();

    // Create tree alias, so that clion doesn't complain about unique pointers.
    const RigidBodyTreed &tree = *tree_;
    KinematicsCache<double> cache = tree.CreateKinematicsCache();
    cache.initialize(q, v);
    tree.doKinematics(cache, true);

    // Desired position tracking
    Eigen::VectorXd err_q; err_q.resize(nq_); err_q.setZero();
    if(exchange_data_.latest_command.position_validity) {
        Eigen::Map<Eigen::VectorXd> q_ref =
                Eigen::Map<Eigen::VectorXd>(exchange_data_.latest_command.joint_position, nq_);
        err_q = q_ref - q;
    }

    // Desired velocity tracking
    Eigen::VectorXd err_v; err_v.resize(nq_); err_v.setZero();
    if(exchange_data_.latest_command.velocity_validity) {
        Eigen::Map<Eigen::VectorXd> v_ref =
                Eigen::Map<Eigen::VectorXd>(exchange_data_.latest_command.joint_velocities, nq_);
        err_v = v_ref - v;
    }

    // Compute the feedforward terms
    RigidBodyTree<double>::BodyToWrenchMap external_wrenches;
    Eigen::VectorXd vd_commanded = kp_.array() * err_q.array() + kd_.array() * err_v.array();
    Eigen::VectorXd tau = tree.inverseDynamics(cache, external_wrenches, vd_commanded);

    // Add reference command
    if(exchange_data_.latest_command.torque_validity) {
        Eigen::Map<Eigen::VectorXd> tau_ref =
                Eigen::Map<Eigen::VectorXd>(exchange_data_.latest_command.joint_torque, nq_);
        tau += tau_ref;
    }

    // Write to exchange
    auto& measurement = exchange_data_.latest_measurement;
    measurement.set_invalid();
    measurement.time_stamp.FromSecond(current_time_second);
    for(auto i = 0; i < nq_; i++) {
        measurement.joint_position[i] = q[i];
        measurement.joint_velocities[i] = v[i];
    }
    measurement.position_validity = measurement.velocity_validity = true;

    // Handin to drake
    discrete_state->get_mutable_vector().SetFromVector(tau);
}


void arm_runner::QpInverseDynamicsController::SetPositionControlledDefaultGains(
    Eigen::VectorXd *Kp,
    Eigen::VectorXd *Kd
) {
    // All the gains are for acceleration, not directly responsible for generating
    // torques. These are set to high values to ensure good tracking. These gains
    // are picked arbitrarily.
    for(auto i = 0; i < Kp->size(); i++) {
        (*Kp)[i] = 100;
    }
    Kd->resize(Kp->size());
    for (auto i = 0; i < Kp->size(); i++) {
        // Critical damping gains.
        (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
    }
}