//
// Created by wei on 8/28/19.
//

#include "inverse_dynamic_controller.h"


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
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    const Eigen::VectorXd x = (*this->EvalVectorInput(context,
                                    get_input_port_estimated_state().get_index())).CopyToVector();
    const Vector7d q = x.head(nq_);
    const Vector7d v = x.tail(nq_);
    auto current_time_second = context.get_time();

    // Create tree alias, so that clion doesn't complain about unique pointers.
    const RigidBodyTreed &tree = *tree_;
    KinematicsCache<double> cache = tree.CreateKinematicsCache();
    cache.initialize(q, v);
    tree.doKinematics(cache, true);

    // Desired position tracking
    Vector7d err_q; err_q.setZero();
    if(exchange_data_.latest_command.position_validity) {
        Eigen::Map<Vector7d> q_ref = Eigen::Map<Vector7d>(exchange_data_.latest_command.joint_position);
        err_q = q_ref - q;
    }

    // Desired velocity tracking
    Vector7d err_v; err_v.setZero();
    if(exchange_data_.latest_command.velocity_validity) {
        Eigen::Map<Vector7d> v_ref = Eigen::Map<Vector7d>(exchange_data_.latest_command.joint_velocities);
        err_v = v_ref - v;
    }

    // Compute the feedforward terms
    RigidBodyTree<double>::BodyToWrenchMap external_wrenches;
    Eigen::VectorXd vd_commanded = kp_.array() * err_q.array() + kd_.array() * err_v.array();
    Vector7d tau = tree.inverseDynamics(cache, external_wrenches, vd_commanded);

    // Add reference command
    if(exchange_data_.latest_command.torque_validity) {
        Eigen::Map<Vector7d> tau_ref = Eigen::Map<Vector7d>(exchange_data_.latest_command.joint_torque);
        tau += tau_ref;
    }

    // Write to exchange
    auto& measurement = exchange_data_.latest_measurement;
    measurement.set_invalid();
    measurement.time_stamp.absolute_time_second = current_time_second;
    for(auto i = 0; i < nq_; i++) {
        measurement.joint_position[i] = q[i];
        measurement.joint_velocities[i] = v[i];
    }
    measurement.position_validity = measurement.velocity_validity = true;

    // Handin to drake
    discrete_state->get_mutable_vector().SetFromVector(tau);
}