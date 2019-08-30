//
// Created by wei on 8/28/19.
//

#pragma once

#include <memory>
#include <string>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/systems/framework/leaf_system.h>

#include "simulated_robot/exchange_type.h"

namespace arm_runner {


class QpInverseDynamicsController : public drake::systems::LeafSystem<double> {
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QpInverseDynamicsController)
        QpInverseDynamicsController(
            std::unique_ptr<RigidBodyTree<double>> tree,
            SimulationExchangeData& exchnage_data,
            const Eigen::VectorXd& kp,
            const Eigen::VectorXd& kd,
            double control_period = 0.005);

        const drake::systems::OutputPort<double>& get_output_port_torque_commanded() const {
            return this->get_output_port(idx_output_port_commanded_torque_);
        }

        const drake::systems::InputPort<double>& get_input_port_estimated_state() const {
            return this->get_input_port(idx_input_port_estimated_state_);
        }

        void CopyStateOut(const drake::systems::Context<double>& context,
                          drake::systems::BasicVector<double>* output) const {
            output->SetFromVector(context.get_discrete_state(0).CopyToVector());
        }

        void DoCalcDiscreteVariableUpdates(
            const drake::systems::Context<double>& context,
            const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
            drake::systems::DiscreteValues<double>* discrete_state) const override;

        // The default gain for feedback controller
        static void SetPositionControlledDefaultGains(
            Eigen::VectorXd* Kp,
            Eigen::VectorXd* Kd);

    private:
        const double control_period_;  // in seconds
        int nq_;
        Eigen::VectorXd kp_;
        Eigen::VectorXd kd_;
        std::unique_ptr<RigidBodyTreed> tree_{nullptr};
        SimulationExchangeData& exchange_data_;
        int idx_input_port_estimated_state_{-1};
        int idx_output_port_commanded_torque_{-1};
    };

}
