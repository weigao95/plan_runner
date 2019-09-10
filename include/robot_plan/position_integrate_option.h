//
// Created by wei on 9/10/19.
//

#pragma once

#include "common/yaml_serializable.h"
#include "common/plan_common_types.h"


namespace plan_runner {

    class PositionIntegratorOption : public YamlSerializableParameter {
    public:
        explicit PositionIntegratorOption(bool use_commaned_fwd_integration = true)
                : use_commanded_fwd_integration_(use_commaned_fwd_integration) {};
        ~PositionIntegratorOption() override = default;

        // Use which position as the one in forward integration
    public:
        const double* GetForwardIntegrationJointPosition(const plan_runner::CommandInput &input) const;
        bool UseCommandForwardIntegration() const { return use_commanded_fwd_integration_; }
        LoadParameterStatus LoadParameterFrom(const YAML::Node& datamap) override;
        void SaveParameterTo(YAML::Node& datamap) const override;
    private:
        bool use_commanded_fwd_integration_;
    };
}
