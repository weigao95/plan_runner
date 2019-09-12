//
// Created by wei on 9/4/19.
//

#pragma once

#include <yaml-cpp/yaml.h>

namespace plan_runner {

    // Depends on the result
    // The upper-level processors have different actions
    enum class LoadParameterStatus {
        Success,  // Everything goes well
        NonFatalError,  // Something wrong, but the object is still OK (default value provided)
        FatalError,  // The object is not OK without given parameter
    };


    // The Serializable to hold the parameter of an object
    class YamlSerializableParameter {
    public:
        virtual ~YamlSerializableParameter() = default;

        // The load from method would update itself from the datamap
        // Some parameter are optional, while others are required
        // This method can throw runtime_error if required parameter is not present
        virtual LoadParameterStatus LoadParameterFrom(const YAML::Node& datamap) {
            return LoadParameterStatus::Success;
        };

        // Save the parameter to the give map
        // Can be LoadFrom after SaveTo
        virtual void SaveParameterTo(YAML::Node& datamap) const {};

        // The default name
        // Should be might be override for better readbility
        virtual std::string DefaultClassParameterNameKey() const { return typeid(*this).name(); }

        // The AND operator on LoadParameterStatus
    protected:
        // Is status A strictly better than status B
        inline static bool IsStatusBetterThan(LoadParameterStatus a, LoadParameterStatus b) {
            if(a == LoadParameterStatus::NonFatalError && b == LoadParameterStatus::FatalError)
                return true;
            if(a == LoadParameterStatus::Success && (b != LoadParameterStatus::Success))
                return true;
            return false;
        }
        inline static LoadParameterStatus TheWorseStatus(LoadParameterStatus a, LoadParameterStatus b) {
            if(IsStatusBetterThan(a ,b))
                return b;
            else
                return a;
        }
    };
}
