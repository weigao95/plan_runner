//
// Created by wei on 9/4/19.
//

#pragma once

#include <yaml-cpp/yaml.h>

namespace arm_runner {

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
        virtual LoadParameterStatus LoadParameterFrom(const YAML::Node& datamap) {};

        // Save the parameter to the give map
        // Can be LoadFrom after SaveTo
        virtual void SaveParameterTo(YAML::Node& datamap) const {};

        // The default name
        // Should be might be override for better readbility
        virtual std::string DefaultClassParameterNameKey() const { return typeid(*this).name(); }
    };
}
