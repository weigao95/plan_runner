//
// Created by wei on 9/4/19.
//

#pragma once

#include <yaml-cpp/yaml.h>

namespace arm_runner {


    // The Serializable to hold the parameter of an object
    class YamlSerializable {
    public:
        virtual ~YamlSerializable() = default;

        // The load from method would update itself from the datamap
        // Some parameter are optional, while others are required
        // This method can throw runtime_error if required parameter is not present
        virtual void LoadFrom(const YAML::Node& datamap) {};

        // Save the parameter to the give map
        // Can be LoadFrom after SaveTo
        virtual void SaveTo(YAML::Node& datamap) {};
    };
}
