//
// Created by wei on 9/2/19.
//

#pragma once

#include <drake/multibody/rigid_body_tree.h>


namespace arm_runner {

    // Check the status of containment
    bool bodyOrFrameContainedInTree(
            const RigidBodyTree<double>& tree,
            const std::string& body_or_frame_name);
    int getBodyOrFrameIndex(
            const RigidBodyTree<double>& tree,
            const std::string& body_or_frame_name);
}
