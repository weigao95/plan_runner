//
// Created by wei on 9/2/19.
//

#pragma once

#include <drake/multibody/rigid_body_tree.h>


namespace plan_runner {

    // Check the status of containment
    bool bodyOrFrameContainedInTree(
            const RigidBodyTree<double>& tree,
            const std::string& body_or_frame_name);
    int getBodyOrFrameIndex(
            const RigidBodyTree<double>& tree,
            const std::string& body_or_frame_name);

    // Used for rotation controllers
    Eigen::Vector3d logSO3(const Eigen::Matrix3d& rotation);
}
