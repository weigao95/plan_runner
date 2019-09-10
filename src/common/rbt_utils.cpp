//
// Created by wei on 9/2/19.
//

#include "common/rbt_utils.h"

bool plan_runner::bodyOrFrameContainedInTree(
    const RigidBodyTree<double> &tree,
    const std::string &body_or_frame_name
) {
    // Initial flag
    bool body_in_tree = false;
    bool frame_in_tree = false;

    // Check body
    try {
        auto body_frame_index = tree.FindBodyIndex(body_or_frame_name);
        body_in_tree = true;
    } catch (const std::logic_error& e) {
        // Let it go
    }

    // Check the frame
    try {
        auto frame = tree.findFrame(body_or_frame_name);
        frame_in_tree = true;
    } catch (const std::logic_error& e) {
        // Let it go
    }

    // OK
    return body_in_tree || frame_in_tree;
}


int plan_runner::getBodyOrFrameIndex(
    const RigidBodyTree<double> &tree,
    const std::string &body_or_frame_name
) {
    int body_frame_index = 0;

    // First try the body
    try {
        body_frame_index = tree.FindBodyIndex(body_or_frame_name);
    } catch (const std::logic_error& e) {
        // Then try the frame
        auto frame = tree.findFrame(body_or_frame_name);
        body_frame_index = frame->get_frame_index();
    }

    // OK
    return body_frame_index;
}