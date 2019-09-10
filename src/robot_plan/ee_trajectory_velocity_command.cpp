//
// Created by wei on 8/30/19.
//

#include "common/rbt_utils.h"
#include "robot_plan/ee_trajectory_velocity_command.h"
#include "robot_plan/forceguard_checker.h"


plan_runner::EETrajectoryVelocityCommandPlan::EETrajectoryVelocityCommandPlan(
    bool has_quaternion,
    std::string task_frame,
    std::vector<double> input_time,
    std::vector<Eigen::MatrixXd> ee_xyz_knots,
    std::vector<Eigen::Quaterniond> ee_quat_knots,
    std::string wrt_frame)
    : has_quaternion_(has_quaternion),
      task_frame_name_(std::move(task_frame)),
      input_time_(std::move(input_time)),
      ee_xyz_knots_(std::move(ee_xyz_knots)),
      ee_quat_knots_(std::move(ee_quat_knots)),
      task_frame_index_(0),
      wrt_frame_name_(std::move(wrt_frame))
{
    // Check
    DRAKE_ASSERT(input_time_.size() == ee_xyz_knots_.size());
    DRAKE_ASSERT(input_time_.size() == ee_quat_knots_.size());

    // The hyper-parameters
    kp_rotation_.setConstant(5);
    kp_translation_.setConstant(10);
}


std::shared_ptr<plan_runner::EETrajectoryVelocityCommandPlan> plan_runner::EETrajectoryVelocityCommandPlan::ConstructFromMessage(
    const RigidBodyTree<double>& tree,
    const robot_msgs::CartesianTrajectoryGoal::ConstPtr &goal
) {
    // Get data and basic check
    const auto& traj = goal->trajectory;
    int num_knot_points = traj.xyz_points.size();
    if(traj.xyz_points.size() < 2)
        return nullptr;
    std::string task_frame_name = traj.ee_frame_id;
    std::string wrt_frame_name = traj.xyz_points[0].header.frame_id;
    bool has_quat = (traj.quaternions.size() == traj.xyz_points.size());

    // Check the name
    if((!bodyOrFrameContainedInTree(tree, task_frame_name)) || (!bodyOrFrameContainedInTree(tree, wrt_frame_name)))
        return nullptr;

    // Allocate the space
    std::vector<Eigen::MatrixXd> position_knot_vec(num_knot_points, Eigen::MatrixXd::Zero(3, 1));
    std::vector<Eigen::Quaterniond> orientation_knot_vec(num_knot_points, Eigen::Quaterniond::Identity());
    std::vector<double> input_time(num_knot_points, 0);

    // Construct the knots
    for (int i = 0; i < num_knot_points; i++) {
        // The cache
        Eigen::Vector3d knot_position;
        Eigen::Quaterniond knot_orientation;
        if (i == 0) {
            // replace first knot point by current position of ee_frame
            knot_position.setZero();
            knot_orientation.setIdentity();
        } else {
            // The point is always here
            const geometry_msgs::PointStamped &xyz_point = traj.xyz_points[i];
            knot_position = Eigen::Vector3d(xyz_point.point.x, xyz_point.point.y, xyz_point.point.z);

            // May not have quaternion
            if(has_quat) {
                const auto& quat = traj.quaternions[i];
                knot_orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
            } else {
                knot_orientation.setIdentity();
            }

            // Check the frame
            if(xyz_point.header.frame_id != wrt_frame_name) {
                return nullptr;
            }
        }

        // Push
        position_knot_vec[i] = knot_position;
        orientation_knot_vec[i] = knot_orientation;
        input_time[i] = (traj.time_from_start[i].toSec());
    }

    // Construct
    auto plan = std::make_shared<EETrajectoryVelocityCommandPlan>(
            has_quat,
            std::move(task_frame_name),
            std::move(input_time),
            std::move(position_knot_vec),
            std::move(orientation_knot_vec),
            std::move(wrt_frame_name)
    );

    // The force guard
    for(const auto& guard_msg : goal->force_guard) {
        auto checker_vec = ExternalForceGuardChecker::ConstructCheckersFromForceGuardMessage(tree, guard_msg);
        for(const auto& checker : checker_vec) {
            plan->AddSafetyChecker(checker);
        }
    }

    // OK
    return plan;
}


void plan_runner::EETrajectoryVelocityCommandPlan::InitializePlan(const plan_runner::CommandInput &input) {
    // Get data
    const auto& tree = *input.robot_rbt;
    const auto& cache = *input.measured_state_cache;
    task_frame_index_ = getBodyOrFrameIndex(tree, task_frame_name_);
    DRAKE_ASSERT(task_frame_index_ != 0);

    // Compute the initial configuration
    const int world_frame = RigidBodyTreeConstants::kWorldBodyIndex;
    Eigen::Isometry3d ee_transform = tree.relativeTransform(cache, world_frame, task_frame_index_);
    ee_xyz_knots_[0] = ee_transform.translation();
    ee_quat_knots_[0] = Eigen::Quaterniond(ee_transform.linear());

    // Transform all other knots as they are expressed in wrt_frame
    int wrt_frame_index = getBodyOrFrameIndex(tree, wrt_frame_name_);
    Eigen::Isometry3d knot_transform = tree.relativeTransform(cache, world_frame, wrt_frame_index);

    // Apply the transform
    for(auto i = 1; i < ee_xyz_knots_.size(); i++) {
        ee_xyz_knots_[i] = knot_transform.linear() * ee_xyz_knots_[i] + knot_transform.translation();
        if(has_quaternion_)
            ee_quat_knots_[i] = knot_transform.rotation() * ee_quat_knots_[i];
        else
            ee_quat_knots_[i] = ee_quat_knots_[0];
    }

    // Compute the trajectory
    Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(3, 1);
    ee_xyz_trajectory_ = PiecewisePolynomial::Cubic(input_time_, ee_xyz_knots_, knot_dot, knot_dot);
    ee_xyz_velocity_trajectory_ = ee_xyz_trajectory_.derivative(1);
    ee_orientation_trajectory_ = PiecewiseQuaternionSlerp(input_time_, ee_quat_knots_);

    // Set the flag
    RobotPlanBase::InitializePlan(input);
}


Eigen::Vector3d plan_runner::EETrajectoryVelocityCommandPlan::logSO3(const Eigen::Matrix3d& rotation) {
    Eigen::AngleAxisd angle_axis(rotation);
    return angle_axis.angle() * angle_axis.axis();
}


// The workforce function
void plan_runner::EETrajectoryVelocityCommandPlan::ComputeCommand(
    const plan_runner::CommandInput &input,
    plan_runner::RobotArmCommand &command
) {
    // Collect data
    DRAKE_ASSERT(input.is_valid());
    const int world_frame = RigidBodyTreeConstants::kWorldBodyIndex;
    const RigidBodyTree<double>& tree = *input.robot_rbt;
    const auto& cache = *input.measured_state_cache;
    auto t = GetTimeSincePlanStartSecond(input.latest_measurement->time_stamp);

    // Compute the desired frame transformation
    Eigen::Vector3d ee_position_ref = ee_xyz_trajectory_.value(t);
    Eigen::Quaterniond ee_orientation_ref = ee_orientation_trajectory_.orientation(t);
    Eigen::Isometry3d ee_transform_ref;
    ee_transform_ref.linear() = ee_orientation_ref.toRotationMatrix();
    ee_transform_ref.translation() = ee_position_ref;

    // Compute the actual transform ee_to_world
    Eigen::Isometry3d ee_transform = tree.relativeTransform(cache, world_frame, task_frame_index_);
    Eigen::Isometry3d world_to_ee = ee_transform.inverse();

    // The rotation error term
    Eigen::Isometry3d transform_error = world_to_ee * ee_transform_ref;
    Eigen::Vector3d log_rotation_error = logSO3(transform_error.linear());

    // The twist feedback
    using TwistVector = drake::TwistVector<double>;
    TwistVector twist_pd;
    twist_pd.head(3) = kp_rotation_.array() * log_rotation_error.array();
    twist_pd.tail(3) = kp_translation_.array() * transform_error.translation().array();

    // The velocity information
    // The linear and angular velocity are all expressed in world
    Eigen::Vector3d ee_linear_velocity_ref = ee_xyz_velocity_trajectory_.value(t);
    Eigen::Vector3d ee_angular_velocity_ref = ee_orientation_trajectory_.angular_velocity(t);
    Eigen::Vector3d ee_linear_v_in_ee = world_to_ee.rotation() * ee_linear_velocity_ref;
    Eigen::Vector3d ee_angular_v_in_ee = world_to_ee.rotation() * ee_angular_velocity_ref;
    TwistVector twist_fwd;
    twist_fwd.head(3) = ee_angular_v_in_ee;
    twist_fwd.tail(3) = ee_linear_v_in_ee;

    // The twist jacobian
    ee_twist_jacobian_expressed_in_ee = tree.geometricJacobian(
        cache, world_frame, task_frame_index_, task_frame_index_);

    // Compute sudo-inverse
    TwistVector desired_twist = twist_pd + twist_fwd;
    auto svd = ee_twist_jacobian_expressed_in_ee.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(0.01);
    Eigen::VectorXd qdot_command = svd.solve(desired_twist);

    // The forward integration position
    const double* q_fwd_integration = integrate_option_.GetForwardIntegrationJointPosition(input);

    // Write to the result
    command.set_invalid();
    for(auto i = 0; i < qdot_command.size(); i++) {
        command.joint_velocities[i] = qdot_command[i];
        command.joint_position[i] = q_fwd_integration[i] + qdot_command[i] * input.control_interval_second;
    }
    command.velocity_validity = true;
    command.position_validity = true;
}


// The processing of parameter
plan_runner::LoadParameterStatus plan_runner::EETrajectoryVelocityCommandPlan::LoadParameterFrom(const YAML::Node &datamap) {
    // The base class
    auto base_load_result = integrate_option_.LoadParameterFrom(datamap);

    // Check the key
    auto key = DefaultClassParameterNameKey();
    if(!datamap[key]) {
        // Keep current value
        return StatusAnd(base_load_result, LoadParameterStatus::NonFatalError);
    }

    // Load it
    if(datamap[key]["rotation"]) {
        kp_rotation_[0] = datamap[key]["rotation"][0].as<double>();
        kp_rotation_[1] = datamap[key]["rotation"][1].as<double>();
        kp_rotation_[2] = datamap[key]["rotation"][2].as<double>();
    }

    if(datamap[key]["translation"]) {
        kp_translation_[0] = datamap[key]["translation"][0].as<double>();
        kp_translation_[1] = datamap[key]["translation"][1].as<double>();
        kp_translation_[2] = datamap[key]["translation"][2].as<double>();
    }
    return StatusAnd(base_load_result, LoadParameterStatus::Success);
}

void plan_runner::EETrajectoryVelocityCommandPlan::SaveParameterTo(YAML::Node &datamap) const {
    // The base class
    integrate_option_.SaveParameterTo(datamap);

    // The rotation gain
    auto key = DefaultClassParameterNameKey();
    datamap[key]["rotation"].push_back(kp_rotation_[0]);
    datamap[key]["rotation"].push_back(kp_rotation_[1]);
    datamap[key]["rotation"].push_back(kp_rotation_[2]);

    // The translation gain
    datamap[key]["translation"].push_back(kp_translation_[0]);
    datamap[key]["translation"].push_back(kp_translation_[1]);
    datamap[key]["translation"].push_back(kp_translation_[2]);
}