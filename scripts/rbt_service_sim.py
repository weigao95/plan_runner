# system
import numpy as np
import attr

# ROS
import rospy
import actionlib
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ROS custom packages
import robot_msgs.msg


def make_cartesian_goal_trajectory_msg(
        xyz_goal,  # type: List[float]
        ee_frame_id,  # type: str
        expressed_in_frame,  # type: str
        speed=0.01,
        duration=None  # type: float
):
    """
    Move the ee_frame to xyz_goal expressed in 'expressed_in_frame'
    :param xyz_goal:
    :param ee_frame_id:
    :param expressed_in_frame:
    :param speed: meters/second
    :param duration:
    :return:
    """
    goal_msg = robot_msgs.msg.CartesianTrajectoryGoal()
    traj = goal_msg.trajectory

    # the first knot point always gets replaced by the current
    xyz_knot = PointStamped()
    xyz_knot.header.frame_id = expressed_in_frame
    xyz_knot.point.x = 0
    xyz_knot.point.y = 0
    xyz_knot.point.z = 0
    traj.xyz_points.append(xyz_knot)

    xyz_knot = PointStamped()
    xyz_knot.header.frame_id = expressed_in_frame
    xyz_knot.point.x = xyz_goal[0]
    xyz_knot.point.y = xyz_goal[1]
    xyz_knot.point.z = xyz_goal[2]
    traj.xyz_points.append(xyz_knot)
    traj.ee_frame_id = ee_frame_id

    # The duration
    if duration is None:
        duration = max(np.linalg.norm(xyz_goal) / (1.0 * speed), 1)
    traj.time_from_start.append(rospy.Duration(0.0))
    traj.time_from_start.append(rospy.Duration(duration))
    return goal_msg


@attr.s
class ServiceNameConfig(object):
    joint_trajectory_action = "/plan_runner/JointTrajectory"
    ee_trajectory_action = "/plan_runner/CartesianTrajectory"


class RobotMovementService(object):

    def __init__(self, service_config=ServiceNameConfig()):
        self._setupActionClients(service_config)
        self._joint_names = [
            "iiwa_joint_1",
            "iiwa_joint_2",
            "iiwa_joint_3",
            "iiwa_joint_4",
            "iiwa_joint_5",
            "iiwa_joint_6",
            "iiwa_joint_7"]
        print('The RobotMovement service initialized!')

    def _setupActionClients(self, service_config):
        self.joint_space_trajectory_action = actionlib.SimpleActionClient(service_config.joint_trajectory_action, robot_msgs.msg.JointTrajectoryAction)
        self.joint_space_trajectory_action.wait_for_server()
        self.ee_trajectory_action = actionlib.SimpleActionClient(service_config.ee_trajectory_action, robot_msgs.msg.CartesianTrajectoryAction)
        self.ee_trajectory_action.wait_for_server()

    def move_to(self, joint_position):
        # The trajectory
        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()

        # The time
        duration = 3
        start_time = rospy.Duration.from_sec(0)
        end_time = rospy.Duration.from_sec(duration)

        # The start will be override
        num_joint = len(joint_position)
        start_state = self.joint_state_from_vector([0]*num_joint)
        end_state = self.joint_state_from_vector(joint_position)
        start_point = self.trajectory_point_from_state(start_state, start_time)
        end_point = self.trajectory_point_from_state(end_state, end_time)

        # Construct the trajectory
        trajectory.points = [start_point, end_point]
        trajectory.joint_names = self._joint_names
        joint_traj_action_goal = robot_msgs.msg.JointTrajectoryGoal()
        joint_traj_action_goal.trajectory = trajectory

        # Run it
        self.joint_space_trajectory_action.send_goal(joint_traj_action_goal)
        print('Joint goal message sent. Wait for result!')
        self.joint_space_trajectory_action.wait_for_result()

        # Check the result
        result = self.joint_space_trajectory_action.get_result()
        finished_normally = (result.status.status == result.status.FINISHED_NORMALLY)
        print('Plan status', result.status.status)
        print('Has finish normally? ', finished_normally)
        return finished_normally

    def joint_state_from_vector(self, q):
        # assert len(q) == len(self._joint_names)
        joint_state = JointState()
        joint_state.name = self._joint_names
        joint_state.position = q
        return joint_state

    def move_home(self):
        q_home = [
            0.035684049129486084,
            0.6206402778625488,
            -0.03777864947915077,
            -0.5906810164451599,
            0.03872400149703026,
            1.786637783050537,
            0.4146159589290619
        ]
        self.move_to(q_home)

    def move_ee_wrt_current_ee(self, trajectory=[0, 0, -0.1]):
        goal_msg = make_cartesian_goal_trajectory_msg(trajectory, 'iiwa_link_ee', 'iiwa_link_ee')
        self.ee_trajectory_action.send_goal(goal_msg)
        print('EE goal message sent. Wait for result!')
        self.ee_trajectory_action.wait_for_result()

        # Check for result
        result = self.ee_trajectory_action.get_result()
        finished_normally = (result.status.status == result.status.FINISHED_NORMALLY)
        print('Has finish normally? ', finished_normally)
        return finished_normally

    def move_ee_wrt_world(self, trajectory=[0.636498, 0.0157858, 0.617309]):
        goal_msg = make_cartesian_goal_trajectory_msg(trajectory, 'iiwa_link_ee', 'world', speed=0.05)
        self.ee_trajectory_action.send_goal(goal_msg)
        print('EE goal message sent. Wait for result!')
        self.ee_trajectory_action.wait_for_result()

        # Check for result
        result = self.ee_trajectory_action.get_result()
        finished_normally = (result.status.status == result.status.FINISHED_NORMALLY)
        print('Plan status', result.status.status)
        print('Has finish normally? ', finished_normally)
        return finished_normally

    @staticmethod
    def trajectory_point_from_state(jointState, timeFromStart):
        trajPoint = JointTrajectoryPoint()
        trajPoint.positions = jointState.position

        numJoints = len(jointState.position)
        trajPoint.velocities = [0]*numJoints
        trajPoint.accelerations = [0]*numJoints
        trajPoint.effort = [0]*numJoints

        trajPoint.time_from_start = timeFromStart
        return trajPoint


if __name__ == '__main__':
    rospy.init_node("RobotMovementService")
    rbt_movement = RobotMovementService()
    rbt_movement.move_home()
    rbt_movement.move_to([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
    #rbt_movement.move_home()
    #rbt_movement.move_ee_wrt_current_ee()
    #rospy.sleep(rospy.Duration(secs=1))
    #rbt_movement.move_ee_wrt_world()
