# system
import numpy as np

# ROS
import rospy
import actionlib
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ROS custom packages
import robot_msgs.msg


class RobotMovementService(object):

    def __init__(self, joint_trajectory_action="/plan_runner/iiwa/JointTrajectory"):
        self._setupActionClients(joint_trajectory_action)
        self._joint_names = [
            "iiwa_joint_1",
            "iiwa_joint_2",
            "iiwa_joint_3",
            "iiwa_joint_4",
            "iiwa_joint_5",
            "iiwa_joint_6",
            "iiwa_joint_7"]
        print('The RobotMovement service initialized!')

    def _setupActionClients(self, joint_trajectory_action):
        self.joint_space_trajectory_action = actionlib.SimpleActionClient(joint_trajectory_action, robot_msgs.msg.JointTrajectoryAction)
        self.joint_space_trajectory_action.wait_for_server()

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
        print('Goal message sent. Wait for result!')
        self.joint_space_trajectory_action.wait_for_result()

        # Check the result
        result = self.joint_space_trajectory_action.get_result()
        finished_normally = (result.status.status == result.status.FINISHED_NORMALLY)
        return finished_normally


    def joint_state_from_vector(self, q):
        assert len(q) == len(self._joint_names)
        joint_state = JointState()
        joint_state.name = self._joint_names
        joint_state.position = q
        return joint_state


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
    q = [1.3, 0, 0, 0, 0, 0, 0]
    rbt_movement.move_to(q)
