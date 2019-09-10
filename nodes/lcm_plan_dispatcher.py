#! /usr/bin/env python
import argparse
import attr
import yaml

# The lcm types
import lcm
from robotlocomotion.robot_plan_t import robot_plan_t

# The ros types
import rospy
import actionlib
from robot_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint


# The argument
parser = argparse.ArgumentParser()
parser.add_argument('--config_path',
                    default='',
                    help='The path to the configuration file')


@attr.s
class KukaPlanDispatcherConfig(object):
    # The ros channel
    joint_trajectory_action = '/plan_runner/JointTrajectory'
    stop_plan_service = '/plan_runner/stop_plan'
    # The lcm channel
    lcm_command_channel = 'COMMITTED_ROBOT_PLAN'
    lcm_stop_channel = 'STOP'


class KukaPlanDispatcherLCM(object):

    def __init__(self, config=KukaPlanDispatcherConfig()):
        self._config = config
        self._joint_trajectory_action = None  # type: actionlib.SimpleActionClient

    def start(self):
        # Setup of the action
        self._joint_trajectory_action = actionlib.SimpleActionClient(
            self._config.joint_trajectory_action,
            JointTrajectoryAction)
        self._joint_trajectory_action.wait_for_server()

        # The lcm staff
        lcm_handle = lcm.LCM()
        lcm_handle.subscribe(self._config.lcm_command_channel, self.handle_joint_trajectory_plan)

        # Start processing
        try:
            rate = rospy.Rate(5)
            while True:
                lcm_handle.handle()
                rate.sleep()
        except RuntimeError:
            pass

    def handle_joint_trajectory_plan(self, channel, data):
        msg = robot_plan_t.decode(data)
        if len(msg.plan) < 2 or msg.num_states < 2:
            return

        # Fill the joint name goal msg
        goal_msg = JointTrajectoryGoal()
        plan_0 = msg.plan[0]
        for i in range(0, len(plan_0.joint_name)):
            name_i = plan_0.joint_name[i]
            goal_msg.trajectory.joint_names.append(name_i)

        # Fill the plan
        for i in range(0, len(msg.plan)):
            plan_i = msg.plan[i]
            point_i = JointTrajectoryPoint()
            point_i.time_from_start = rospy.Duration(nsecs=plan_i.utime * 1000)
            for j in range(0, len(plan_i.joint_position)):
                point_i.positions.append(plan_i.joint_position[j])
                point_i.velocities.append(0)
                point_i.accelerations.append(0)
                point_i.effort.append(0)

            # Append to trajectory
            goal_msg.trajectory.points.append(point_i)

        # Send the goal
        self._joint_trajectory_action.send_goal(goal_msg)


def main():
    # Parse the argument
    args, unknown = parser.parse_known_args()
    config_path = args.config_path  # type: str

    # Construct the the config
    dispatcher_config = KukaPlanDispatcherConfig()
    if len(config_path) > 0:
        with open(config_path) as config_file:
            datamap = yaml.load(config_file, Loader=yaml.CLoader)
            dispatcher_config.lcm_command_channel = datamap['lcm_plan_channel']
            dispatcher_config.lcm_stop_channel = datamap['lcm_stop_channel']
            config_file.close()

    # OK
    rospy.init_node('lcm_command_bridge')
    dispatcher = KukaPlanDispatcherLCM(dispatcher_config)
    dispatcher.start()


if __name__ == '__main__':
    main()
