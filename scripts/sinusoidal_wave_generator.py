#!/usr/bin/env python
import rospy
import numpy as np
import actionlib
import trajectory_msgs.msg
import control_msgs.msg



if __name__ == "__main__":
    rospy.init_node('sinusoidal_wave_generator')
    command_topic = rospy.get_param('~command_topic')
    command_joint = rospy.get_param('~command_joint')
    wave_mag = rospy.get_param('~wave_mag')
    wave_offset = rospy.get_param('~wave_offset')
    wave_freq = rospy.get_param('~wave_freq')

    publisher = rospy.Publisher(command_topic, trajectory_msgs.msg.JointTrajectory, queue_size=10)
    # client = actionlib.SimpleActionClient(command_topic, control_msgs.msg.FollowJointTrajectoryAction)
    
    # joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
    point = trajectory_msgs.msg.JointTrajectoryPoint()
    point.positions = [0.0]
    point.velocities = []
    point.accelerations = []
    point.time_from_start = rospy.Duration().from_sec(0.1)

    traj = trajectory_msgs.msg.JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = [command_joint]
    traj.points.append(point)
    # goal = control_msgs.msg.FollowJointTrajectoryGoal()
    # goal.trajectory.header.stamp = rospy.Time.now()
    # goal.trajectory.joint_names = [command_joint]
    # goal.trajectory.points.append(point)

    r = rospy.Rate(wave_freq) # 50 hz
    while not rospy.is_shutdown():
        traj.header.stamp = rospy.Time.now()
        # goal.trajectory.header.stamp = rospy.Time.now()
        # point.positions[joint_names.index(command_joint)] = wave_mag * np.sin((2 * np.pi / 50) * rospy.Time.now().to_sec())
        # point.positions[joint_names.index(command_joint)] = wave_mag * np.sin(rospy.Time.now().to_sec())
        # point.positions[0] = wave_mag * np.sin(rospy.Time.now().to_sec())
        # client.send_goal_and_wait(goal, rospy.Duration(0), rospy.Duration(0))
        point.positions[0] = wave_mag * np.sin(rospy.Time.now().to_sec()) + wave_offset
        # client.send_goal(goal, rospy.Duration(0), rospy.Duration(0))
        publisher.publish(traj)
        r.sleep()

