#!/usr/bin/python

import os

import rospkg
import rospy
import rosbag

import math
import numpy as np

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def get_gnss_ins_sim(motion_def_file, fs_imu, fs_gps):
    '''
    Generate simulated GNSS/IMU data using specified trajectory.
    '''
    # set IMU model:
    D2R = math.pi/180.0
    imu_err = 'low-accuracy'
    # generate GPS and magnetometer data:
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    # init simulation:
    sim = ins_sim.Sim(
        [fs_imu, fs_gps, fs_imu],
        motion_def_file,
        ref_frame=1,
        imu=imu,
        mode=None,
        env=None,
        algorithm=None
    )
    
    # run:
    sim.run(1)
    sim.plot(['ref_pos','ref_vel' , 'gyro', 'accel'], opt={'ref_pos': 'projection'})


    # get simulated data:
    rospy.logwarn(
        "Simulated data size {}".format(
            len(sim.dmgr.get_data_all('gyro').data[0])
        )
    )

    # imu measurements:
    step_size = 1.0 / fs_imu
    print('gyro',sim.dmgr.get_data_all('gyro').data[0])
    print('accel',sim.dmgr.get_data_all('accel').data[0])
    print('ref_pos',sim.dmgr.get_data_all('ref_pos').data)
    print('ref_vel',sim.dmgr.get_data_all('ref_vel').data)
    print('ref_att_quat',sim.dmgr.get_data_all('ref_att_quat').data)
    print('gps',sim.dmgr.get_data_all('gps').data[0])

    for i, (gyro, accel, pose, vel, ref_att_quat) in enumerate(
        zip(
            # a. gyro
            sim.dmgr.get_data_all('gyro').data[0], 
            # b. accel
            sim.dmgr.get_data_all('accel').data[0],
            # c. position
            sim.dmgr.get_data_all('ref_pos').data,
            # d. vel
            sim.dmgr.get_data_all('ref_vel').data,
            # e. quaternion
            sim.dmgr.get_data_all('ref_att_quat').data,
        )
    ):
        yield {
            'stamp': i * step_size,
            'data': {
                # a. gyro:
                'gyro_x': gyro[0],
                'gyro_y': gyro[1],
                'gyro_z': gyro[2],
                # b. accel:
                'accel_x': accel[0],
                'accel_y': accel[1],
                'accel_z': accel[2],
                # c. pos:
                'pos_x': pose[0],
                'pos_y': pose[1],
                'pos_z': pose[2],
                # d. vel:
                'vel_x': vel[0],
                'vel_y': vel[1],
                'vel_z': vel[2],
                # e. quaternion
                'q_w': ref_att_quat[0],
                'q_x': ref_att_quat[1],
                'q_y': ref_att_quat[2],
                'q_z': ref_att_quat[3]
            }
        }


def gnss_ins_sim_recorder():
    """
    Record simulated GNSS/IMU data as ROS bag
    """
    # ensure gnss_ins_sim_node is unique:
    rospy.init_node('gnss_ins_sim_recorder_node')

    # parse params:
    motion_def_name = rospy.get_param('/gnss_ins_sim_recorder_node/motion_file')
    sample_freq_imu = rospy.get_param('/gnss_ins_sim_recorder_node/sample_frequency/imu')
    sample_freq_gps = rospy.get_param('/gnss_ins_sim_recorder_node/sample_frequency/gps')
    topic_name_imu = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name')
    rosbag_output_path = rospy.get_param('/gnss_ins_sim_recorder_node/output_path')
    rosbag_output_name = rospy.get_param('/gnss_ins_sim_recorder_node/output_name')
    topic_name_odom = '/pose/ground_truth'

    # generate simulated data:
    motion_def_path = os.path.join(
        rospkg.RosPack().get_path('gnss_ins_sim'), 'config', 'motion_def', motion_def_name
    )
    imu_simulator = get_gnss_ins_sim(
        # motion def file:
        motion_def_path,
        # gyro-accel/gyro-accel-mag sample rate:
        sample_freq_imu,
        # GPS sample rate:
        sample_freq_gps
    )

    with rosbag.Bag(
        os.path.join(rosbag_output_path, rosbag_output_name), 'w'
    ) as bag:
        # get timestamp base:
        timestamp_start = rospy.Time.now()

        for i,measurement in enumerate(imu_simulator):
            if (i == 0):
                ini_x = measurement['data']['pos_x']
                ini_y = measurement['data']['pos_y']
                ini_z = measurement['data']['pos_z']
            # init:
            msg = Imu()
            # a. set header:
            msg.header.frame_id = 'NED'
            msg.header.stamp = timestamp_start + rospy.Duration.from_sec(measurement['stamp'])
            # b. set orientation estimation:
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0
            # c. gyro:
            msg.angular_velocity.x = measurement['data']['gyro_x']
            msg.angular_velocity.y = measurement['data']['gyro_y']
            msg.angular_velocity.z = measurement['data']['gyro_z']
            msg.linear_acceleration.x = measurement['data']['accel_x']
            msg.linear_acceleration.y = measurement['data']['accel_y']
            msg.linear_acceleration.z = measurement['data']['accel_z']

            # write:
            bag.write(topic_name_imu, msg, msg.header.stamp)

            msg_odom = Odometry()
            msg_odom.header.frame_id = 'inertial'
            msg_odom.child_frame_id = 'inertial'
            msg_odom.header.stamp = timestamp_start + rospy.Duration.from_sec(measurement['stamp'])
            # b. set orientation:
            msg_odom.pose.pose.orientation.x = measurement['data']['q_x']
            msg_odom.pose.pose.orientation.y = measurement['data']['q_y']
            msg_odom.pose.pose.orientation.z = measurement['data']['q_z']
            msg_odom.pose.pose.orientation.w = measurement['data']['q_w']

            # c. set position:
            msg_odom.pose.pose.position.x = measurement['data']['pos_x']-ini_x
            msg_odom.pose.pose.position.y = measurement['data']['pos_y']-ini_y
            msg_odom.pose.pose.position.z = measurement['data']['pos_z']-ini_z

            # d. set velocity:
            msg_odom.twist.twist.linear.x = measurement['data']['vel_x']
            msg_odom.twist.twist.linear.y = measurement['data']['vel_y']
            msg_odom.twist.twist.linear.z = measurement['data']['vel_z']
            bag.write(topic_name_odom, msg_odom, msg_odom.header.stamp)

if __name__ == '__main__':
    try:
        gnss_ins_sim_recorder()
    except rospy.ROSInterruptException:
        pass