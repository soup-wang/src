#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@文件        :imu_tf.py
@说明        :发布gpstf转换和imu 校正转化
@时间        :2020/09/04 06:42:08
@作者        :王美汤
@版本        :1.0
'''

import turtlesim.msg
import tf
import rospy
import roslib

from sensor_msgs.msg import Imu, NavSatFix
# 获取loca位置
from geometry_msgs.msg import PoseStamped
import math

from pyquaternion import Quaternion

from mavros_msgs.msg import HomePosition

import gps_utiles
# roslib.load_manifest('learning_tf')

# 使用 tf 完成坐标变换，订阅这个topic "turtleX/pose" 并且对每一条msg进行以下计算

class tf_map:

    def __init__(self):
        rospy.init_node('turtle_tf_broadcaster')  # 添加新的节点

        rospy.Subscriber(
            "/mavros/imu/data", Imu, self.imu_callback)
        # 获取罗盘值
        rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.GPS_callback)

        # 获取home点值
        rospy.Subscriber(
            "/mavros/home_position/home", HomePosition, self.home_GPS_callback)

        self.yaw = None
        self.home_gps = (0,0)
        rospy.spin()


    def imu_callback(self,msg):

        # pitch=180∗atan2(accelX,sqrt(accelY∗accelY+accelZ∗accelZ))/PI;
        # roll=180∗atan2(accelY,sqrt(accelX∗accelX+accelZ∗accelZ))/PI;
        rollxyz = msg.linear_acceleration
        roll = math.atan2(rollxyz.y, math.sqrt(
            rollxyz.x ** 2 + rollxyz.z ** 2))
        pitch = math.atan2(rollxyz.x, math.sqrt(
            rollxyz.y ** 2+rollxyz.z ** 2))

        self.yaw = self.q2yaw(msg.orientation)
        # print ("roll",roll)
        # print ("pitch",pitch)

        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0.3),
                        tf.transformations.quaternion_from_euler(
                            -pitch, -roll,math.pi),
                        rospy.Time.now(),
                        "base_link",
                        "world")  # 发布乌龟的平移和翻转
        #  imu_calibration_link


    def q2yaw(self,q):
        '''
        四元数转为yaw

        '''
        q_ = Quaternion(q.w, q.x, q.y, q.z)
        rotate_z_rad = q_.yaw_pitch_roll[0]
        yaw = (-rotate_z_rad / math.pi * 180.0) % 360
        return yaw


    def GPS_callback(self,msg):
        '''
            获取GPS 信息
            msg: 消息格式

            header: 
                seq: 110
                stamp: 
                    secs: 1598234193
                    nsecs: 418708509
                frame_id: "base_link" 
            status: gps 状态
                status: -1
                service: 1
            latitude: 0.0  纬度
            longitude: 0.0  经度
            altitude: 16.433    高度
            position_covariance: [18446744065119.617, 0.0, 0.0, 0.0, 18446744065119.617, 0.0, 0.0, 0.0, 14184584949071.611] 位置协方差
            position_covariance_type: 2
        '''
        current_gps = (msg.longitude, msg.latitude)
        if current_gps != None and self.yaw != None:
            meter = gps_utiles.gps_to_meter_coord(current_gps, self.home_gps)

            gps_tf = tf.TransformBroadcaster()
            q = tf.transformations.quaternion_from_euler(0, 0, (-self.yaw )*math.pi/180)
            gps_tf.sendTransform((meter[0], meter[1], 0),
                                q,
                                rospy.Time.now(),
                                "world",
                                "map")  # 发布乌龟的平移和翻转
            #  imu_calibration_link


    def home_GPS_callback(self,msg):
        '''
            header: 
                seq: 1
            stamp: 
                secs: 1599204520
                nsecs: 710171079
                frame_id: ''
            geo: 
                latitude: 31.1285234
                longitude: 121.4201952
                altitude: 19.8317872565
            position: 
                x: 0.0
                y: 0.0
                z: -0.0
            orientation: 
                x: 0.707106781187
                y: 0.707106781187
                z: 4.32978028118e-17
                w: 4.32978028118e-17
            approach: 
                x: 0.0
                y: 0.0
                z: -0.0
            获取home_GPS位置 
        '''
        self.home_gps = (msg.geo.longitude, msg.geo.latitude)


if __name__ == '__main__':
    tf_map()
