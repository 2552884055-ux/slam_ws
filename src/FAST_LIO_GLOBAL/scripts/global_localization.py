#!/usr/bin/env python3
# coding=utf8


from __future__ import print_function, division, absolute_import

import sys
import os
sys.path.insert(0, os.path.expanduser('~/.local/lib/python3.8/site-packages'))
sys.path.insert(0, os.path.expanduser('/opt/ros/noetic/lib/python3/dist-packages'))

import copy
import _thread
import time
import signal


import open3d as o3d
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import numpy as np
import tf
import tf.transformations


def signal_handler(sig, frame):
    rospy.loginfo("Received termination signal. Shutting down gracefully...")
    # 在这里执行退出前的清理工作
    rospy.signal_shutdown("Shutting down...")
    sys.exit(0)  # 退出程序

# 捕获SIGINT和SIGTERM信号
signal.signal(signal.SIGINT, signal_handler)  # 捕获Ctrl+C (SIGINT)
signal.signal(signal.SIGTERM, signal_handler)  # 捕获系统发送的SIGTERM


global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc

# criteria default=ICPConvergenceCriteria class with relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30
def registration_at_scale(pc_scan, pc_map, initial, scale):
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        1.0 * scale, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000)
    )

    return result_icp.transformation, result_icp.fitness

def registration_at_scale_fpfh_sansac(pc_scan, pc_map, initial, scale):
    pc_scan_down = voxel_down_sample(pc_scan, 0.05)
    pc_map_down = voxel_down_sample(pc_map, 0.05)

    pc_scan_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pc_map_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # FPFH
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    pc_scan_down, o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=100))

    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    pc_map_down, o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=100))

    # # sansac
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    pc_scan_down, pc_map_down, source_fpfh, target_fpfh, True,
    0.2,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
    3, [
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.2)
    ],
    o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.95)
    )


    rospy.logwarn('sansac fitness score:{}'.format(result_ransac.fitness))
    return result_ransac.transformation, result_ransac.fitness


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    data = np.zeros(len(pc), dtype=[
        ('x', np.float64),
        ('y', np.float64),
        ('z', np.float64),
        ('intensity', np.float64),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    # 当前scan原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    # 发布fov内点云
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV


def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom ,fitness
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)

    # 粗配准
    # icp
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)
    # fpfh + sansac
    # if fitness < 0.995:
    #     transformation, _ = registration_at_scale_fpfh_sansac(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)
    # else:
    #     transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    rospy.loginfo('Time: {}'.format(toc - tic))
    rospy.loginfo('')

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = transformation

        # 发布map_to_odom
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = 'map'
        pub_map_to_odom.publish(map_to_odom)
        rospy.loginfo('fitness score:{}'.format(fitness))
        return True
    else:
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('{}'.format(transformation))
        rospy.logwarn('fitness score:{}'.format(fitness))
        return False


def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


def initialize_global_map(pc_msg):
    global global_map

    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    rospy.loginfo('Global map received.')


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()
    pub_pc_in_map.publish(pc_msg)

    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)

    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])


def thread_localization():
    global T_map_to_odom
    while True:
        # 每隔一段时间进行全局定位
        rospy.sleep(1 / FREQ_LOCALIZATION)
        # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
        global_localization(T_map_to_odom)


if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.4
    SCAN_VOXEL_SIZE = 0.1
    # MAP_VOXEL_SIZE = 0.05
    # SCAN_VOXEL_SIZE = 0.01

    # Global localization frequency (HZ)
    # FREQ_LOCALIZATION = 0.5
    FREQ_LOCALIZATION = 2

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    # LOCALIZATION_TH = 0.8
    LOCALIZATION_TH = 0.95

    # FOV(rad), modify this according to your LiDAR type
    FOV = 6.28319

    # The farthest distance(meters) within FOV
    FOV_FAR = 30

    fitness = 0

    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

    pub_map_to_odom_flag = rospy.Publisher('/map_to_odom_flag', Bool, queue_size=1)
    
    # 新增：发布等待初始位姿的标志位
    pub_waiting_for_initial_pose = rospy.Publisher('/waiting_for_initial_pose', Bool, queue_size=1)

    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)

    # 初始化全局地图
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('pcd_map', PointCloud2))

    # test
    # quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    # xyz = [0, 0, 0]
    # pose_msg = PoseWithCovarianceStamped()
    # pose_msg.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
    # pose_msg.header.stamp = rospy.Time().now()
    # pose_msg.header.frame_id = 'map'

    # initial_pose = pose_to_mat(pose_msg)

    # 初始化
    while not initialized:
        rospy.logwarn('Waiting for initial pose....')
        
        # 发布等待初始位姿标志
        wait_msg = Bool()
        wait_msg.data = True
        pub_waiting_for_initial_pose.publish(wait_msg)
        rospy.loginfo("Published waiting_for_initial_flag: %s", wait_msg.data)

        # 等待初始位姿
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        initial_pose = pose_to_mat(pose_msg)

        if cur_scan:
            initialized = global_localization(initial_pose)
        else:
            rospy.logwarn('First scan not received!!!!!')
        time.sleep(1)

    rospy.loginfo('')
    rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')

    # 
    msg = Bool()
    msg.data = True

    
    start_time = rospy.Time.now()
    duration = rospy.Duration(10)  # 持续10秒
    # 持续发布消息
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < duration:
        pub_map_to_odom_flag.publish(msg)
        rospy.loginfo("Published map_to_odom_flag: %s", msg.data)

        # 可选：控制发布频率
        rospy.sleep(0.5)  # 每0.1秒发布一次    
    
    pub_map_to_odom_flag.publish(msg)


    # 开始定期全局定位
    # _thread.start_new_thread(thread_localization, ())
    rospy.spin()
    time.sleep(1)

    
