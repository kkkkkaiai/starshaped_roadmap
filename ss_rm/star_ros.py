#!/usr/bin/env python
import rospy
import os
import time

from starshaped_re.graph import GraphManager
from starshaped_re.starshaped_fit import StarshapedRep
from starshaped_re.modulation import modulation_velocity

# ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import Polygon, PolygonStamped
from geometry_msgs.msg import Point32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetModelStateRequest, SetModelState

from copy import copy
import numpy as np
import numpy.linalg as LA

from fast_obstacle_avoidance.obstacle_avoider.lidar_avoider import SampledAvoider
from vartools.dynamical_systems import LinearSystem

robot_c = None
robot_yaw = None
robot_v = None
robot_w = None
is_robot_init = False

laser = None
laser_points = []
laser_points_filter_max_range = []
is_laser_init = False
resolusion = 0.05

arrow_array_pub = None

def odom_cb(msg):
    global robot_c, robot_yaw, is_robot_init, robot_v, robot_w
    robot_c = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    robot_yaw = np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)*2
    robot_v = msg.twist.twist.linear.x
    robot_w = msg.twist.twist.angular.z
    if not is_robot_init:
        is_robot_init = True

def laser_callback(msg):
    global laser_points, laser_points_filter_max_range, is_laser_init
    laser_points = []
    laser_points_filter_max_range = []
    inf = float('inf')
    is_max_data = False
    for i in range(len(msg.ranges)):
        # check if the point is inf
        point_data = msg.ranges[i]
        if point_data == inf:
            is_max_data = True
            point_data = msg.range_max - 0.0001

        if point_data < msg.range_max and point_data > msg.range_min:
            angle = msg.angle_min + i * msg.angle_increment
            x = point_data * np.cos(angle)
            y = point_data * np.sin(angle)
            laser_points.append([x, y])
            if not is_max_data:
                laser_points_filter_max_range.append([x, y])

        is_max_data = False
        if len(laser_points) > 0:
            is_laser_init = True

def publish_cmd_vel(pub, control_signal):
    cmd = Twist()
    cmd.linear.x = control_signal[0]
    cmd.angular.z = control_signal[1]
    pub.publish(cmd)

def publish_polygon(pub, points):
    polygon = PolygonStamped()
    polygon.header.frame_id = "odom"
    for i in range(len(points)):
        point = Point32()
        point.x = points[i][0]
        point.y = points[i][1]
        polygon.polygon.points.append(point)
    pub.publish(polygon)

def publish_polygon_array(pub, points_array):
    polygon_array = PolygonArray()
    polygon_array.header.frame_id = "odom"
    for i in range(len(points_array)):
        polygon_stamp = PolygonStamped()
        polygon_stamp.header.frame_id = "odom"
        for j in range(len(points_array[i])):
            point = Point32()
            point.x = points_array[i][j][0]
            point.y = points_array[i][j][1]
            polygon_stamp.polygon.points.append(point)
        polygon_array.polygons.append(polygon_stamp)
    
    pub.publish(polygon_array)

def publish_trajectory(pub, trajectory):
    path = Path()
    path.header.frame_id = "odom"
    for i in range(len(trajectory)):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = trajectory[i][0]
        pose.pose.position.y = trajectory[i][1]
        pose.pose.position.z = 0.2
        path.poses.append(pose)
    pub.publish(path)

def publish_frontier(pub, frontier_points):
    marker_array = MarkerArray()
    for i in range(len(frontier_points)):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = frontier_points[i][0]
        marker.pose.position.y = frontier_points[i][1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)
    pub.publish(marker_array)


def publish_subgoal(pub, goal_position):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "basic_shapes"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = goal_position[0]
    marker.pose.position.y = goal_position[1]
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    pub.publish(marker)

def publish_path(pub, path_list):
    path = Path()
    path.header.frame_id = "odom"
    for i in range(len(path_list)):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = path_list[i][0]
        pose.pose.position.y = path_list[i][1]
        path.poses.append(pose)
    pub.publish(path)

def publish_arrow(pub, position, yaw, id, color=[0.5, 0.5, 0.5]):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "basic_shapes"
    marker.id = id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = 0.16
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = np.sin(yaw/2)
    marker.pose.orientation.w = np.cos(yaw/2)
    marker.scale.x = 0.5
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    pub.publish(marker)

def publish_arrow_array(pub, arrow_list_with_position):
    # the first element in col is the position, the second element is the yaw
    marker_array = MarkerArray()
    for i in range(len(arrow_list_with_position)):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.id = i-8
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = arrow_list_with_position[i][0][0]
        marker.pose.position.y = arrow_list_with_position[i][0][1]
        marker.pose.position.z = 0.16
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = np.sin(arrow_list_with_position[i][1]/2)
        marker.pose.orientation.w = np.cos(arrow_list_with_position[i][1]/2)
        marker.scale.x = 0.2
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)
    pub.publish(marker_array)

def clear_road_map(pub):
    marker_array = MarkerArray()
    for i in range(-10, 500):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "road_maps"
        marker.id = i
        marker.action = Marker.DELETE
        marker_array.markers.append(marker)
    pub.publish(marker_array)

def display_road_map(pub, graph_manager):
    # the node in the graph is displayed as a cuboid
    # the edge in the graph is displayed as a line
    marker_array = MarkerArray()
    start_id = graph_manager.start_id
    for node_id in graph_manager._nodes:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "road_maps"
        marker.id = node_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = graph_manager._nodes[node_id]._position[0]
        marker.pose.position.y = graph_manager._nodes[node_id]._position[1]
        marker.pose.position.z = 0.25
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.05
        marker.color.a = 0.8
        if node_id == start_id:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        marker_array.markers.append(marker)

        # judge the node is starshaped
        if node_id in graph_manager._star_id_list:
            edges = graph_manager.get_neighbor(node_id)
            for edge in edges:
                lines = Marker()
                lines.header.frame_id = "odom"
                lines.header.stamp = rospy.Time.now()
                lines.ns = "basic_shapes"
                lines.id = 100 + edge
                lines.type = Marker.LINE_STRIP
                lines.action = Marker.ADD
                point_from = Point32()
                point_from.x = graph_manager._nodes[node_id]._position[0]
                point_from.y = graph_manager._nodes[node_id]._position[1]
                point_from.z = 0.1
                lines.points.append(point_from)
                point_to = Point32()
                point_to.x = graph_manager._nodes[edge]._position[0]
                point_to.y = graph_manager._nodes[edge]._position[1]
                point_to.z = 0.25
                lines.points.append(point_to)
                lines.pose.orientation.w = 1.0
                lines.scale.x = 0.01
                lines.color.a = 0.8
                lines.color.r = 0.5
                lines.color.g = 0.5
                lines.color.b = 1.0
                marker_array.markers.append(lines)
            
    pub.publish(marker_array)

def publish_goal_position(pub, goal_position):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "goal_position"
    marker.id = -10
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = goal_position[0]
    marker.pose.position.y = goal_position[1]
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    pub.publish(marker)

def transform_to_global_frame(points, robot_c, robot_yaw):
    points = np.array(points)
    points = points.T
    try:
        points = np.vstack((points, np.ones(points.shape[1])))
    except:
        print('Empty laser points, rerun the node')
        exit()
    transform_matrix = np.array([[np.cos(robot_yaw), -np.sin(robot_yaw), robot_c[0]], 
                  [np.sin(robot_yaw), np.cos(robot_yaw), robot_c[1]], 
                  [0, 0, 1]])
    points = np.dot(transform_matrix, points)
    return points[:2].T


def modulation_round_robot(position, robot_yaw, local_position, graph_manager, safety_margin=0.5, pub=True):
    # calculate the gamma of current position
    current_velocity, current_min_Gamma = modulation_velocity(position, local_position, graph_manager)
    current_velocity = current_velocity / np.linalg.norm(current_velocity)

    # front, left, right point
    # use yaw to calcute the front, left and right point
    front_point = position + np.array([np.cos(robot_yaw), np.sin(robot_yaw)]) * safety_margin
    left_point = position + np.array([-np.sin(robot_yaw), np.cos(robot_yaw)]) * safety_margin
    right_point = position + np.array([np.sin(robot_yaw), -np.cos(robot_yaw)]) * safety_margin
    print('point', front_point, left_point, right_point)

    # the gamma of the front, left and right point
    front_velocity, front_min_Gamma = modulation_velocity(front_point, local_position, graph_manager)
    left_velocity, left_min_Gamma = modulation_velocity(left_point, local_position, graph_manager)
    right_velocity, right_min_Gamma = modulation_velocity(right_point, local_position, graph_manager)
    
    if current_min_Gamma == np.inf:
        current_min_Gamma = 1e10
    if front_min_Gamma == np.inf:
        front_min_Gamma = 1e10
    if left_min_Gamma == np.inf:
        left_min_Gamma = 1e10
    if right_min_Gamma == np.inf:
        right_min_Gamma = 1e10

    if pub:
        arrow_array_list = []
        # publish the arrow marker, the second element is the yaw of the modulated velocity
        arrow_array_list.append([position, np.arctan2(current_velocity[1], current_velocity[0])])
        arrow_array_list.append([front_point, np.arctan2(front_velocity[1], front_velocity[0])])
        arrow_array_list.append([left_point, np.arctan2(left_velocity[1], left_velocity[0])])
        arrow_array_list.append([right_point, np.arctan2(right_velocity[1], right_velocity[0])])
        # print('pubpubpub', arrow_array_list)
        publish_arrow_array(arrow_array_pub, arrow_array_list)

    # computing weight based on the Gamma value
    weight_temp = np.array([front_min_Gamma, left_min_Gamma, right_min_Gamma, current_min_Gamma])
    # filter the weight with the lower of 1.35
    weight = copy(weight_temp)
    # get the index of the weight that is lower than 1.35 and higher than 1
    weight_import_1 = np.where(weight < 1.5)
    weight_import_2 = np.where(weight > 0.8)
    print(weight_import_1, weight_import_2, weight)
    weight_import = np.intersect1d(weight_import_1, weight_import_2)

    new_velocity = copy(current_velocity)
    if len(weight_import) != 0:
        weight_temp = np.zeros(4)
        if len(weight_import) != 4:
            for i in range(len(weight_import)):
                weight_temp[weight_import[i]] = weight[weight_import[i]]
        else:
            weight_temp = copy(weight)
        print(weight_temp)
        weight = weight_temp / np.sum(weight_temp)
        
        # calculate the new velocity
        new_velocity = front_velocity * weight[0] + left_velocity * weight[1] + right_velocity * weight[2] + current_velocity * weight[3]
     
    return new_velocity

def modulation_rectangle_robot(position, robot_yaw, local_position, graph_manager, safety_margin=0.5, pub=True):
    current_velocity, current_max_Gamma = modulation_velocity(position, local_position, graph_manager)
    current_velocity = current_velocity / np.linalg.norm(current_velocity)

    # left front, right front, left back, right back point
    # use yaw to calcute the left front, right front, left back and right back point, use pi/4 as a bias
    left_front_point = position + np.array([np.cos(robot_yaw + np.pi/4), np.sin(robot_yaw + np.pi/4)]) * safety_margin
    right_front_point = position + np.array([np.cos(robot_yaw - np.pi/4), np.sin(robot_yaw - np.pi/4)]) * safety_margin
    left_back_point = position + np.array([np.cos(robot_yaw + np.pi - np.pi/4), np.sin(robot_yaw + np.pi - np.pi/4)]) * safety_margin
    right_back_point = position + np.array([np.cos(robot_yaw + np.pi + np.pi/4), np.sin(robot_yaw + np.pi + np.pi/4)]) * safety_margin


    # the gamma of the left front, right front, left back and right back point
    left_front_velocity, left_front_max_Gamma = modulation_velocity(left_front_point, local_position, graph_manager)
    right_front_velocity, right_front_max_Gamma = modulation_velocity(right_front_point, local_position, graph_manager)
    left_back_velocity, left_back_max_Gamma = modulation_velocity(left_back_point, local_position, graph_manager)
    right_back_velocity, right_back_max_Gamma = modulation_velocity(right_back_point, local_position, graph_manager)

    if current_max_Gamma == np.inf:
        current_max_Gamma = 1e10
    if left_front_max_Gamma == np.inf:
        left_front_max_Gamma = 1e10
    if right_front_max_Gamma == np.inf:
        right_front_max_Gamma = 1e10
    if left_back_max_Gamma == np.inf:
        left_back_max_Gamma = 1e10
    if right_back_max_Gamma == np.inf:
        right_back_max_Gamma = 1e10

    if pub:
        arrow_array_list = []
        # publish the arrow marker, the second element is the yaw of the modulated velocity
        arrow_array_list.append([position, np.arctan2(current_velocity[1], current_velocity[0])])
        arrow_array_list.append([left_front_point, np.arctan2(left_front_velocity[1], left_front_velocity[0])])
        arrow_array_list.append([right_front_point, np.arctan2(right_front_velocity[1], right_front_velocity[0])])
        arrow_array_list.append([left_back_point, np.arctan2(left_back_velocity[1], left_back_velocity[0])])
        arrow_array_list.append([right_back_point, np.arctan2(right_back_velocity[1], right_back_velocity[0])])

        publish_arrow_array(arrow_array_pub, arrow_array_list)

    # computing weight based on the Gamma value
    weight_temp = np.array([left_front_max_Gamma, right_front_max_Gamma, left_back_max_Gamma, right_back_max_Gamma, current_max_Gamma])
    # filter the weight with the lower of 1.35
    weight = copy(weight_temp)
    # get the index of the weight that is lower than 1.35 and higher than 0.5
    weight_import_1 = np.where(weight < 1.5)
    weight_import_2 = np.where(weight > 0.95)
    weight_import = np.intersect1d(weight_import_1, weight_import_2)
    # print(weight_import_1, weight_import_2, weight)
    
    new_velocity = copy(current_velocity)
    if len(weight_import) != 0:
        weight_temp = np.zeros(5)
        if len(weight_import) != 5:
            for i in range(len(weight_import)):
                weight_temp[weight_import[i]] = weight[weight_import[i]]
        else:
            weight_temp = copy(weight)
        weight = weight_temp / np.sum(weight_temp)
        
        # calculate the new velocity
        new_velocity = left_front_velocity * weight[0] + right_front_velocity * weight[1] + left_back_velocity * weight[2] + right_back_velocity * weight[3] + current_velocity * weight[4]

    return new_velocity

def modulation_nearest_point(position, local_position, laser_points, graph_manager, safety_margin=0.5, pub=True):
    # find the nearst point in the laser_points
    min_distance = 1e10
    nearest_point = None
    for i in range(len(laser_points)):
        distance = np.linalg.norm(position - laser_points[i])
        if distance < min_distance:
            min_distance = distance
            nearest_point = copy(laser_points[i])

    # print('nearst point', nearest_point)
    current_velocity, current_min_Gamma = modulation_velocity(position, local_position, graph_manager)
    current_velocity = current_velocity / np.linalg.norm(current_velocity)

    # the gamma of the nearest point
    nearest_velocity, nearest_min_Gamma = modulation_velocity(nearest_point, local_position, graph_manager)

    if current_min_Gamma == np.inf:
        current_min_Gamma = 1e10
    if nearest_min_Gamma == np.inf:
        nearest_min_Gamma = 1e10

    if pub:
        arrow_array_list = []
        # publish the arrow marker, the second element is the yaw of the modulated velocity
        arrow_array_list.append([position, np.arctan2(current_velocity[1], current_velocity[0])])
        arrow_array_list.append([nearest_point, np.arctan2(nearest_velocity[1], nearest_velocity[0])])

        publish_arrow_array(arrow_array_pub, arrow_array_list)

    # computing weight based on the Gamma value
    # weight_temp = np.array([nearest_min_Gamma, current_min_Gamma])
    # filter the weight with the lower of 1.35
    # weight = copy(weight_temp)

    new_velocity = copy(current_velocity)
    if min_distance < safety_margin * 1.5:
        weight_temp = np.zeros(2)

        # calculate the new velocity
        new_velocity = nearest_velocity
    
    return new_velocity
                                
def modulation_nearest_group(position, local_position, laser_points, graph_manager, safety_margin=0.5, pub=True, combine_pulsive=False):
    # find the nearst point in the laser_points
    min_distance = 1e10
    nearest_point = None
    
    for i in range(len(laser_points)):
        distance = np.linalg.norm(position - laser_points[i])
       
        if distance < min_distance:
            min_distance = distance
            nearest_point = copy(laser_points[i])

    # current_velocity, current_min_Gamma = modulation_velocity(position, local_position, graph_manager)
    current_velocity = (local_position - position) / np.linalg.norm(local_position - position)


    # check the distance bewteen position and local_position
    if np.linalg.norm(position - local_position) < safety_margin:
        return current_velocity

    # calculate the angle of the nearest point
    nearest_angle = np.arctan2(nearest_point[1] - position[1], nearest_point[0] - position[0])
    # calculate the direction from the nearest point to current position
    direction = (nearest_point - position) / np.linalg.norm(nearest_point - position)

    # calculate the 7 nearest angles of the nearest point with 0.05 rad
    nearest_angles = []
    for i in range(-3, 4):
        nearest_angles.append(nearest_angle + i * 0.02)

    nearest_points = []
    for i in range(len(nearest_angles)):
        nearest_points.append(position + np.array([np.cos(nearest_angles[i]), np.sin(nearest_angles[i])]) * (min_distance-0.05))
    
    # the combined velocity vector of the nearest points
    velocity_vectors = []
    gamma_values = []
    if combine_pulsive:
        combine_pulsive_vectors = []
    for i in range(len(nearest_points)):
        velocity, gamma = modulation_velocity(nearest_points[i], local_position, graph_manager)
        velocity_vectors.append(velocity)
        if combine_pulsive:
            compulsive_vector = (position - nearest_points[i]) / np.linalg.norm(position - nearest_points[i])
            combine_pulsive_vectors.append(compulsive_vector)
        gamma_values.append(gamma)

    if pub:
        arrow_array_list = []
        for i in range(len(nearest_points)):
            arrow_array_list.append([nearest_points[i], np.arctan2(velocity_vectors[i][1], velocity_vectors[i][0])])
        publish_arrow_array(arrow_array_pub, arrow_array_list)
    
    # computing weight based on the Gamma value
    weight_temp = 1 / np.array(gamma_values)
    # print('gamma', gamma_values)
    
    new_velocity = copy(current_velocity)
    if  min_distance < safety_margin * 2.5:
        weight = weight_temp / np.sum(weight_temp)

        # calculate the new velocity
        weighted_velocity = np.zeros(2)
        for i in range(len(nearest_points)):
            weighted_velocity += velocity_vectors[i] * weight[i]

        if combine_pulsive:
            pulsive_velocity = np.zeros(2)
            for i in range(len(nearest_points)):
                pulsive_velocity += combine_pulsive_vectors[i] * weight[i]
            
            pulsive_velocity = pulsive_velocity / np.linalg.norm(pulsive_velocity)

            # if the angle difference between pulsive_velocity and weighted_velocity 
            # is less than 90 degree, the pulsive velocity is added
            angle_diff = np.arccos(np.dot(pulsive_velocity, weighted_velocity) / (np.linalg.norm(pulsive_velocity) * np.linalg.norm(weighted_velocity)))
            print(angle_diff)
            if abs(angle_diff) < 3 * np.pi / 2:
                print('min_distance', min_distance, max((-(2.0)*min_distance+1.5), 0))
                weighted_velocity = weighted_velocity + pulsive_velocity*(max((-(2.0)*min_distance+1.5), 0))

        angle_diff = np.arccos(np.dot(current_velocity, direction) / (np.linalg.norm(current_velocity) * np.linalg.norm(direction)))

        if angle_diff < np.pi / 3 and angle_diff > -np.pi / 3:
            new_velocity = weighted_velocity / np.linalg.norm(weighted_velocity)

    return new_velocity

def limit_velocity_around_attractor(velocity, position, attractor_position, 
                                    distance_decrease=1.0, maximum_velocity=1.0):
    dist_attr = LA.norm(position - attractor_position)

    if not dist_attr:
        return np.zeros(velocity.shape)

    mag_vel = LA.norm(velocity)
    if not mag_vel:
        return velocity

    if dist_attr > distance_decrease:
        desired_velocity = maximum_velocity
    else:
        desired_velocity = maximum_velocity * (
            dist_attr / distance_decrease
        )
    return velocity / mag_vel * desired_velocity

def evaluate_initial_velocity(current_position, local_goal_position, max_velocity=1.0):
    A_matrix = np.array([[1, 0], [0, 1]])
    velocity = A_matrix.dot(local_goal_position - current_position)

    if max_velocity is not None:
        velocity = limit_velocity_around_attractor(velocity, current_position, local_goal_position, maximum_velocity=max_velocity)

    return velocity

class PID:
    def __init__(self, kp_v=0.5, kd_v=0.05, kp_w=2.0, kd_w=0.05) -> None:
        self.kp_v = kp_v
        self.kd_v = kd_v

        self.kp_w = kp_w
        self.kd_w = kd_w

        self.v_ref = 1.0
        self.w_ref = 0

        self.prev_v_error = 0
        self.prev_w_error = 0

    def set_v_ref(self, v_ref):
        self.v_ref = v_ref

    def cal_ref(self, current_state, target_state):
        # current_state: x, y, theta, v, w
        # target_state: x, y, theta
        theta_error = target_state[2] - current_state[2]
        if theta_error > np.pi:
            theta_error -= 2 * np.pi
        if theta_error < -np.pi:
            theta_error += 2 * np.pi
        
        # if theta_error is large, the ref_v should decrease
        ref_v = self.v_ref * (0.5/np.abs(theta_error))
        ref_v = np.min([ref_v, self.v_ref])
        ref_w = theta_error

        return ref_v, ref_w

    def update(self, current_state, target_state):
        # current_state: x, y, theta, v, w
        # target_state: x, y, theta
        ref_v, ref_w = self.cal_ref(current_state, target_state)
        v_error = ref_v - current_state[3]
        w_error = ref_w

        p_term_v = self.kp_v * v_error
        d_term_v = self.kd_v * (v_error - self.prev_v_error)

        p_term_w = self.kp_w * w_error
        d_term_w = self.kd_w * (w_error - self.prev_w_error)

        self.prev_v_error = v_error
        self.prev_w_error = w_error

        return p_term_v + d_term_v, p_term_w + d_term_w



if __name__ == '__main__':
    rospy.init_node('test_star_ros', anonymous=True)

    #### assign
    start_position = np.array([-3.2, -0.4])
    goal_position = np.array([4.1, 0.0])

    # WORLD 1 
    # START REGION left -3.6 right -2.7 up 3.2 down -1.35
    # END REGION left 4.5 right 4.8 up 3.2 down -1.35
    # DISPLAY PARAMETER
    # scale 210, angle 0, x 0.6, y 0.8
    # WORLD 2 
    # START REGION left -3.53 right -1.76 up 1.0 down 0.372
    # END REGION 1 left 3.9 right 4.78 up 2.0 down -1.37
    # END REGION 2 left 1.13 right 4.78 up -0.9 down -1.37

    odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
    laser_sub = rospy.Subscriber('/scan', LaserScan, laser_callback)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    arrow_pub = rospy.Publisher('/arrow_marker', Marker, queue_size=10)
    polygon_pub = rospy.Publisher('/polygon', PolygonStamped, queue_size=10)
    polygon_array_pub = rospy.Publisher('/polygon_array', PolygonArray, queue_size=10)
    subgoal_pub = rospy.Publisher('/subgoal', Marker, queue_size=10)
    traj_pub = rospy.Publisher('/trajectory', Path, queue_size=10)
    frontier_pub = rospy.Publisher('/frontier', MarkerArray, queue_size=10)
    arrow_array_pub = rospy.Publisher('/arrow_array', MarkerArray, queue_size=10)
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    roadmap_pub = rospy.Publisher('/roadmap', MarkerArray, queue_size=10)
    initial_velocity_pub = rospy.Publisher('/initial_velocity', Marker, queue_size=10)
    modulated_velocity_pub = rospy.Publisher('/modulated_velocity', Marker, queue_size=10)
    goal_position_pub = rospy.Publisher('/goal_position', Marker, queue_size=10)

    pid_controller = PID()

    # put the robot at the start position in gazebo
    publish_cmd_vel(cmd_pub, [0, 0])
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    robot_model_state = SetModelStateRequest()
    robot_model_state.model_state.model_name = 'turtlebot3'
    robot_model_state.model_state.pose.position.x = start_position[0]
    robot_model_state.model_state.pose.position.y = start_position[1]
    robot_model_state.model_state.pose.position.z = 0.0
    robot_model_state.model_state.pose.orientation.x = 0.0
    robot_model_state.model_state.pose.orientation.y = 0.0
    robot_model_state.model_state.pose.orientation.z = 0.0
    robot_model_state.model_state.pose.orientation.w = 1.0
    robot_model_state.model_state.twist.linear.x = 0.0
    robot_model_state.model_state.twist.linear.y = 0.0
    robot_model_state.model_state.twist.linear.z = 0.0
    robot_model_state.model_state.twist.angular.x = 0.0
    robot_model_state.model_state.twist.angular.y = 0.0
    robot_model_state.model_state.twist.angular.z = 0.0
    set_result = set_model_state(robot_model_state)
    print('set_result', set_result)

    # clear_road_map(roadmap_pub)

    while not is_laser_init:
        print('waiting for laser init...', len(laser_points), is_laser_init)
        rospy.Rate(10).sleep()
    laser_points_copy = copy(laser_points)

    new_position = copy(robot_c)
    last_position = copy(robot_c)

    
    temp_points = transform_to_global_frame(laser_points_copy, robot_c, robot_yaw)
    temp_filtered_points = transform_to_global_frame(laser_points_filter_max_range, robot_c, robot_yaw)

    trajectory_list = []
    velocity_list = []
    search_time_list = []
    compute_time_list = []
    
    update_hz = 40
    rate = rospy.Rate(update_hz)


    fast_avoid = SampledAvoider(control_radius=0.20, evaluate_normal=False)
    fast_avoid.update_laserscan(temp_filtered_points.T, in_robot_frame=False)
    initial_dynamics = LinearSystem(
            attractor_position=new_position, maximum_velocity=0.2
        )

    star_node = StarshapedRep(temp_points, start_position, robot_theta=robot_yaw, 
                              robot_radius=0.0, ros_simu=True, 
                              points_for_frontier=temp_filtered_points)
    # star_node.draw('test1', save=True)
    graph_manager = GraphManager(star_node)
    start_star_id = copy(graph_manager.start_id)
    current_star_id = copy(graph_manager.start_id)

    path, reach = graph_manager.find_path(current_star_id, goal_position)
    local_id = path['path_id'][-1]
    local_position = path['path'][-1]

    reach_local = False
    reach_global = False

    iter = 0
    interval = 50

    safety_margin = 0.25
    local_position_limit = copy(safety_margin)
    move_start_time = time.time()
    try:
        while not reach_global and iter < 100000:
            current_position = copy(robot_c)
            current_yaw = copy(robot_yaw)
            current_v = copy(robot_v)
            current_w = copy(robot_w)

            search_time = None

            iter += 1
            if len(laser_points) == 0 or len(laser_points_filter_max_range) == 0:
                Warning('no laser points')
            else:
                temp_points = copy(laser_points)
                temp_filtered_points = copy(laser_points_filter_max_range)
        
            # apped current position to trajectory
            trajectory_list.append(current_position)
            publish_trajectory(traj_pub, trajectory_list)

            initial_velocity = evaluate_initial_velocity(current_position, local_position)
            publish_arrow(initial_velocity_pub, current_position, np.arctan2(initial_velocity[1], initial_velocity[0]), 1, [1, 0, 0])
            
            detection_points = transform_to_global_frame(temp_points, current_position, current_yaw)
            temp_filtered_points = transform_to_global_frame(temp_filtered_points, current_position, current_yaw)
            
            start_time = time.time()
           
            initial_dynamics = LinearSystem(
                local_position, maximum_velocity=0.2
            )
            fast_avoid.update_laserscan(temp_filtered_points.T, in_robot_frame=False)
            new_velocity = fast_avoid.avoid(
                initial_velocity,  position=current_position
            )
            end_time = time.time()
            compute_time_list.append(end_time - start_time)

            new_velocity = new_velocity / np.linalg.norm(new_velocity)
            new_velocity = new_velocity / update_hz
            last_position = copy(new_position)
            new_position = current_position + new_velocity
            
            target_yaw = np.arctan2(new_velocity[1], new_velocity[0])
            publish_arrow(modulated_velocity_pub, current_position, target_yaw, 2, [0, 1, 0])
            # print('position', current_position, new_position, local_position)

            control_signal = pid_controller.update(
                current_state=[robot_c[0], robot_c[1], robot_yaw, robot_v, robot_w],
                target_state=[new_position[0], new_position[1], target_yaw]
            )
            velocity_list.append(control_signal)

            # judge if reach the local position
            if np.linalg.norm(local_position - current_position) < local_position_limit and \
                np.linalg.norm(local_position - goal_position) > 0.001:
                print('reach the local position')
                reach_local = True

            # judge if reach the global position
            if np.linalg.norm(goal_position - current_position) < 0.1:
                print('reach the global position')
                reach_global = True
                reach_local = False
                break
            
            # if reach local, generate a new star node and find a new path
            if reach_local:
                print('test', local_position, new_position)
                # generate a new star node

                find_obstacle = False
                in_other_starshape = False
                # judge if there are points around the local goal
                for i in range(len(detection_points)):
                    if np.linalg.norm(detection_points[i] - local_position) < 0.05:
                        print('remove the obstacle', np.linalg.norm(detection_points[i] - local_position))
                        find_obstacle = True
                        graph_manager.remove_node(local_id)
                        current_star_id = graph_manager.get_current_star_id(current_position)
                        search_start_time = time.time()
                        path, reach = graph_manager.find_path(current_star_id, goal_position)
                        search_end_time = time.time()
                        search_time = search_end_time - search_start_time

                        print('path', path)
                        if len(path['path']) > 2:
                            # judge if the second point and third point are in the same starshape, choose the third one
                            local_id = path['path_id'][1]
                            local_position = path['path'][1]
                        else:
                            local_id = path['path_id'][-1]
                            local_position = path['path'][-1]

                        break

                # judge if the id have a starshaped representation
                print(graph_manager._nodes[local_id].get_star_rep())
                if not find_obstacle:
                    if graph_manager._nodes[local_id].get_star_rep() is None:
                        star_rep = StarshapedRep(detection_points, current_position, robot_theta=robot_yaw, robot_radius=0.0,
                                                ros_simu=True, points_for_frontier=temp_filtered_points)
                        graph_manager.extend_star_node(local_id, star_rep, new_position)

                    current_star_id = copy(local_id)
                    star_rep.draw('test1', save=False)
                    search_start_time = time.time()
                    path, reach = graph_manager.find_path(current_star_id, goal_position)
                    search_end_time = time.time()
                    search_time = search_end_time - search_start_time

                    if len(path['path']) > 2:
                        local_id = path['path_id'][1]
                        local_position = path['path'][1]
                    else:
                        local_id = path['path_id'][-1]
                        local_position = path['path'][-1]

                reach_local = False

            # get the points of the all star nodes and pulish polygon
            star_id_list = graph_manager._star_id_list
            star_points = []
            for i in range(len(star_id_list)):
                star_points.append(graph_manager._nodes[star_id_list[i]].get_star_rep().get_points())

            if search_time is not None:
                search_time_list.append(search_time)
            else:
                search_time_list.append(0.0)

            publish_polygon_array(polygon_array_pub, star_points)
            publish_path(path_pub, path['path'])
            publish_subgoal(subgoal_pub, local_position)
            publish_cmd_vel(cmd_pub, control_signal)
            publish_goal_position(goal_position_pub, goal_position)

            # display the roadmap
            display_road_map(roadmap_pub, graph_manager)

            rate.sleep()
    except KeyboardInterrupt:
        print('keyboard interrupt')

    move_end_time = time.time()
    publish_cmd_vel(cmd_pub, [0, 0])
    while(len(search_time_list) < len(trajectory_list)):
        search_time_list.append(0.0)

    datas = []
    for i in range(len(trajectory_list)):
        # add trajectory, velocity, compute time, search time
        datas.append(np.array([trajectory_list[i][0], trajectory_list[i][1], velocity_list[i][0], velocity_list[i][1], compute_time_list[i], search_time_list[i]]))

    time_cost = move_end_time - move_start_time
    datas.append(np.array([time_cost, 0, 0, 0, 0, 0]))

    if not reach_global:
        datas.append(np.array([0, 0, 0, 0, 0, 0]))
    else:
        datas.append(np.array([1, 0, 0, 0, 0, 0]))
    