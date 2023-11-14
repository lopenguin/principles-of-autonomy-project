#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  5 12:57:46 2023

@author: astrodec32
"""

from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import random
import time

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly
    
UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs) 

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi #ensures all joint angles are between [-pi pi]

def joint_space_distance(joint_angles_a, joint_angles_b):
    joint_diffs = [normalize_angle(a - b) for a, b in zip(joint_angles_a, joint_angles_b)] #subtracts joint angles after normalizing to find distance between poses
    squared_diffs = [diff**2 for diff in joint_diffs] # squared difference beetween them
    return np.sqrt(sum(squared_diffs)) #total value of squared differences (i.e.  sum of squares for error)

def find_x_nearest(Vertices,x_rand):
    for j in range(len(Vertices)):
        if j == 0:
            dist_min = joint_space_distance(Vertices[j],x_rand)
            node_min = Vertices[j]
        else:
            dist = joint_space_distance(Vertices[j],x_rand)
            if dist < dist_min:
                dist_min = dist
                node_min = Vertices[j]
                
    return node_min, dist_min

def find_x_new(x_nearest,x_rand,max_step,lower_limits,upper_limits):
    dist = joint_space_distance(x_nearest,x_rand)
    if dist <= max_step: # no need for unit vector math if dist is already within tolerance
        return x_rand
      
    unit_vector = [(x_rand_i - x_nearest_i) / dist for x_rand_i, x_nearest_i in zip(x_rand, x_nearest)]
    x_new = [x_nearest_i + max_step * unit_i for x_nearest_i, unit_i in zip(x_nearest, unit_vector)]
    
    x_new_bounded = []
    for i, angle in enumerate(x_new):
        temp_angle = min(angle,upper_limits[i])
        bounded_angle = max(temp_angle,lower_limits[i])
        x_new_bounded.append(bounded_angle)
        
    return x_new_bounded

def is_node_within_goal_region(node, goal_region):
    return all(lower <= angle <= upper for angle, (lower, upper) in zip(node, goal_region))

def find_rrt_path(start_pose,x_new,Edges):
    rrt_path = [x_new]
    current_node = x_new
    while current_node != start_pose:
        current_node = Edges[current_node]
        rrt_path.append(current_node)
    return rrt_path

def simulate_path(world, final_path):
    for joint_angle in final_path:
        set_joint_positions(world.robot, world.arm_joints, joint_angle)
        time.sleep(0.05)
        

def robot_rrt(world, start_joint_angles, goal_region):
    world._update_initial()
    lower_limits, upper_limits = get_custom_limits(world.robot, world.arm_joints) #gets joint limits
    max_step = 5*np.pi/180
    N = 10000
    start_joint_angles_tuple = tuple(start_joint_angles)
    #print("start_joint_angles_tuple", start_joint_angles_tuple)
    Vertices = [start_joint_angles_tuple] #create list of vertices
    Edges = {} # create dict mapping vertices to other vertices (i.e. edges)
    Edges[start_joint_angles_tuple] = [] # first set in dict is the start_pose, no edges yet
    
    for i in range(N):
        print(f"\rCurrent value of i: {i}", end="")
        #Probably should add goal biasing here as well
        Goal_Bias = 0.2
        
        if random.random() < Goal_Bias:
            x_rand = goal_joint_angles
            #print(x_rand)
            #wait_for_user()
        else:
            sample_fn = get_sample_fn(world.robot, world.arm_joints) #generates random arm joints
            x_rand = tuple(sample_fn())
        #print("x_rand_pose", x_rand)
        #x_rand = tuple(next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, x_rand_pose, max_candidates=100, norm=2, max_attempts=1000, max_time=10.0), None))
        x_nearest, dist_min = find_x_nearest(Vertices,x_rand) # Find x_nearest existing node
        x_new = tuple(find_x_new(x_nearest,x_rand,max_step,lower_limits,upper_limits)) # Find x_new in the same direction as x_rand (needs to be smaller for the IK to solve)
        #print("X_new:", x_new)
        #add collision checking later
        #print("x_nearest", x_nearest)
        #print("x_new", x_new)
        Vertices.append(x_new) #add x_new to set of nodes
        Edges[x_new] = x_nearest #add x_new x_nearest edge to dict
        
        if is_node_within_goal_region(x_new,goal_region):
            print("start angles:",start_joint_angles)
            print("goal angles:",goal_joint_angles)
            print("x_new:",x_new)
            #wait_for_user()
            rrt_path = find_rrt_path(start_joint_angles,x_new,Edges)
            final_path = rrt_path[::-1]
            print('Path Found!')
            return final_path
    print("Vertices Length",len(Edges))
    print('No Path Found') # if this happens increase N, maybe add goal biasing
            
def main(start_joint_angles, goal_region):
    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True) 
    grasp_height = 0.1 # right above the sugar
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    sugar_box_pose = get_pose(world.get_body(sugar_box))
    sugar_box_position, sugar_box_orientation = sugar_box_pose
    sugar_box_angles = ((sugar_box_position[0], sugar_box_position[1], sugar_box_position[2] + grasp_height), (Euler(roll=0, pitch=np.pi/2, yaw=0)))
    print("sugar box angles",sugar_box_angles)
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    for i in range(100):
        goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(0.1)
        
    final_path = robot_rrt(world, start_joint_angles, goal_region)
    wait_for_user()
    if final_path:
        simulate_path(world, final_path)
    else:
        print("not path found")
    
        
        
# Configures numpy to display floating-point numbers with a certain formatting.
# This makes the output more readable and concise.
#np.set_printoptions(precision=3, suppress=True)

# Instantiates a 'World' object, which likely encapsulates the simulation environment.
# The `use_gui=True` argument indicates that a graphical user interface should be shown.
#world = World(use_gui=True)      
# These lines add objects (sugar_box and spam_box) to the simulation at specified
# positions and orientations on counters in the simulated world.
#sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
#spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
#world._update_initial()
#for i in range(100):
    #goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
    #set_joint_positions(world.robot, world.base_joints, goal_pos)
    #if (i % 30 == 0):
#sugar_box_pose = get_pose(world.get_body(sugar_box))
#sugar_box_position, sugar_box_orientation = sugar_box_pose
#grasp_height = 0.1 # right above the sugar
#tool_link = link_from_name(world.robot, 'panda_hand') # Retrieves the ID of the 'tool_link' (end effector) of the robot in the simulation.
#joints = get_movable_joints(world.robot) #Retrieves the list of movable joints of the robot.
#goal_pose = ((sugar_box_position[0], sugar_box_position[1], sugar_box_position[2] + grasp_height), (Euler(roll=0, pitch=np.pi/2, yaw=0)))
#goal_pose = sugar_box_pose
#def closest_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, max_candidates=INF, norm=INF, **kwargs):
#start_pose = get_link_pose(world.robot, tool_link)

#print(goal_joint_angles)
#goal_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, target_pose=goal_pose, max_time=10.00), None)
#start_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, start_pose, max_candidates=10, norm=2, max_attempts=100, max_time=5.0), None)

#goal_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, goal_pose, max_candidates=100, norm=2, max_attempts=1000, max_time=10.0), None)
#angle_offsets = [0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25]
#goal_joint_angles = [angle + offset for angle, offset in zip(start_joint_angles, angle_offsets)]
#sample_goal = get_sample_fn(world.robot, world.arm_joints)
#goal_joint_angles = tuple(sample_goal())
goal_joint_angles = (2.838311267173578, 1.7244685769643007, -2.84226720544487, -1.7908654445975372, 1.7530127023674704, 1.1025644779622914, 0.17423056193600317)
#print("goal:", goal_joint_angles)
#start_joint_angles = tuple(sample_goal())
start_joint_angles = (-2.333400804952562, 0.7383606900280834, 1.1742612657904137, -1.2249193382261694, -1.7583702601412796, 3.22570026313478, -1.0923235354053924)
#print("Modified goal_joint_angles:", goal_joint_angles)
#print("starting pose:", start_pose)
#print(goal_pose)
#print("start joint angles:", start_joint_angles)
#print("goal joint angles", goal_joint_angles)
#start_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, start_pose, max_candidates=INF, norm=INF), None)
#goal_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, goal_pose, max_candidates=INF, norm=INF), None)
tolerance_radians = 5*(np.pi/180) # tolerance for goal_pose to make a feasible goal region
goal_region = [(angle - tolerance_radians, angle + tolerance_radians) for angle in goal_joint_angles]
#print("goal_region:", goal_region)
#need another line so that the goal_region is within the joint limits
start_in_goal = all(lower <= angle <= upper for angle, (lower, upper) in zip(start_joint_angles, goal_region))
#print("Is the start configuration within the goal region?", start_in_goal)
#sample_fn = get_sample_fn(world.robot, world.arm_joints)
#print("Sample_fn", sample_fn)
#print("joint space distance:", joint_space_distance(start_joint_angles,goal_joint_angles))
#sample_fn = get_sample_fn(world.robot, world.arm_joints)
#ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
#final_path = robot_rrt(world, start_joint_angles, goal_region)
#print("goal joint angles", goal_joint_angles)
#print("path:",final_path)
main(start_joint_angles, goal_region)



