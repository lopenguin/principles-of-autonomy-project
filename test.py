from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import tools.helpers as helpers
import time
sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_info, set_joint_position

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

def main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(0.5, 0.65, np.pi / 4))
    
    sugar_box_pose = get_pose(world.get_body(sugar_box))
    
    set_pose(sugar_box,(Point(0.71, 1.10, -0.65), Euler(np.pi/2,0.,-np.pi/2)))
    print('sugar box pose',sugar_box_pose)
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    # wait_for_user()
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    sample_fn = get_sample_fn(world.robot, world.arm_joints)

    print("Going to operate the base without collision checking")
    goal_pos = translate_linearly(world, 0.9) # does not do any collision checking!!
    goal_pos[1] += 0.7
    set_joint_positions(world.robot, world.base_joints, goal_pos)
    
    drawer_joint_info = get_joint_info(world.kitchen,56)
    drawer_lower_lim = drawer_joint_info.jointLowerLimit
    #drawer_upper_lim = drawer_joint_info.jointUpperLimit
    drawer_upper_lim = 0.30

    drawer_poses = np.linspace(drawer_lower_lim,drawer_upper_lim,len(helpers.OPENING_TRAJ))


    for joint_angle,drawer_pose in zip(helpers.OPENING_TRAJ,drawer_poses):
        tool_link = link_from_name(world.robot, 'panda_hand')
        robot_position = get_link_pose(world.robot,tool_link)

        set_joint_positions(world.robot, world.arm_joints, joint_angle)
        set_joint_position(world.kitchen,56,drawer_pose)
        time.sleep(0.05)
    
    
    print("Going to use IK to go from a sample start state to a goal state\n")
    for i in range(10):
        #print('Iteration:', i)
        #conf = sample_fn()
        #set_joint_positions(world.robot, world.arm_joints, conf)
        #startt = [-0.9993014606753718, 0.5575981344031993, 0.08848226914383991, -1.9027443759647094, -0.07403383822822018, 2.4572408795101772, -0.08136335471573197]
        #startt = [1.1179275530880908, -1.7247582114792013, -1.905165080776905, -1.7852457187137816, 0.7740623064191361, 2.8037386402624724, 2.896613453226052]
        startt = [0.6108557398957775, -1.2408811105330781, -2.2342699407330207, -2.085964847766605, 0.8128326480682402, 2.2560068429844358, 2.8898273321656816]
        set_joint_positions(world.robot, world.arm_joints, startt)
        ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
        start_pose = get_link_pose(world.robot, tool_link)
        print('start_pose', start_pose)
        end_pose = Pose(Point(0.71, 1.10, -0.65), Euler(np.pi/2,0.,-np.pi/2))
        #end_pose = Pose(Point(0.75, 2.0 -1.50), Euler(np.pi/2,0.,-np.pi/2))
        print('end_pose', end_pose)
        for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            #print(conf)
            if conf is None:
                print('Failure!')
                # wait_for_user()
                break
            set_joint_positions(world.robot, ik_joints, conf)
        actual_end_pose = get_link_pose(world.robot, tool_link)
        print('actual_end_pose', actual_end_pose)
        wait_for_user()
    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
