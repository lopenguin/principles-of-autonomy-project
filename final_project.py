'''
The most complete script for the PADM Final Project

Integrates PDDL, motion planning, RRT, and trajectory optimization
'''
import os
import sys
import numpy as np
import random
import time

# Add pybullet dependencies
sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'ss-pybullet')))
from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, single_collision, pairwise_collision
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, set_joint_position, get_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_info, enable_gravity, enable_real_time, disable_real_time
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

# For PDDL/Activity Planning
from pddl.activity_planning import ActivityPlan
from pddl.kitchen_map import get_goal, KMAP_JOINT, KMAP_TASK

# For RRT and world setup
import tools.helpers as helpers

# Constants
DOMAIN_FILE = 'pddl/kitchen.pddl'
PROBLEM_FILE = 'pddl/problem.pddl'
RRT_JOINT_SPACE = True


class KitchenRobot:
    def __init__(self):
        # Create pybullet world
        world = World(use_gui=True)
        sugar_box = helpers.add_sugar_box(world, idx=0, counter=1, pose2d=(0.1, 0.65, np.pi / 4))
        spam_box = helpers.add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
        enable_gravity()
        world._update_initial()

        # move the robot base into position
        goal_pos = translate_linearly(world, 1.2)
        goal_pos[1] += 0.7 # hacky way to move the robot sideways
        set_joint_positions(world.robot, world.base_joints, goal_pos)

        # save
        self.world = world
        self.sugar_box = sugar_box
        self.spam_box = spam_box
        self.tool_link = link_from_name(world.robot, 'panda_hand')


    ## Run activity planning (offline planning)
    def generate_plan(self):
        planner = ActivityPlan(DOMAIN_FILE, PROBLEM_FILE)
        plan, _ = planner.hill_climb_search()

        self.plan = plan
        if plan is not None:
            return True
        else:
            return False
        
    ## Run RRT
    def rrt_on_action(self, start, goal_region):
        world = self.world
        


        



def main():
    # initialize the world
    kr = KitchenRobot()

    # run activity planning
    status = kr.generate_plan()
    if (not status):
        print("Failed to find a plan... Please try again.")
        return
    
    # Prepare to move
    print("Plan found! Press enter to execute.")
    wait_for_user()

    # Proceed through the plan
    for i, act in enumerate(kr.plan):
        print(f"{i+1}. {act.name} {act.parameters}")

        # RRT!
        if (RRT_JOINT_SPACE):
            start = get_joint_positions(kr.world.robot, kr.world.arm_joints)
            goal = get_goal(act, KMAP_JOINT)
            tolerance_radians = 1*(np.pi/180) # tolerance for goal_pose to make a feasible goal region
            goal_region = [(angle - tolerance_radians, angle + tolerance_radians) for angle in goal]
        else:
            start = get_link_pose(kr.world.robot, kr.tool_link)
            goal = get_goal(act, KMAP_TASK)
        
        kr.world._update_initial() # not sure if we need this
        path = rrt_path = kr.rrt_on_action(start, goal_region)

        




if __name__ == '__main__':
    main()