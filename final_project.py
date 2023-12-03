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
sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['ss-pybullet'])
from pybullet_tools.utils import set_pose, pairwise_collision, wait_for_user, link_from_name, pairwise_collision
from pybullet_tools.utils import get_custom_limits, set_joint_positions, set_joint_position, \
                                 get_joint_positions, get_link_pose, get_joint_info, enable_gravity

from src.world import World
from src.utils import translate_linearly

# For PDDL/Activity Planning
from pddl.activity_planning import ActivityPlan
from pddl.kitchen_map import get_goal, KMAP_JOINT, KMAP_TASK

# For RRT and world setup
import tools.helpers as helpers

# Constants
DOMAIN_FILE = 'pddl/kitchen.pddl'
PROBLEM_FILE = 'pddl/problem.pddl'
USE_JOINT_SPACE = True

RRT_MAX_ITER = 10000
RRT_GOAL_BIAS = 0.2
RRT_MAX_STEP = 5 * (np.pi/180) # 5 deg
RRT_COLLIDES_STEP = 0.01 # ~0.5 deg


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

        # RRT functions
        self.lower_limits, self.upper_limits = get_custom_limits(world.robot, world.arm_joints)

        # save
        self.world = world
        self.sugar_box = sugar_box
        self.spam_box = spam_box
        self.tool_link = link_from_name(world.robot, 'panda_hand')
        self.jt_samplefun = helpers.get_sample_fn(world.robot, world.arm_joints)

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
    def rrt_on_action(self, start, goal, goal_region):
        if not USE_JOINT_SPACE:
            raise NotImplementedError

        world = self.world
        
        nodes = [tuple(start)]
        edges = {}
        edges[nodes[0]] = []

        # run until convergence
        for i in range(RRT_MAX_ITER):
            # goal biasing
            if random.random() < RRT_GOAL_BIAS:
                x_rand = goal
            else:
                if USE_JOINT_SPACE:
                    x_rand = tuple(self.jt_samplefun())

            # get the node nearest to this sample
            x_nearest, _ = helpers.find_x_nearest(nodes, x_rand)
            x_new = tuple(helpers.find_x_new(x_nearest,x_rand, RRT_MAX_STEP, \
                                             self.lower_limits, self.upper_limits))
            
            # Collision checking
            if (self.collides(x_nearest, x_new)):
                continue

            # add node
            nodes.append(x_new)
            edges[x_new] = x_nearest
            
            # termination
            if helpers.is_node_within_goal_region(x_new, goal_region):
                path = helpers.find_rrt_path(start, x_new, edges)
                path = path[::-1]
                return path
            
        return None
    
    def collides(self, x0, x1):
        if not USE_JOINT_SPACE:
            raise NotImplementedError
        
        # measure every 0.5 deg (0.01 rad)
        dist = np.linalg.norm(np.array(x0) - np.array(x1))
        n = max(1,int(dist / (RRT_COLLIDES_STEP*len(x0))))
        check_angles = np.linspace(x0,x1,n)

        world = self.world
        for angle in check_angles:
            set_joint_positions(world.robot, world.arm_joints, angle)
            is_collision = pairwise_collision(world.robot,world.kitchen)
            if is_collision:
                print(f"COLLISION FOUND: {is_collision}")
                return True
        return False
    

    
    def simulate_path(self, act, path):
        world = self.world

        if not USE_JOINT_SPACE:
            raise NotImplementedError

        for point in path:
            # move the robot
            set_joint_positions(world.robot, world.arm_joints, point)

            # simulate grabbing
            if (act.name == 'placein') or \
               (act.name == 'placeon'):
                # move spam or sugar
                robot_position = get_link_pose(world.robot,self.tool_link)
                if 'spam' in act.parameters:
                    body = world.get_body(self.spam_box)
                else:
                    body = world.get_body(self.sugar_box)
                
                set_pose(body,robot_position)
            time.sleep(0.05)

        # Simulate "opening" drawer
        if act.name == 'open':
            # Drawer info
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

    # Proceed through the plan
    for i, act in enumerate(kr.plan):
        wait_for_user()
        print(f"{i+1}. {act.name} {act.parameters}")

        # Motion Planning Setup
        if (USE_JOINT_SPACE):
            start = get_joint_positions(kr.world.robot, kr.world.arm_joints)
            goal = get_goal(act, KMAP_JOINT)
            tolerance_radians = 1*(np.pi/180) # tolerance for goal_pose to make a feasible goal region
            goal_region = [(angle - tolerance_radians, angle + tolerance_radians) for angle in goal]
        else:
            start = get_link_pose(kr.world.robot, kr.tool_link)
            goal = get_goal(act, KMAP_TASK)
        
        kr.world._update_initial() # not sure if we need this

        # RRT
        path = kr.rrt_on_action(start, goal, goal_region)
        if path is not None:
            print('RRT path found!')

        # Trajectory optimization
        # path = kr.optimal_traj(start,goal)

        # visualize the path
        kr.simulate_path(act, path)

    # pause at end!
    wait_for_user()

        




if __name__ == '__main__':
    main()