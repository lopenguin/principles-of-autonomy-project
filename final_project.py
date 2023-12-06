'''
The most complete script for the PADM Final Project

Integrates PDDL, motion planning, RRT, and trajectory optimization
'''
import os
import sys
import numpy as np
import random
import time

from pydrake.solvers import MathematicalProgram, Solve

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

OPTIMIZE_NUM_PTS = 50


class KitchenRobot:
    def __init__(self):
        # Create pybullet world
        world = World(use_gui=True)
        #sugar_box = helpers.add_sugar_box(world, idx=0, counter=1, pose2d=(0.1, 0.65, np.pi / 4))
        sugar_box = helpers.add_sugar_box(world, idx=0, counter=1, pose2d=(0.13, 0.65, np.pi / 4))

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

        start = get_joint_positions(world.robot, world.arm_joints)
        for angle in check_angles:
            set_joint_positions(world.robot, world.arm_joints, angle)
            is_collision = pairwise_collision(world.robot,world.kitchen)
            if is_collision:
                print(f"COLLISION FOUND: {is_collision}")
                return True
        set_joint_positions(world.robot, world.arm_joints, start)
        return False
    
    
    # optimization-based traj opt
    def optimize_trajectory(self, start, goal, num_pts):
        if not USE_JOINT_SPACE:
            raise NotImplementedError
        
        # convert to numpy
        start = np.array(start)
        goal = np.array(goal)
        num_jts = len(start)

        # Create optimization problem
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(num_jts, num_pts, "theta")

        ## Constraints
        # start and goal
        prog.AddLinearEqualityConstraint(x[:, 1], start)
        prog.AddLinearEqualityConstraint(x[:,-1], goal)
        # joint limits
        for i in range(num_jts):
            # apply for one joint along all paths
            prog.AddBoundingBoxConstraint(self.lower_limits[i], self.upper_limits[i], x[i,:])
        
        ## Objective: shortest path
        for i in range(1,num_pts):
            for j in range(num_jts):
                prog.AddCost(np.transpose(x[j,i] - x[j,i-1]) * (x[j,i] - x[j,i-1]))

        ## Solve!
        result = Solve(prog)
        
        if (not result.is_success()):
            return None
        
        path_np = result.GetSolution(x)

        # convert to list of tuples
        path = []
        for i in range(num_pts):
            pt = np.array(path_np[:,i].T)
            path.append(tuple(pt))

        return path

    # optimize trajectory with collision avoidance
    def optimize_trajectory_with_collision_avoidance(self,start,goal, num_pts):
        if USE_JOINT_SPACE:
            raise NotImplementedError
        
        # convert to numpy
        start = np.array(start)
        goal = np.array(goal)
        num_jts = len(start)

        # Create optimization problem
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(num_jts, num_pts, "theta")

        ## Constraints
        # start and goal
        prog.AddLinearEqualityConstraint(x[:, 1], start)
        prog.AddLinearEqualityConstraint(x[:,-1], goal)
        # joint limits
        for i in range(num_jts):
            # apply for one joint along all paths
            prog.AddBoundingBoxConstraint(self.lower_limits[i], self.upper_limits[i], x[i,:])
        
        ## Objective: shortest path
        for i in range(1,num_pts):
            for j in range(num_jts):
                prog.AddCost(np.transpose(x[j,i] - x[j,i-1]) * (x[j,i] - x[j,i-1]))

        ## Solve!
        result = Solve(prog)
        # use to set initial guess with collisions
        prog.SetInitialGuess(prog.ReconstructTrajectory(result))


        ## Repeat with collisions
        for i in range(num_jts):
            for c in helpers.COLLISIONS_LIST:
                sign = c['sign'][i]
                val = c['val'][i]
                prog.AddConstraint(sign*x[i,:] < val)
        
       


    
    def simulate_path(self, act, path):
        world = self.world

        if not USE_JOINT_SPACE:
            raise NotImplementedError

        for point in path:
            # move the robot
            set_joint_positions(world.robot, world.arm_joints, point)
            is_collision = pairwise_collision(world.robot,world.kitchen)
            if is_collision:
                print(f"COLLISION FOUND: {is_collision}")

            # simulate grabbing
            if (act.name == 'placein') or \
               (act.name == 'placeon'):
                # move spam or sugar
                robot_position = get_link_pose(world.robot,self.tool_link)
                if 'spam' in act.parameters:
                    body = world.get_body(self.spam_box)
                    z_offset = 0.15  # 5 cm below the gripper
                    
                    # New robot position (adjusting Z-coordinate)
                    new_position = (
                        robot_position[0][0],  # X-coordinate, unchanged
                        robot_position[0][1],  # Y-coordinate, unchanged
                        robot_position[0][2] - z_offset  # Z-coordinate, offset downwards
                    )
                    
                    # New orientation quaternion
                    new_orientation = (0.0, 0.0, 0.38268343236508984, 0.9238795325112867)
                    
                    # New robot position and orientation
                    robot_position = (new_position, new_orientation)


                    #print(robot_position)
                else:
                    body = world.get_body(self.sugar_box)
                    
                    z_offset = 0.2  # 5 cm below the gripper
                    
                    # New robot position (adjusting Z-coordinate)
                    new_position = (
                        robot_position[0][0],  # X-coordinate, unchanged
                        robot_position[0][1],  # Y-coordinate, unchanged
                        robot_position[0][2] - z_offset # Z-coordinate, offset downwards
                    )
                    
                    # New orientation quaternion
                    new_orientation = (0.0, 0.0, 0.38268343236508984, 0.9238795325112867)
                    
                    # New robot position and orientation
                    robot_position = (new_position, new_orientation)
                    #)
                
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
                is_collision = pairwise_collision(world.robot,world.kitchen)
                if is_collision:
                    print(f"COLLISION FOUND: {is_collision}")
                set_joint_position(world.kitchen,56,drawer_pose)
                time.sleep(0.05)
            '''
            for joint_angle in helpers.VERT_TRAJ:
                set_joint_positions(world.robot, world.arm_joints, joint_angle)
                is_collision = pairwise_collision(world.robot,world.kitchen)
                if is_collision:
                    print(f"COLLISION FOUND: {is_collision}")
                time.sleep(0.05)
            
            for joint_angle in helpers.TWIST_TRAJ:
                set_joint_positions(world.robot, world.arm_joints, joint_angle)
                is_collision = pairwise_collision(world.robot,world.kitchen)
                if is_collision:
                    print(f"COLLISION FOUND: {is_collision}")
                time.sleep(0.05)
            '''
    def vert_twist(self, path_vert):
        
        world = self.world
    
        if not USE_JOINT_SPACE:
            raise NotImplementedError
            
        for point in path_vert:
            # move the robot
            set_joint_positions(world.robot, world.arm_joints, point)
            is_collision = pairwise_collision(world.robot,world.kitchen)
            if is_collision:
                print(f"COLLISION FOUND: {is_collision}")
    
            time.sleep(0.05)
    



def main():
    # helpful prints

    # initialize the world
    kr = KitchenRobot()

    # run activity planning
    status = kr.generate_plan()
    if (not status):
        print("Failed to find a plan... Please try again.")
        return
    
    # Prepare to move
    print("Plan found! Press enter to execute.")

    # Follow the plan
    if rrt:
        print("Showing RRT-planned trajectory")
        check = 0
    else:
        print("Showing optimization-based trajectory")
        check = 1
    wait_for_user("Press enter to begin")

    for i, act in enumerate(kr.plan):
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

        # Trajectory generation
        if (rrt):
            # RRT
            path = kr.rrt_on_action(start, goal, goal_region)
        else:
            # optimization
            path = kr.optimize_trajectory(start,goal, OPTIMIZE_NUM_PTS)
            
            if act.name == 'open':
                start_vert = [0.6108557398957775, -1.2408811105330781, -2.2342699407330207, -2.085964847766605, 0.8128326480682402, 2.2560068429844358, 2.8898273321656816]
                goal_vert = [0.19804023471086296, -1.4349267532033405, -1.2478317792941391, -2.2974082827545645, 0.11014769033115979, 2.5257883467675386, -2.1023655766474945]
                path_vert = kr.optimize_trajectory(start_vert,goal_vert, OPTIMIZE_NUM_PTS)
            
            


        if path is not None:
            print('Path found.')
        else:
            print("No path found! Try running again?")
            wait_for_user()

        # visualize the path
        time.sleep(1.0)
        kr.simulate_path(act, path)
        if check ==1:
            if act.name == 'open':
                kr.vert_twist(path_vert)
        wait_for_user()

    # pause at very end
    print("Problem complete.")
    wait_for_user("Press enter to close")



if __name__ == '__main__':
    # parse input arguments
    if len(sys.argv) == 1:
        # Pick randomly between RRT and colision mode
        if (random.random() > 0.5):
            rrt = False
        else:
            rrt = True
    else:
        if (sys.argv[1] == "rrt"):
            rrt = True
        elif (sys.argv[1] == "optimize"):
            rrt = False
        else:
            print("Use \"rrt\" or \"optimize\" keywords to pick. Defaulting to RRT...)")
            rrt = True

    if (rrt):
        print("-"*20)
        print("Running in RRT mode.")
        print("-"*20)
    else:
        print("-"*29)
        print("Running in optimization mode.")
        print("-"*29)

    main()