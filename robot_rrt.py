
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

from pddl.activity_planning import ActivityPlan
from pddl.kitchen_map import get_goal, KMAP_JOINT


sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, single_collision, pairwise_collision
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, set_joint_position, get_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_info, enable_gravity, enable_real_time, disable_real_time

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
        
        tool_link = link_from_name(world.robot, 'panda_hand')
        robot_position = get_link_pose(world.robot,tool_link)
        #print("robot euler", robot_position)
        set_joint_positions(world.robot, world.arm_joints, joint_angle)
        if act.name=='placein':
            if 'spam' in act.parameters:
                set_pose(world.get_body(spam_box),robot_position)
            else:
                set_pose(world.get_body(sugar_box),robot_position)
        if act.name=='placeon':
            if 'spam' in act.parameters:
                set_pose(world.get_body(spam_box),robot_position)
            else:
                set_pose(world.get_body(sugar_box),robot_position)
        time.sleep(0.05)
        
    if act.name=='open':
        opening_traj = ([1.1178574403159747, -1.724930993309394, -1.9050806285604385, -1.7852890170289148, 0.7736997489053827, 2.8038560343365693, 2.896826686500735],
[1.1013069163417306, -1.6927338576258006, -1.9253286493895838, -1.8017585866233166, 0.8187442043740294, 2.7805411689345583, 2.8703762628140157],
[1.0825677148350366, -1.6675628053633924, -1.9418936427624798, -1.8194137801909198, 0.8465616635568587, 2.7612855455263183, 2.854467842488122],
[1.0476872809641424, -1.6798255005615588, -1.9395258381907408, -1.8451266623858578, 0.8003244941502503, 2.7685496829351193, 2.880938136152729],
[1.0340769105164718, -1.644983714762732, -1.9600910202942143, -1.8591769314042192, 0.8443821801123903, 2.741603222422441, 2.855955109437254],
[1.0137528536923393, -1.6262824651797032, -1.9726102241631058, -1.8763549542898232, 0.8555107436279634, 2.7257101739020397, 2.850096455134897],
[0.9953978807859496, -1.6044584588996929, -1.9865130724088753, -1.8922144621671624, 0.8709147270402129, 2.70699981111533, 2.8420586188562056],
[0.9736291416936602, -1.590494567957466, -1.9962784204975852, -1.9090988411941936, 0.8710849764353039, 2.693805299355199, 2.8426200915523787],
[0.9539208291759573, -1.5731462285576914, -2.0075996664498312, -1.9245838597650362, 0.8762954533160361, 2.677632657315823, 2.8405060833742692],
[0.9325675471853998, -1.5597137688277551, -2.016813161219989, -1.9401834556245312, 0.8738917817875791, 2.664076762007425, 2.842680786156151],
[0.9092671070566611, -1.5505322020144705, -2.0238021947233706, -1.9559241811829589, 0.8635930602954236, 2.6534162684739195, 2.849303903256867],
[0.8753469536925547, -1.5616408166113152, -2.021161363639747, -1.975039332784247, 0.8177527578929897, 2.6576236573063623, 2.876190740381722],
[0.8558778051889959, -1.5459569948900955, -2.0311335116626053, -1.9879421769052437, 0.8179434944365207, 2.6412734948567476, 2.8770539019907573],
[0.834561955518911, -1.5343214848406954, -2.039157062415979, -2.000931265267594, 0.8106173597202337, 2.627527371345965, 2.882252790446532],
[0.8190043391071935, -1.5130401533059876, -2.0515587347632236, -2.0112846348659286, 0.8192391135386767, 2.606181655015346, 2.8782842097955945],
[0.800471091984285, -1.4978656613057586, -2.061091195796666, -2.0221110776843956, 0.8169974056795901, 2.5889795651732768, 2.8805399327734125],
[0.7864890285418618, -1.4757438591583913, -2.0739471883257563, -2.0308840029758635, 0.8257366381270757, 2.5660812384271434, 2.8765033156240825],
[0.7750981354743125, -1.4504728085061345, -2.0885424814907108, -2.0383719264343796, 0.8388234099283087, 2.54023255728369, 2.8700216962225835],
[0.7624407204030765, -1.428460283221777, -2.1017247897520965, -2.045746996987889, 0.8460720083589619, 2.516380598866864, 2.86683643868601],
[0.744179011538562, -1.4161376427548118, -2.1099193835758343, -2.0540997888181964, 0.8379555248900203, 2.4997204335217713, 2.872207132672006],
[0.746072676400857, -1.374542952854584, -2.134534922215985, -2.0565831897173785, 0.8740226997461154, 2.459069529072131, 2.8529902843463315],
[0.7289187041871124, -1.362462900440027, -2.1428681880364158, -2.0634284037756094, 0.8652619990064743, 2.4417991012206044, 2.85848768303171],
[0.7037604001228486, -1.3623545348955215, -2.1442616446160607, -2.0714942609691125, 0.838829821456951, 2.4338361454834763, 2.873754946742529],
[0.7006075758518016, -1.3321288702677792, -2.1636253321515153, -2.073660987172021, 0.8569901703798619, 2.4008811272635824, 2.8641805779462772],
[0.6867622003601195, -1.3178102249868793, -2.173785807519651, -2.077702630504988, 0.8517574545512634, 2.380463596594625, 2.8675038642892265],
[0.6572073872343447, -1.325010413724727, -2.170642756709622, -2.0844682798516496, 0.815453357208276, 2.3771996809668607, 2.888190817905096],
[0.6411070811634296, -1.3148528637987895, -2.1783367981547115, -2.087517705452533, 0.8047349994491917, 2.3592813019632577, 2.894521125172868],
[0.6499162085524324, -1.2740676130083026, -2.206344915060994, -2.0847541472459756, 0.8383646308231434, 2.315859584300086, 2.875650064651867],
[0.6440193589723666, -1.2536911800361472, -2.222012550585813, -2.084512425450508, 0.8426099590838882, 2.2884886763723054, 2.87331301491759],
[0.624876607655656, -1.249942352372666, -2.2261200287404024, -2.0860897768574977, 0.8237609326341104, 2.2747350298670304, 2.8838076674613773],
[0.6108557398957775, -1.2408811105330781, -2.2342699407330207, -2.085964847766605, 0.8128326480682402, 2.2560068429844358, 2.8898273321656816],
)
        drawer_joint_info = get_joint_info(world.kitchen,56)
        drawer_lower_lim = drawer_joint_info.jointLowerLimit
        #drawer_upper_lim = drawer_joint_info.jointUpperLimit
        drawer_upper_lim = 0.30
        drawer_poses = np.linspace(drawer_lower_lim,drawer_upper_lim,len(opening_traj))
        for joint_angle,drawer_pose in zip(opening_traj,drawer_poses):
            #print('opening joint angle',joint_angle)
            tool_link = link_from_name(world.robot, 'panda_hand')
            robot_position = get_link_pose(world.robot,tool_link)
            #print("robot euler", robot_position)
            set_joint_positions(world.robot, world.arm_joints, joint_angle)
            set_joint_position(world.kitchen,56,drawer_pose)
            time.sleep(0.05)

def collides(x0, x1):
    for angle in [x0,x1]:
        #Need to create samples in between 
        set_joint_positions(world.robot, world.arm_joints, angle)
        #is_collision = single_collision(world.robot)
        is_collision = pairwise_collision(world.robot,world.kitchen)
        if is_collision:
            print(f"COLLISION FOUND: {is_collision}")
            return True
    return False
        
        

def robot_rrt(world, start_joint_angles, goal_region):  
    #disable_real_time()
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
        if (collides(x_nearest, x_new)):
            continue

        #print("x_nearest", x_nearest)
        #print("x_new", x_new)
        Vertices.append(x_new) #add x_new to set of nodes
        Edges[x_new] = x_nearest #add x_new x_nearest edge to dict
        
        if is_node_within_goal_region(x_new,goal_region):
            #print("start angles:",start_joint_angles)
            #print("goal angles:",goal_joint_angles)
            #print("x_new:",x_new)
            #wait_for_user()
            rrt_path = find_rrt_path(start_joint_angles,x_new,Edges)
            final_path = rrt_path[::-1]
            print('Path Found!')
            return final_path
    print("Vertices Length",len(Edges))
    print('No Path Found') # if this happens increase N, maybe add goal biasing



if __name__ == '__main__':
    start_joint_angles = (-2.333400804952562, 0.7383606900280834, 1.1742612657904137, -1.2249193382261694, -1.7583702601412796, 3.22570026313478, -1.0923235354053924)
    print("Initial starting joint angles",start_joint_angles)
    domain_file = 'pddl/kitchen.pddl'
    problem_file = 'pddl/problem.pddl'
    MAP = KMAP_JOINT

    # BOO
    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    #grasp_height = 0.1 # right above the sugar
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(0.1, 0.65, np.pi / 4))
    #sugar_box_pose = get_pose(world.get_body(sugar_box))
    #sugar_box_position, sugar_box_orientation = sugar_box_pose
    #sugar_box_angles = ((sugar_box_position[0], sugar_box_position[1], sugar_box_position[2] + grasp_height), (Euler(roll=0, pitch=np.pi/2, yaw=0)))
    #print("sugar box angles",sugar_box_angles)
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    enable_gravity()
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    #print("Going to operate the base without collision checking")
    goal_pos = translate_linearly(world, 1.2) # does not do any collision checking!!
    goal_pos[1] += 0.7
    set_joint_positions(world.robot, world.base_joints, goal_pos)

    # activity planning
    planner = ActivityPlan(domain_file, problem_file)
    plan_hc, end_hc = planner.hill_climb_search()
    wait_for_user()
    for i, act in enumerate(plan_hc):
        start_joint_angles = get_joint_positions(world.robot, world.arm_joints)
        goal_joint_angles = get_goal(act, MAP)
        print(f"{i+1}. {act.name} {act.parameters}:\n\t{goal_joint_angles}")
        wait_for_user()
        tolerance_radians = 1*(np.pi/180) # tolerance for goal_pose to make a feasible goal region
        goal_region = [(angle - tolerance_radians, angle + tolerance_radians) for angle in goal_joint_angles]
        #print("goal_region:", goal_region)
        #need another line so that the goal_region is within the joint limits
        start_in_goal = all(lower <= angle <= upper for angle, (lower, upper) in zip(start_joint_angles, goal_region))
        print("Is the start configuration within the goal region?", start_in_goal)
        final_path = robot_rrt(world, start_joint_angles, goal_region)
        wait_for_user()
        start_joint_angles = tuple(goal_joint_angles)
        print("next starting joint angles",start_joint_angles)
        if final_path:
            #world = World(use_gui=True)
            simulate_path(world, final_path)
            if act.name=='pickup':
                if 'spam' in act.parameters:
                    tool_link = link_from_name(world.robot, 'panda_hand')
                    robot_position = get_link_pose(world.robot,tool_link)
                    set_pose(world.get_body(spam_box),robot_position)
                else:
                    tool_link = link_from_name(world.robot, 'panda_hand')
                    robot_position = get_link_pose(world.robot,tool_link)
                    set_pose(world.get_body(sugar_box),robot_position)
            wait_for_user()
        else:
            print("not path found")
            wait_for_user()
            
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
# goal_joint_angles = (2.838311267173578, 1.7244685769643007, -2.84226720544487, -1.7908654445975372, 1.7530127023674704, 1.1025644779622914, 0.17423056193600317)
# goal_joint_angles = [1.1723088743324874, -0.09932000222746469, -1.0510790462723252, -2.062314453545313, -0.09528274438761919, 2.011110746888584, 0.9494727751742751]
# #print("goal:", goal_joint_angles)
# #start_joint_angles = tuple(sample_goal())
# start_joint_angles = (-2.333400804952562, 0.7383606900280834, 1.1742612657904137, -1.2249193382261694, -1.7583702601412796, 3.22570026313478, -1.0923235354053924)
# #print("Modified goal_joint_angles:", goal_joint_angles)
# #print("starting pose:", start_pose)
# #print(goal_pose)
# #print("start joint angles:", start_joint_angles)
# #print("goal joint angles", goal_joint_angles)
# #start_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, start_pose, max_candidates=INF, norm=INF), None)
# #goal_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, goal_pose, max_candidates=INF, norm=INF), None)
# tolerance_radians = 5*(np.pi/180) # tolerance for goal_pose to make a feasible goal region
# goal_region = [(angle - tolerance_radians, angle + tolerance_radians) for angle in goal_joint_angles]
# #print("goal_region:", goal_region)
# #need another line so that the goal_region is within the joint limits
# start_in_goal = all(lower <= angle <= upper for angle, (lower, upper) in zip(start_joint_angles, goal_region))
# #print("Is the start configuration within the goal region?", start_in_goal)
# #sample_fn = get_sample_fn(world.robot, world.arm_joints)
# #print("Sample_fn", sample_fn)
# #print("joint space distance:", joint_space_distance(start_joint_angles,goal_joint_angles))
# #sample_fn = get_sample_fn(world.robot, world.arm_joints)
# #ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
# #final_path = robot_rrt(world, start_joint_angles, goal_region)
# #print("goal joint angles", goal_joint_angles)
# #print("path:",final_path)
# main(start_joint_angles, goal_region)
# wait_for_user()