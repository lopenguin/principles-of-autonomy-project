'''
helpers.py

Contains helper functions.
'''
import os
import sys
import numpy as np

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, stable_z_on_aabb
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, interval_generator

from src.world import World
from src.utils import COUNTERS, compute_surface_aabb, name_from_type

UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    '''
    Adds YCB-type object to a pybullet world.
    Used for sugar and spam boxes.
    '''
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    '''
    Put a YCB object onto a surface
    '''
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

# Lambdas to add objects to the world
add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs) 

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    '''
    Autogenerates random joint angles within joint limits of arm
    '''
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def normalize_angle(angle):
    # ensures all joint angles are between [-pi pi]
    return (angle + np.pi) % (2 * np.pi) - np.pi 


'''
RRT Helper Functions
'''
def joint_space_distance(joint_angles_a, joint_angles_b):
    '''
    Computes distance between joint angles in joint space (l2 norm)
    '''
    joint_diffs = [normalize_angle(a - b) for a, b in zip(joint_angles_a, joint_angles_b)] #subtracts joint angles after normalizing to find distance between poses
    squared_diffs = [diff**2 for diff in joint_diffs] # squared difference beetween them
    return np.sqrt(sum(squared_diffs)) #total value of squared differences (i.e.  sum of squares for error)

def find_x_nearest(Vertices,x_rand):
    '''
    Tests list of x values against x_rand to find which is closest
    to x_rand
    '''
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
