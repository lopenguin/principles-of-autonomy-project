'''
kitchen_map.py

Dictionaries that map locations (openable, surface) to state coordinates (joint, task)
'''
import numpy as np


import pybullet as p
# from activity_planning import ActivityPlan
from pddl.activity_planning import ActivityPlan

## IMPORTED STUFF
def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler) # TODO: extrinsic (static) vs intrinsic (rotating)

def Point(x=0., y=0., z=0.):
    return np.array([x, y, z])

def Euler(roll=0., pitch=0., yaw=0.):
    return np.array([roll, pitch, yaw])

def Pose(point=None, euler=None):
    point = Point() if point is None else point
    euler = Euler() if euler is None else euler
    return point, quat_from_euler(euler)

KMAP_JOINT = {
    'place': {
        'stove': [0.0682973250952767, 0.6499127368581343, -0.0006061389437870515, -1.0625107631386301, 0.0003704864736713276, 1.7124233996773883, 0.8531605328675052],
        'countertop': [2.0263174857291624, -0.9767016766234768, -2.1946419786238747, -1.1652727156886957, -0.7616297353418, 1.796696137014173, 0.541558247303203],
        'drawer': [-0.9993014606753718, 0.5575981344031993, 0.08848226914383991, -1.9027443759647094, -0.07403383822822018, 2.4572408795101772, -0.08136335471573197]
    },
    'open': {
        'drawer': [1.1179275530880908, -1.7247582114792013, -1.905165080776905, -1.7852457187137816, 0.7740623064191361, 2.8037386402624724, 2.896613453226052]
    }
}

KMAP_TASK = {
    'place': {
        'stove': Pose(Point(0.1, 0.65, -0.25), Euler(0., -np.pi, 0.)),
        'countertop': Pose(Point(0.2, 1.1, -0.37), Euler(0., -np.pi, 0.)),
        'drawer': Pose(Point(0.45, 1.2, -0.5), Euler(0.,-np.pi,0.))
    },
    'open': {
        'drawer': Pose(Point(0.45, 1.2, -0.65), Euler(np.pi/2,0.,-np.pi/2))
    }
}

# helper function to get goal from action input
def get_goal(action, MAP):
    # Move slightly differently for 'open' task
    if (action.name == 'open'):
        return MAP['open'][action.parameters[0]]
    
    return MAP['place'][action.parameters[1]]


# test functionality
def main():
    # configuration
    domain_file = 'kitchen.pddl'
    problem_file = 'problem.pddl'
    MAP = KMAP_TASK

    # activity planning
    planner = ActivityPlan(domain_file, problem_file)
    plan_hc, end_hc = planner.hill_climb_search()

    for i, act in enumerate(plan_hc):
        goal = get_goal(act, MAP)
        print(f"{i+1}. {act.name} {act.parameters}:\n\t{goal}")

if __name__ == '__main__':
    main()