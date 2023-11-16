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
        'stove': [1.1723088743324874, -0.09932000222746469, -1.0510790462723252, -2.062314453545313, -0.09528274438761919, 2.011110746888584, 0.9494727751742751],
        'countertop': [0.47629213712860496, 1.1255312928859258, -1.6065791270239005, -1.9140833006606703, 1.138238260575534, 1.6856110791200738, -0.6352417171473008],
        'drawer': [0.6436414273380775, -1.1248249465455373, -1.802781048452051, -2.1748346370157474, -1.3069435568944163, 1.9995133225951625, 0.3516916273779094]
    },
    'open': {
        'drawer': [-2.2573818107859815, 1.12669938031197, 0.846416970968698, -2.1438169865576198, 1.2167940897598966, 2.2723351961370124, 2.628284968846344]
    }
}

KMAP_TASK = {
    'place': {
        'stove': Pose(Point([-0.2, 0.65, -0.25]), Euler(0., -np.pi, 0.)),
        'countertop': Pose(Point([0.2, 1.1, -0.37]), Euler(0., -np.pi, 0.)),
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