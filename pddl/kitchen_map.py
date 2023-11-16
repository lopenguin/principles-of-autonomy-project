'''
kitchen_map.py

Dictionaries that map locations (openable, surface) to state coordinates (joint, task)
'''
import numpy as np

from  activity_planning import ActivityPlan

KMAP_JOINT = {
    'place': {
        'stove': np.array([]),
        'countertop': np.array([]),
        'drawer': np.array([])
    },
    'open': {
        'drawer': np.array([])
    }
}

KMAP_TASK = {
    'place': {
        'stove': np.array([-0.2, 0.65, np.pi / 4]),
        'countertop': np.array([]),
        'drawer': np.array([0.2, 1.1, np.pi / 4])
    },
    'open': {
        'drawer': np.array([])
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