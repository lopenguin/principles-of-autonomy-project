'''
activity_planning.py

Solves the "kitchen" problem by generating an activity plan.
'''
from pddl_parser.PDDL import PDDL_Parser

class ActivityPlan():
    def __init__(self, domain_file, problem_file):
        '''
        Parse domain and problem files
        '''
        self.parser = PDDL_Parser()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)

        # Grounding process
        self.ground_actions = []
        for action in self.parser.actions:
            for act in action.groundify(self.parser.objects, self.parser.types):
                self.ground_actions.append(act)


    # search through entire graph with BFS (uninformed search!)
    def graph_search(self):
        '''
        Uninformed search through entire graph with BFS 
        modified from pddl library implementation for testing.
        Returns plan, state
        *plan* is a list of actions
        *state* is the predicates in effect at the final node
        '''
        state = self.parser.state
        goal_pos = self.parser.positive_goals
        goal_not = self.parser.negative_goals
        ground_actions = self.ground_actions

        # Do nothing
        if self.applicable(state, goal_pos, goal_not):
            return []
        
        # Search
        visited = set([state])
        fringe = [state, None]
        while fringe:
            state = fringe.pop(0)
            plan = fringe.pop(0)
            for act in ground_actions:
                if self.applicable(state, act.positive_preconditions, act.negative_preconditions):
                    new_state = self.apply(state, act.add_effects, act.del_effects)
                    if new_state not in visited:
                        if self.applicable(new_state, goal_pos, goal_not):
                            full_plan = [act]
                            while plan:
                                act, plan = plan
                                full_plan.insert(0, act)
                            return full_plan, new_state
                        visited.add(new_state)
                        fringe.append(new_state)
                        fringe.append((act, plan))
        return None
    
    
    def hill_climb_search(self):
        '''
        Enforced hill climbing heuristic fast forward search
        Note: we never backtrack

        Algorithm:
        1. Start with initial state (expand to children)
        2. Run relaxed plan on children to assign heuristic
        3. Pick child with smallest heuristic as new state
        4. Repeat until goal is reached
        '''
        state = self.parser.state
        goal_pos = self.parser.positive_goals
        goal_not = self.parser.negative_goals
        plan = []

        queue = {}

        MAX_ITER = 100
        for _ in range(MAX_ITER):
            # check if state is goal (termination condition)
            if (self.applicable(state, goal_pos, goal_not)):
                return plan, state

            h_ff = []
            states_ff = []
            actions_ff = []

            # find possible actions from states
            for act in self.ground_actions:
                if self.applicable(state, act.positive_preconditions, act.negative_preconditions):
                    # compute the FF heuristic for the resulting state
                    new_state = self.apply(state, act.add_effects, act.del_effects)
                    h_ff.append(self.fast_forward_bfs(new_state))
                    states_ff.append(new_state)
                    actions_ff.append(act)

            if (len(h_ff) == 0):
                print("FF Heuristic Failed")
            else:

                # h_ff computed for all child actions
                # add to queue by heuristic
                lowest_h = min(h_ff)
                for i in range(len(h_ff)):
                    if (h_ff[i] == lowest_h):
                        if (h_ff[i] in queue.keys()):
                            queue[h_ff[i]].append((h_ff[i], states_ff[i], actions_ff[i], plan.copy()))
                        else:
                            queue[h_ff[i]] = [(h_ff[i], states_ff[i], actions_ff[i], plan.copy())]
                
                # get the next action
                lowest_h = min(queue.keys())
                

                next = queue[lowest_h].pop(0)
                plan = next[3]
                plan.append(next[2])
                state = next[1]


        print("Hill Climbing Failed! Returning partial plan")
        return plan, state



    
    def fast_forward_bfs(self, start):
        '''
        eliminate delete effects to find relaxed heuristic to the goal from given start node.
        returns: number of actions to reach goal
        '''
        state = start
        goal_pos = self.parser.positive_goals
        goal_not = self.parser.negative_goals
        ground_actions = self.ground_actions

        # check if goal already satisfied
        if self.applicable(state, goal_pos, goal_not):
            return 0

        # Proceed with BFS
        visited = set([state])
        fringe = [state, None]
        while fringe:
            state = fringe.pop(0)
            plan = fringe.pop(0)
            for act in ground_actions:
                if self.applicable(state, act.positive_preconditions, act.negative_preconditions):
                    new_state = self.apply(state, act.add_effects, set())
                    if new_state not in visited:
                        if self.applicable(new_state, goal_pos, goal_not):
                            full_plan = [act]
                            while plan:
                                act, plan = plan
                                full_plan.insert(0, act)
                            return len(full_plan)
                        visited.add(new_state)
                        fringe.append(new_state)
                        fringe.append((act, plan))

        return None



    # HELPER FUNCTIONS
    def applicable(self, state, positive, negative):
        '''
        check if state satisfies preconditions
        '''
        return positive.issubset(state) and negative.isdisjoint(state)

    def apply(self, state, positive, negative):
        '''
        apply the add/delete effects of an action
        '''
        return state.difference(negative).union(positive)
    

def print_plan(plan, end, name):
    print("--------------")
    if plan is not None:
        print(f"{name} plan:")
        for i, act in enumerate(plan):
            print(f"{i+1}. {act.name} {act.parameters}")
        # print("Final state:", end)
    else:
        print('No plan was found')

def main():
    domain_file = 'pddl/kitchen.pddl'
    # problem_file = 'pddl/problem.pddl'
    problem_file = 'pddl/debugproblem.pddl' # for debugging only

    # create our planner
    planner = ActivityPlan(domain_file, problem_file)

    # Breadth-first search
    plan_BFS, end_BFS = planner.graph_search()
    print_plan(plan_BFS, end_BFS, "BFS")


    # Enforced hill climbing
    plan_hc, end_hc = planner.hill_climb_search()
    print_plan(plan_hc, end_hc, "Enforced hill climbing")
    

if __name__ == "__main__":
    main()