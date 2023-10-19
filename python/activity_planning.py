'''
activity_planning.py

Solves the "kitchen" problem by generating an activity plan.
'''
from pddl_parser.PDDL import PDDL_Parser

class ActivityPlan():
    # Parse the domain and problem files
    def __init__(self, domain_file, problem_file):
        self.parser = PDDL_Parser()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)

    # search through entire graph
    def graph_search(self):
        state = self.parser.state
        goal_pos = self.parser.positive_goals
        goal_not = self.parser.negative_goals
        # Do nothing
        if self.applicable(state, goal_pos, goal_not):
            return []
        # Grounding process
        ground_actions = []
        for action in self.parser.actions:
            for act in action.groundify(self.parser.objects, self.parser.types):
                ground_actions.append(act)
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
                            return full_plan
                        visited.add(new_state)
                        fringe.append(new_state)
                        fringe.append((act, plan))
        return None
    
    # enforced hill climbing
    def hill_climb_search(self):
        # search variables
        state = self.problem.init

        MAX_ITERATIONS = 100
        for _ in range(MAX_ITERATIONS):
            for a in self.domain.actions:
                # check if action is possible in current state
                print(type(a.precondition))
            break


    def test(self):
        print(f"Predicates: {self.domain.predicates}")
        print(f"Actions: {self.domain.actions}")

    # determine if a state is applicable
    def applicable(self, state, positive, negative):
        return positive.issubset(state) and negative.isdisjoint(state)

    # apply an action
    def apply(self, state, positive, negative):
        return state.difference(negative).union(positive)
    


def main():
    domain_file = 'pddl/kitchen.pddl'
    problem_file = 'pddl/problem.pddl'

    plan = ActivityPlan(domain_file, problem_file)
    plan.graph_search()
    

if __name__ == "__main__":
    main()