'''
activity_planning.py

Solves the "kitchen" problem by generating an activity plan.
'''
from pddl_parser import PDDL_Parser

class ActivityPlan():
    # Parse the domain and problem files
    def __init__(self, domain_file, problem_file):
        self.domain = parse_domain(domain_file)
        self.problem = parse_problem(problem_file)

    # search through entire graph
    def graph_search(self):
        raise NotImplementedError
    
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

    


def main():
    domain_file = 'pddl/kitchen.pddl'
    problem_file = 'pddl/problem.pddl'

    plan = ActivityPlan(domain_file, problem_file)
    plan.hill_climb_search()
    

if __name__ == "__main__":
    main()