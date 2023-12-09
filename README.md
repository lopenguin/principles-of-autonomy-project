# 16.413 Project

## Organization

This repository is organized into as follows:

- `models` contains CAD data for the simulator.
- `pddl` contains PDDL domain and problem files, as well as a script to test PDDL activity planning.
- `src` contains the backbone of the simulator and provided utility files.
- `pddl-parser` and `ss-pybullet` are external respositories copied into this repo.

We have written the following scripts:

- `pddl/activity_planning.py` runs an activity planner over a sample problem.
- `robot_rrt.py` opens the simulation and runs RRT with no trajectory optimization.

## Activity Planning

### Running for yourself

To run our activity planning demonstration, make sure you are in the **pddl directory of this repo**. Then run:

```
python3 activity_planning.py
```

Optionally, you may test out a different problem file. In the command line, run:

```
python3 activity_planning.py problem_hard.pddl
```

### PDDL Domain

In this section we will go over how we designed our PDDL Domain. For each of the following sections, we will define the respective PDDL aspect and provide our reasoning about each choice.

#### Types:

The following are the types we declared to help structure our PDDL domain.

```
surface: Can be placed on without opening
openable: Must be opened before placing
box: Item that can be moved
```

**Explanation:** Surfaces on the countertop and in the drawer needed to be separated due to having different pre-conditions.

#### Objects:

Here is the list of defined objects in our PDDL domain.

```
surface: stove, countertop
openable: drawer
box: sugar, spam
```

**Explanation:** Given the project description, we chose only two surfaces to minimize the state space. Note that this means that the the spam is in the goal location of the sugar and must be moved first. Since the goal only requires use of the red drawer, we only required one openable. Finally, the sugar and spam are the only boxes described in the project description.

#### Predicates:

These are the predicates we define in our PDDL domain.

```
holding (box)
boxOn (box, surface)
opened (openable)
boxIn (box, openable)
surfClear (surface)
cabClear (openable)
armClear
```

**Explanation:** Holding is a neccessary predicate for the arm to pick up boxes. boxOn and boxIn are both neccessary predicates due to having different preconditions for placing boxes in the drawer or on the surfaces. Note that we assume the drawer is closed as a part of the inital state and will have to be opened, therfore opened is a predicate. We also need to check if surfaces or the drawer is empty as a precondition to placing it, so those two predicates were created. Note that these needed to be two different predicates due to how the Types were created. ArmClear is needed to ensure nothing is being held. Note that armClear, surfClear, and cabClear were made to because our PDDL domain does not seem to have a function to check if boxOn or boxIn were negative for all variations.

#### Actions:

`placeOn (box,surface)` - Used to place an object on a surface object of type "surface". Preconditions include holding(box) and surfClear(surface). Effects include not (holding (box)),not (surfClear), armClear, and boxOn(box, surface).

`placeIn (box, openable)` - Used to place an object inside an "openable" type object. Similar to `placeOn`, but has an additional precondition that the drawer must be open. Preconditions include holding(box), opened(openable), and cabClear(openable). Effects include not (holding(box)), not (cabClear(openable)), armClear, and boxIn (box,openable).

`open (openable)` - Used to open an "openable" type object, such as the drawer in this case. Preconditions include armClear. Effects include opened(openable).

`pickUp (box, surface)` - Action to pick up an object on a "surface" type object. Preconditions include boxOn (box, surface), and armClear. Effects inlcude not (boxOn(box, surface), not (armClear), surfClear (surface), and holding(box).

`cabPickUp (box, openable)` - Action to pick up an object on a "openable" type object. Similar to pickUp, but has an additional precondition that the drawer must be open. Preconditions include boxIn (box,openable), opened (openable), and armClear. Effects include not (boxIn (box, openable)), not (armClear), holding(box), and cabClear(openable).

**Additional:**

We also added an addition problem named `problem_hard.pddl` which we define to test a more difficult problem than the one give. In this `problem_hard` scenario, we need to exchange the positions of the sugar and spam, which will require the arm to temporarily place one of these two in the drawer before it can move the other to its goal location. This problem mainly helped us to test our fast forward heuristic, and check that all our actions are properly defined.

## Initial and Goal states:

Based on the PDDL Domain we have previously defined, we are ready to define our Initial, and Goal states.

Initial State:

This represents the goal state from the problem description described in our PDDL design.

```
boxOn (sugar, stove)
boxOn (spam, countertop)
armClear
cabClear (drawer)
```

Goal State:

This represents the goal state from the problem description described in our PDDL design.

```
boxOn (sugar, countertop)
boxIn (spam, drawer)
```

### Approach, Files, Functions

The code for this part of the project is entirely contained within the script `python/activity_planning.py`. Within this file, the class `ActivityPlanner` is designed as a multi-function planning class. It's initialized with the domain and problem files, and contains functions for a breadth-first graph search and enforced hill climbing.

Additional helper methods are meant for use inside the class:

- `applicable` checks if a given state satisfies the preconditions (positive and negative kept separate) of an action. This is useful to determine which actions can be taken from a given state.
- `apply` returns a new state resulting from applying the add and delete effects of an action. Add and delete effects are kept separate to allow flexibility for a fast-forward heuristic.

The reason we use a class is compartmentalization: the user only needs to pass in directories to the problem and domain once, and then can evaluate many different search methods. Depending on the needs of the future, the class structure can be adapted to accomodate multiple problem files. Code structure is much more transparent since each class method has a clear task.

Below we provide more detail on how each search is structured:

- `graph_search` uses breadth-first search to find an optimal solution. This code is based off of the library implementation of breadth-first search, which we used to figure out how to interact with the parsed PDDL. It has optional arguments intended to support computing the fast-forward heuristic at any state. By passing in a state (set of predicates), the user can start BFS from that state. By changing the `ff` flag to `True`, the user can evaluate BFS ignoring delete effects of actions.
- `hill_climb_search` uses a version of enforced hill-climbing as presented in lecture. The algorithm is as follows:

1. Set current state to the initial state
1. Determine the possible actions that can be run on the current state. Apply each action to get the possible children states.
1. Run the BFS relaxed plan on the children to assign fast-forward heuristic. Add the (action corresponding to each child state, child state, list of actions executed so far) to a queue, sorted by the heuristic value. If there are repeat heuristic values, create a subqueue for that value and add in order of observations, with new observations going to the back of the subqueue.
1. Pick the lowest heuristic in the queue and the first action in the subqueue. Set the current state to the selected child state.
1. Repeat until goal is reached

The queue system may be slightly different from the lecture notes implementation. The motivation of this was to prevent flip-flopping between actions with equivalent heuristic values. Adding a queue for actions forces the planner to go with the lowest-heuristic action without making the problem significantly more complex.

### Results
Our activity planner quickly converges to a list of actions. Particularly, it has the following output for the given activity plan:

```
BFS plan:
1. open ('drawer',)
2. pickup ('spam', 'countertop')
3. placein ('spam', 'drawer')
4. pickup ('sugar', 'stove')
5. placeon ('sugar', 'countertop')
--------------
hill climbing plan:
1. open ('drawer',)
2. pickup ('spam', 'countertop')
3. placein ('spam', 'drawer')
4. pickup ('sugar', 'stove')
5. placeon ('sugar', 'countertop')
```

Note that the BFS and hill climbing plans are the same, since the predicate space is so simple.

## Motion Planning

### Running for Yourself
To run the motion planner, execute the following command:

```
python3 final_project.py rrt
```

### High Level Description
The motion planner first integrates the PDDL from activity planning by generating a plan based off the goal and end states provided. It will then sequentially execute each action by generating a rapidly exploring random trees (RRT) trajectory in joint space. Upon starting, the kitchen environment and robot arm will be created. The robot arm will be set near the kithcen counter to complete the tasks and the sugar and spam boxes will be generated. The user will then be prompted to start the simulation. Once started, the activity planner will determine the correct steps to achieve the goal, and the first step will be sent to the RRT motion planner. You will see some strange, jumpy behavior from the arm. This is the collision checker ensuring that the final RRT path does not collide with the environment. Once the path is found, the robot arm will move to the next location. The user will then be prompted to continue the simulation, and the next step in the activity plan will be activated by the motion planner. This continues until the goal is reached. Once the goal is reached, the simulation environment closes, ending the simulation.

### Organization

The motion planning process involves the following scripts:

- `final_project.py`: The main script integrating different modules. This script contains both motion planning with RRT and trajectory optimization (to be explained in the next section)
- `helpers.py`: Provides support functions and configurations as well as RRT functions and a hard-coded drawer opening trajectory.

### Set-Up

The world is initialized with the `KitchenRobot` class. It creates the world with the graphical user interface (GUI), creates and places `sugar_box` and `spam_box`, and linearly translates the robot arm near the counter (but not colliding with). Additionally, it can generate the robot arm lower and upper joint angle limits. It's sub functions are described as follows:

- `generate_plan(self)` generates a PDDL plan using hill climb search described in the previous section
- `rrt_on_action(self, start, goal, goal_region)` generates an RRT trajectory of robot arm angles that avoids collisions between start and end, moves to a desired goal region, and remains within joint angles limits.

The overall simulation is handled by `main()`, which first establishes the robot and kitchen. It then generates an acitivty plan and determines whether RRT or trajectory optimization will be used based on user input. If there is no user input then executing `python3 final_project.py will choose either RRT or trajectory optimization randomly. The robot and kitchen world is shown in the following screenshot of the simulation.

![Kithcen World with Robot Arm](https://github.com/lopenguin/principles-of-autonomy-project/blob/main/KitchenWorld.png?raw=true)


### PDDL Integration

Once the activity plan is generated, each step is sent to a loop in `main()` that executes that specific motion plan. 

The motion planner integrates with the PDDL by obtaining the start and goal positions of the current step in the activity plan. The starting position is obtained by querying the current world state using `start = get_joint_positions(kr.world.robot, kr.world.arm_joints)` and the goal is determined based on the activity plan. Each object from the activity plan has defined joint angles experimentally determined using `test.py`. The object to joint angle mapping is described in `kitchen_map.py` and the goal is generated via `goal = get_goal(act, KMAP_JOINT)`. Note that `kitchen_map.py` has objects in both joint and task space. 

Once both the start and goal joint angles are known, they can be passed to RRT to find a path between them.

### RRT Solver
If RRT was chosen, then the path between start and end points will be generated through `rrt_on_action(start, goal, goal_region)`. The goal_region is a tuneable parameter for RRT. 

`rrt_on_action` begins by either choosing a random set of joint angles using with the robot and kitchen environment with `helpers.get_sample_fn(world.robot, world.arm_joints)` or chooses a random point within the goal region based off of a tuneable goal bias value called `RRT_GOAL_BIAS`.

### Tuneable Parameters

The goal region is determined by expanding the set of goal joint angles within a certain angle tolerance as shown:

```
goal_region = [(angle - tolerance_radians, angle + tolerance_radians) for angle in goal]
```

`tolerance_radians` is tuneable to determine the angular size of the upper and lower bounds of the goal region. It is currently set at 1 degree, but can be manually changed in `main()` on this line:

```
tolerance_radians = 1*(np.pi/180) # tolerance for goal_pose to make a feasible goal region
```
#### 2) RRT Max Iterations:

Determines the maximum number of iterations RRT will go through to find a path to the goal region. This is set at the beginning of `final_project.py` by the `RRT_MAX_ITER` variable. It is currently set to 10,000.

#### 3) RRT Goal Biasing:

Determines the probability that `x_rand` will be chosen within the goal region and is set at the beginning of `final_project.py` with `RRT_GOAL_BIAS`. It is currently set to 0.2, meaning that there is a 20% probability that `x_rand` will be chosen from within the goal region.

#### 4) Max step for `x_new`

Determines the maximum angular step for `x_new` which is truncated from `x_rand` and is set at the beginning of `final_project.py` with `RRT_MAX_STEP`. It is currently set to 5 degrees, meaning that `x_new` must be within 5 degrees from `x_nearest`.

#### 5) Max Step for Collision Checking

Determines how many points between `x_nearest` and `x_new` are checked for collisions and is set at the beginning of `final_project.py` with `RRT_COLLIDES_STEP`. It is currently set to 0.01 radians (~0.5 degrees), meaning that the collision checker will check for collisions  at 0.5 degree increments at and between `x_nearest` and `x_new`.






## Trajectory optimization
TODO

## Installation
Note that running this code will require a Linux operating system. The author's of this README used either an Ubuntu OS or Ubuntu VM with a Windows OS through VMware Workstation.

Follow the instructions below to run the code in this repository.

We'll assume `python` is Python 3.8. We also assume you have cloned the repo.

1. Install non-Python dependencies:
  ```sh
  curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
  sudo apt-get install git-lfs clang cmake python3-pip python3-venv  && git lfs install --skip-repo
  ```
2. Install Python packages: `pip install -r requirements.txt`

3. Build pybullet and pddl-parser:
  ```sh
  cd ss-pybullet/pybullet_tools/ikfast/franka_panda/ && \
  python setup.py install
  cd - && cd pddl-parser && \
  python setup.py install
  ```
4. Install Drake for trajectory optimization
```
pip install --upgrade pip
pip install drake pydrake
```












