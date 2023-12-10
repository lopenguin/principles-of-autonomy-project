# 16.413 Project

## Organization
*Note that installation instructions are at the bottom of this README.

This repository is organized into as follows:

- `models` contains CAD data for the simulator.
- `pddl` contains PDDL domain and problem files, as well as a script to test PDDL activity planning.
- `src` contains the backbone of the simulator and provided utility files.
- `pddl-parser` and `ss-pybullet` are external respositories copied into this repo.
- `pydrake` is used for trajectory optimization

We have written the following scripts:

- `pddl/activity_planning.py` runs an activity planner over a sample problem.
- `final_project.py` opens the simulation, runs the activity planner, RRT and Trajectory optimization
- `helpers.py` contains useful functions using `py.bullet` and the underlying functions for RRT in joint space
- `robot_rrt.py` contains a RRT-focused version we used for testing. Do not evaluate based on this.
- `test_environment.py` and `test.py` also contain simplified versions for testing.

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
The motion planner first integrates the PDDL from activity planning by generating a plan based off the goal and end states provided. It will then sequentially execute each action by generating a rapidly exploring random trees (RRT) trajectory in joint space. Upon starting, the kitchen environment and robot arm will be created. The robot arm will be set near the kithcen counter to complete the tasks and the sugar and spam boxes will be generated. The user will then be prompted to start the simulation. Once started, the activity planner will determine the correct steps to achieve the goal, and the first step will be sent to the RRT motion planner. You will see some strange, jumpy behavior from the arm. This is the collision checker ensuring that the final RRT path does not collide with the environment. Once the path is found, the robot arm will move to the next location. The user will then be prompted to continue the simulation, and the next step in the activity plan will be activated by the motion planner. This continues until the goal is reached. Once the goal is reached, the simulation environment closes, ending the simulation. The terminal commands for both RRT and trajectory optimization are seen below:

<figure>
  <img src="https://github.com/lopenguin/principles-of-autonomy-project/blob/main/screenshots/terminal_view.png?raw=true" alt="Terminal Prompt for RRT">
  <figcaption>Fig.1 - Terminal Prompt for RRT.</figcaption>
</figure>

<figure>
  <img src="https://github.com/lopenguin/principles-of-autonomy-project/blob/main/screenshots/terminal_traj_opt.png?raw=true" alt="Terminal Prompt for Trajectory Optimization">
  <figcaption>Fig.2 - Terminal Prompt for Trajectory Optimization.</figcaption>
</figure>

### Organization

The motion planning process involves the following scripts:

- `final_project.py`: The main script integrating different modules. This script contains both motion planning with RRT and trajectory optimization (to be explained in the next section)
- `helpers.py`: Provides support functions and configurations as well as RRT functions and a hard-coded drawer opening trajectory.

### Set-Up

The world is initialized with the `KitchenRobot` class. It creates the world with the graphical user interface (GUI), creates and places `sugar_box` and `spam_box`, and linearly translates the robot arm near the counter (but not colliding with). Additionally, it can generate the robot arm lower and upper joint angle limits. It's sub functions are described as follows:

- `generate_plan(self)` generates a PDDL plan using hill climb search described in the previous section
- `rrt_on_action(self, start, goal, goal_region)` generates an RRT trajectory of robot arm angles that avoids collisions between start and end, moves to a desired goal region, and remains within joint angles limits.

The overall simulation is handled by `main()`, which first establishes the robot and kitchen. It then generates an acitivty plan and determines whether RRT or trajectory optimization will be used based on user input. If there is no user input then executing `python3 final_project.py will choose either RRT or trajectory optimization randomly. The robot and kitchen world in the initial and finals states are shown in the following screenshots of the simulation.

<figure>
  <img src="https://github.com/lopenguin/principles-of-autonomy-project/blob/main/screenshots/KitchenWorld.png?raw=true" alt="Initial State">
  <figcaption>Fig.3 - Initial State: Kitchen World with Robot Arm.</figcaption>
</figure>

<figure>
  <img src="https://github.com/lopenguin/principles-of-autonomy-project/blob/main/screenshots/Final_State.png?raw=true" alt="Final (Goal) State">
  <figcaption>Fig.4 - Final (Goal) State: Kitchen World with Robot Arm.</figcaption>
</figure>

### PDDL Integration

Once the activity plan is generated, each step is sent to a loop in `main()` that executes that specific motion plan. 

The motion planner integrates with the PDDL by obtaining the start and goal positions of the current step in the activity plan. The starting position is obtained by querying the current world state using `start = get_joint_positions(kr.world.robot, kr.world.arm_joints)` and the goal is determined based on the activity plan. Each object from the activity plan has defined joint angles experimentally determined using `test.py`. The object to joint angle mapping is described in `kitchen_map.py` and the goal is generated via `goal = get_goal(act, KMAP_JOINT)`. Note that `kitchen_map.py` has objects in both joint and task space. 

Once both the start and goal joint angles are known, they can be passed to RRT to find a path between them.

### RRT Solver
If RRT was chosen, then the path between start and end points will be generated through `rrt_on_action(start, goal, goal_region)`. The goal_region is a tuneable parameter for RRT. 

#### x_rand
`rrt_on_action` begins by either choosing a random set of joint angles, called `x_rand` that is within joint angle limits within the robot and kitchen environment with `helpers.get_sample_fn(world.robot, world.arm_joints)` or chooses a the goal set of joint angles based off of a tuneable goal bias value called `RRT_GOAL_BIAS`.

#### x_nearest
Next, `x_nearest` is calculated in `find_x_nearest(Vertices,x_rand)` by choosing the node that is closest in joint angle space to `x_rand`. To determine the closest set of joint angles the J2 norm (Euclidean Distance) was used between each set of joint angles as shown in the following:

The Euclidean distance between two 7-dimensional vectors (e.g., joint angle configurations of a robotic arm) is computed as:

`||v - w||_2 = sqrt((v1 - w1)^2 + (v2 - w2)^2 + ... + (v7 - w7)^2)`

Here, `v = (v1, v2, ..., v7)` and `w = (w1, w2, ..., w7)` represent two different joint angle configurations.

#### x_new
Now `x_new` can be calculated with `x_new = tuple(helpers.find_x_new(x_nearest,x_rand, RRT_MAX_STEP, self.lower_limits, self.upper_limits))`. This will calculate an `x_new` that is within the degree limits of RRT_MAX_STEP and within joint angle limits.

#### Collision Checking
Once `x_new` is created, we must confirm there is no collision at this point or any points between `x_nearest` and `x_new`. The function `collides(self, x0, x1)` takes in two joint angle vectors and performs collision checking at both points and at in-between points of fidelity based on the tuneable RRT_COLLIDES_STEP. `numpy` functions are used to created the angles to be checked using `np.linspace`.

Once each angle to be collision checked is created, we use a `pybullet_tools.utils` function called `pairwise_collision`. First, the joint angle is set in the kitchen environment, then a the collision function is run against `world.robot` and `world.kitchen`, which will result in a binary true or false statement of whether there was a collision. If there is a collision between `x_nearest` and `x_new`, then `x_new` is discarded, and the loop starts over with choosing a new `x_rand`.

Note that this collision checking method causes the robot to move in the GUI in a very rapid and strange fashion. However, we ignore this, understanding that this is just a part of the RRT determining collisions and is not the actual final RRT trajectory.

#### Finding the Final RRT Trajectory
If `x_new` passes collision checking, then the path between it and `x_nearest` is obstacle free and within joint angle limits. It is then added as a node in the RRT tree.

We then check if `x_new` is within the goal region with `helpers.is_node_within_goal_region(x_new, goal_region)`. This determines if `x_new` is within `goal_region`. If it is, then RRT is complete and the final path is determined through `helpers.find_rrt_path(start, x_new, edges)` which starts from `x_new` and appends their parent value (listed as a key in a python dictionary). This is repeated until the starting joint angles are added to the path.


### Tuneable Parameters

#### 1) Goal-Region Tolerance

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

#### 3) RRT Max Step:
Determines the maximum angular distance between `x_nearest` and `x_new`. This is set at the beginning of `final_project.py` by the `RRT_MAX_STEP` variable. It is currently set to 5 degrees.

#### 4) RRT Goal Biasing:

Determines the probability that `x_rand` will be equal to the goal state and is set at the beginning of `final_project.py` with `RRT_GOAL_BIAS`. It is currently set to 0.2, meaning that there is a 20% probability that `x_rand` will be equal to the goal joint angles.
Determines the maximum angular step for `x_new` which is truncated from `x_rand` and is set at the beginning of `final_project.py` with `RRT_MAX_STEP`. It is currently set to 5 degrees, meaning that `x_new` must be within 5 degrees from `x_nearest`.

#### 5) Max Step for Collision Checking

Determines how many points between `x_nearest` and `x_new` are checked for collisions and is set at the beginning of `final_project.py` with `RRT_COLLIDES_STEP`. It is currently set to 0.01 radians (~0.5 degrees), meaning that the collision checker will check for collisions  at 0.5 degree increments at and between `x_nearest` and `x_new`.

### Example Case
We continue using the same example plan from the activity planning section with the following steps:
```
1. open ('drawer',)
2. pickup ('spam', 'countertop')
3. placein ('spam', 'drawer')
4. pickup ('sugar', 'stove')
5. placeon ('sugar', 'countertop')
```

This plan is mapped to a set of start and goal joint angles and then a trajectory for each action is found with the RRT method described above. Please view the following video for the results. Note that the collision checking motion was removed and only the RRT trajectories are shown. Additionally, all motions are generated by RRT except for the drawer opening motion, which was created via hard-code trajectories found using inverse kinematics via `next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)` in `test.py`. Finally, drawer opening and object grabbing are described in the Simulate Path section below.




https://github.com/lopenguin/principles-of-autonomy-project/assets/46176315/74989039-824e-4ab7-a1f1-56e4ab479367







## Trajectory optimization
### Running for Yourself
To run the motion planner with trajectory optimization, execute the following command:

```
python3 final_project.py optimizes
```
### Details
Trajectory optimization follows the same organization, setup, and PDDL integration as RRT trajectory generation.

The specific code for trajectory optimization is in `KitchenRobot.optimize_trajectory(self, start, goal, num_pts)` within `final_project.py`. The code uses pydrake to set up an optimization problem (in `MathematicalProgram`) to find the shortest path over joint angles between the start and end states. The only tunable parameter is the number of points to consider on this trajectory; the default is set at 50 which is more than enough to find feasible trajectories within the kitchen environment. The minimizing function was created using `AddCost`. Start and end states were constrained using `AddLinearEqualityConstraint` and joint angle limits were constrained with `AddBoundingBoxConstraint`. Note that there are no collision avoidance constraints. Despite this, the entire trajectory is designed so that collisions will not occur, such as moving the arm out of the way after opening the drawer. For details on the constrained optimization set-up, please see `16_413 Grad Project Final Write-Up.pdf` in the project files.

### Example Case
Our example case proceeds similarly to the RRT example, although the motion is considerably smoother. Note that after opening the drawer an additional vertical trajectory occurs that is not found in the RRT. This is another optimal trajectory to ensure the entire trajectory over all activities is collision free. Like RRT, the only hard-coded trajectory is the drawer opening sequence. See the video:



https://github.com/lopenguin/principles-of-autonomy-project/assets/46176315/9512b4ce-a83a-4e39-90a6-881f1d442250



## Simulating the Path
Once the path is created either from RRT or trajectory optimization, it is passed to `simulate_path(self, act, path)`, which also takes into account the current action `act`. For each point in the path, the robot arm is moved via `set_joint_positions(world.robot, world.arm_joints, point)` and a collision checker follows. If there is a collision during this stage, then the terminal will say "Collision Found" at each point it collides in the path. However, since RRT already considers collisons and trajectory optimization also accounts for this, the path should have no collisions for either for the given scenario.

Next, if the action is `placein` or `placeon`, the robot's pose as (X,Y,Z,quaternion) will be calculated via `robot_position = get_link_pose(world.robot,self.tool_link)`. Then, the pose of the object to be grabbed (`spam` or `sugar_box`) will be matched to the robot arm gripper. Note that we only change the X,Y,Z position and not the quaternion to retain the object's original attitude. Additionally, both objects have slight offsets in the -Z direction to make it look more realistic when "gripping". 

If the action is `open`, the generated trajectory will move the robot arm right in front of the handle of the closed drawer. Then, the hard-coded opening trajectory ensues. The drawer is open by finding its specific number in the kitchen environment and updating its drawer position in step with the robot arm. A drawer opening trajectory of equal length to the hard-coded trajectory was created via `np.linspace` so that it smoothly opens the drawer consistent with robot motion. Additionally, if trajectory optimization is run, then the robot arm will perform one extra movmement calculated using Drake. This moves the arm directly above the drawer so that follow-on actions result in no collision. This ensures the entire sequence provided by trajectory optimization is collision free.

<figure>
  <img src="https://github.com/lopenguin/principles-of-autonomy-project/blob/main/screenshots/Drawer_Open.png?raw=true" alt="Final Drawer Open State with RRT">
  <figcaption>Fig.5 - Final Drawer Open State with RRT.</figcaption>
</figure>

<figure>
  <img src="https://github.com/lopenguin/principles-of-autonomy-project/blob/main/screenshots/traj_opt_drawer.png?raw=true" alt="Final Drawer Open State with Trajectory Optimization">
  <figcaption>Fig.6 - Final Drawer Open State with Trajectory Optimization.</figcaption>
</figure>

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








