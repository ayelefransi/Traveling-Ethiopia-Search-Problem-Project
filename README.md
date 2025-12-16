##Traveling Ethiopia Search Problem & Robotic Simulation
##Project Overview
This project implements various Artificial Intelligence search algorithms to solve the "Traveling Ethiopia" problem. It progresses from basic uninformed search (BFS/DFS) to heuristic search (A*), adversarial search (MiniMax), and finally, a 3D robotic simulation using ROS 2 and Gazebo.

##Prerequisites* **Programming Language:** Python 3.x
* **Key Libraries:** `collections` (deque), `heapq`, `math`
* **OS:** Ubuntu 24.04 (Noble Numbat)
* **Robotics Middleware:** ROS 2 Jazzy Jalisco
* **Simulator:** Gazebo Harmonic (gz-sim)

---

## Project Structure
###Question 1: Uninformed Search Strategies* **Goal:** Convert the state space graph (Figure 1) into a data structure and implement search strategies.
* **Implementation:**
* **Data Structure:** The map is represented as a Python Adjacency Dictionary.
* **Algorithms:**
* **Breadth-First Search (BFS):** Uses a FIFO Queue to find the shortest path in terms of hops.
* **Depth-First Search (DFS):** Uses a LIFO Stack to explore paths deeply before backtracking.




* **Usage:** Run the Jupyter notebook cell for Q1.

###Question 2: Uniform Cost Search (UCS)* **Goal:** Navigate using backward costs (distance/price) from Figure 2.
* **Implementation:**
* **Data Structure:** Weighted Adjacency Dictionary.
* **Algorithm:** Uses a `PriorityQueue` (heapq) to explore the path with the lowest cumulative cost g(n).
* **Multi-Goal Search:** A customized UCS loop that visits a dynamic list of cities (e.g., "Axum", "Lalibela", "Bale") by constantly finding the nearest unvisited goal to preserve the local optimum.


* **Usage:** Run the Jupyter notebook cell for Q2.

###Question 3: A* Search (Heuristic Search)* **Goal:** Find the optimal path from "Addis Ababa" to "Moyale" using both heuristics and path costs (Figure 3).
* **Implementation:**
* **Logic:** f(n) = g(n) + h(n)
* g(n): Backward cost (distance between cities).
* h(n): Heuristic value (estimated straight-line distance to goal).


* **Result:** This is the most efficient search, prioritizing paths that are mathematically closer to the destination.


* **Usage:** Run the Jupyter notebook cell for Q3.

###Question 4: Adversarial Search (MiniMax)* **Goal:** An agent competes against an adversary to find the best quality coffee (Figure 4).
* **Implementation:**
* **Agent (Maximizer):** Tries to maximize utility (coffee quality).
* **Adversary (Minimizer):** Tries to minimize the agent's utility.
* **Algorithm:** Recursive MiniMax function traversing the game tree to Level 3 terminals.


* **Usage:** Run the Jupyter notebook cell for Q4.

---

##🤖 Question 5: ROS 2 & Gazebo SimulationThis section involves simulating a robot in a 3D environment representing the "Relaxed State Space Graph" (Figure 5).

###1. Package Setup (Ubuntu 24.04 / ROS 2 Jazzy)Because this system uses **Ubuntu 24.04**, the simulation runs on **ROS 2 Jazzy** and **Gazebo Harmonic**.

**File Locations:**

* `~/ros2_ws/src/traveling_ethiopia/urdf/ethiopia_bot.urdf`: Defines the blue 3-wheeled differential drive robot using the `gz-sim-diff-drive` plugin.
* `~/ros2_ws/src/traveling_ethiopia/worlds/ethiopia.world`: Defines the simulation environment with cities mapped to Cartesian coordinates.
* `~/ros2_ws/src/traveling_ethiopia/launch/simulation.launch.py`: Python launch script bridging ROS 2 and Gazebo.
* `~/ros2_ws/src/traveling_ethiopia/traveling_ethiopia/navigator.py`: The control node containing the BFS logic and P-Controller to drive the robot.

###2. How to Run the Simulation**Step 1: Build the Workspace**
Open a terminal and run:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

```

**Step 2: Launch the Environment**
This opens the Gazebo simulator with the robot spawned at Addis Ababa (0,0).

```bash
ros2 launch traveling_ethiopia simulation.launch.py

```

**Step 3: Run the Navigator**
Open a **new** terminal window and run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run traveling_ethiopia navigator

```

The robot will generate a path (BFS) and autonomously drive to the destination defined in the `execute_mission` function.
