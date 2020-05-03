# ORCA* ROS
Decentralized navigation system based on ORCA and Theta* algorithms and implemented as ROS nodes

## ORCA* Algorithm

Implementation of ORCA algorithm with global path planner based on Theta* algorithm and computing differential-drive control outputs **[LINK]**

### Description
The algorithm is based on the idea of planning a global path for all agents independently and moving along this path with local collision avoidance. Theta* algorithm are using to global path planning and ORCA algorithm are using for local collision avoidance with agents and static obstacles. Also direct moving to goal without global planning is available.

**Theta*** is a version of A* algorithm for any-angle path planning on grids. Theta* mostly the same as A*, but, unlike A*, Theta* allows parent of current vertex may be any other vertex, which are visible from current [[1](https://arxiv.org/pdf/1401.3843.pdf)].

The **ORCA** algorithm is a decentralized collision avoidance algorithm in a multi-agent environment. The main idea of ​​the algorithm is iterative selection of new agent speed, close to a certain preferred velocity. Selection of the new speed is based on ORCA principle. **Optimal Reciprocal Collision Avoidance (ORCA)** — the principle, which provides a sufficient condition for multiple robots to avoid collisions among one another, and thus can guarantee collision-free navigation [[2](http://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf)].
The principle is based on the concept of velocity obstacles, which are used to search for a new  speed of agent so that during the time **_t_** there is no collision with other agents. In the process of searching for a new velocity, algorithm creates a set of n-1 linear constraints(where n is the number of agents that the current one takes into account). A new velocity (Vnew) that satisfies these constraints and is close to the preferred velocity, are searched using an linear programming. The preferred velocity is selected so that the agent makes a move to the target point. More information about ORCA algorithm you can find at ORCA creators web page [[3](http://gamma.cs.unc.edu/ORCA/)].

The agent is a disk of radius _r_ with buffer radius _ε_ centered at _p_ with their start and global goal positions. 


<p align="center">
  <img width="500" src="./images/reps.png" alt="Agent sheme" >
</p>

For each neighboring agent (located at a distance _R_ or less), their position and current speed are known. At the beginning of task execution every agent tries to find a path to their global goal, using Theta* algorithm. After getting of global path (which is a sequence of points in space, where the first point is the global goal), agent begin moving. First target point is the last element in path sequence. At each simulation step, for each agent, a new velocity _Vnew_ are searched, using ORCA algorithm. After that global simulation time is changed to _dt_, the position of all agents is changed to _dt_ * _Vnew_ (own for each agent) and every agent compute new target point. If the agent has reached the last point in the sequence, then the next point in the sequence becomes the target, and the previous one is deleted. Otherwise, a visibility check is made between the agent’s position and the last point. If visibility is confirmed, then the last becomes the target point, otherwise the path from the agent's position to the last point is searched.

Block scheme of the algorithm is shown in the figures below.

<p align="center">
  <img width="1000" src="./images/scheme.png" alt="Algorithm block sheme" >
</p>


The implementation of ORCA* algorithm based on **[LINK]**. Implementation relies only on C++11 standard and STL. Open-source library to work with XML (tinyXML) is included at the source level (i.e. .h and .cpp files are part of the project).

## ROS Imlemetation

TODO

## Getting Started

### Requirements

TODO

### Installing

Download current repository to your local machine. Use

```bash
git clone https://github.com/haiot4105/ORCAStarROS.git
```
or direct downloading.

### Build

TODO

### Run Tests

TODO

### ROS Agent

TODO

#### Launch

TODO

#### I/O Format

TODO

### ROS Mission

#### Launch

TODO

#### I/O Format

TODO

Input files are an XML files with a specific structure.
Input file should contain:

* Mandatory tag `<agents>`. It describes the parameters of the agents.
    * `number` — mandatory attribute that define the number of agents;
    * `<default_parameters>` — mandatory tags that defines default parameters of agents and agent's perception.
      * `agentsmaxnum` — mandatory attribute that defines a number of neighbors, that the agent takes into account;
      * `movespeed` — mandatory attribute that defines maximum speed of agent;
      * `sightradius` — mandatory attribute that defines the radius in which the agent takes neighbors into account;
      * `size` — mandatory attribute that defines size of the agent (radius of the agent);
      * `timeboundary` — mandatory attribute that defines the time within which the algorithm ensures collision avoidance with other agents;
      * `timeboundaryobst` — mandatory attribute that defines the time within which the algorithm ensures collision avoidance with static obstacles.
    * `<agent>` — mandatory tags that defines parameters of each agent.
        * `id` — mandatory attribute that defines the identifier of agent;
        * `start.xr` — mandatory attribute that defines the coordinate of start position on the x-axis (hereinafter, excluding `map` tag, points (x,y) are in coordinate system, which has an origin (0,0) in lower left corner. More about coordinate systems in the illustration below);
        * `start.yr` — mandatory attribute that defines the coordinate of start position on the y-axis;
        * `goal.xr` — mandatory attribute that defines the coordinate of finish position on the x-axis;
        * `goal.yr` — mandatory attribute that defines the coordinate of finish position on the y-axis;
        * `agentsmaxnum` — attribute that defines a number of neighbors, that the agent takes into account;
        * `movespeed` — attribute that defines maximum speed of agent;
        * `sightradius` — attribute that defines the radius in which the agent takes neighbors into account;
        * `size` — attribute that defines size of the agent (radius of the agent);
        * `timeboundary` — attribute that defines the time within which the algorithm ensures collision avoidance with other agents;
        * `timeboundaryobst` — attribute that define the time within which the algorithm ensures collision avoidance with static obstacles.
* Mandatory tag `<map>`. It describes the environment for global path planning.
  * `<height>` and `<width>` — mandatory tags that define size of the map. Origin is in the upper left corner. (0,0) - is upper left, (*width*-1, *height*-1) is lower right (more about coordinate systems in the illustration below).
  * `<cellsize>` — optional tag that defines the size of one cell.
  * `<grid>` — mandatory tag that describes the square grid constituting the map. It consists of `<row>` tags. Each `<row>` contains a sequence of "0" and "1" separated by blanks. "0" stands for traversable cell, "1" — for untraversable (actually any other figure but "0" can be used instead of "1").

* Mandatory tag `<obstacles>`. It describes static obstacles for collision avoidance.
  * `number` — mandatory attribute that defines the number of obstacles;
  * `<obstacle>` — mandatory tags which defines each static obstacles for collision avoidance.
    * `<vertex>` — mandatory tags which defines vertex of static obstacle for collision avoidance.
      *  `xr` — mandatory attribute that defines the coordinate of vertex on the x-axis;
      *  `yr` — mandatory attribute that defines the coordinate of vertex on the y-axis.

* Mandatory tag `<algorithm>`. It describes the parameters of the algorithm.
  * `<delta>` — mandatory tag that defines the distance between the center of the agent and the finish, which is enough to reach the finish (ORCA parameter);
  * `<timestep>` — mandatory tag that defines the time step of simulation (ORCA parameter);
  * `<searchtype>` — tag that defines the type of planning. Possible values - "thetastar" (use Theta* for planning), "direct" (turn off global planning and always use direction to global goal). Default value is "thetastar" (global planning parameter);
  * `<breakingties>` — tag that defines the priority in OPEN list for nodes with equal f-values. Possible values - "0" (break ties in favor of the node with smaller g-value), "1" (break ties in favor of the node with greater g-value). Default value is "0" (Theta* parameter);
  * `<cutcorners>` — boolean tag that defines the possibilty to make diagonal moves when one adjacent cell is untraversable. The tag is ignored if diagonal moves are not allowed. Default value is "false" (Theta* parameter);
  *  `<allowsqueeze>` — boolean tag that defines the possibility to make diagonal moves when both adjacent cells are untraversable. The tag is ignored if cutting corners is not allowed. Default value is "false" (Theta* parameter);
  *  `<hweight` — defines the weight of the heuristic function. Should be real number greater or equal 1. Default value is "1" (Theta* parameter);

<p align="center">
  <img width="1000" src="./images/map.png" alt="Map sheme" >
</p>

#### Visualisation

TODO

## Links

1. [Daniel K. et al. Theta*: Any-angle path planning on grids // Journal of Artificial Intelligence Research. – 2010. – vol. 39. – p. 533-579.](https://arxiv.org/pdf/1401.3843.pdf)
2. [Van Den Berg J. et al. Reciprocal n-body collision avoidance // Robotics research. – Springer, Berlin, Heidelberg, 2011. – p. 3-19.](http://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf)
3. [ORCA creators webpage](http://gamma.cs.unc.edu/ORCA/)
