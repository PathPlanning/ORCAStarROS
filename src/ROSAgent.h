/*!
\file
\brief File contains ROSAgent class.
*/


#include<vector>
#include <sstream>

#include "ros/ros.h"
#include "ORCAAgent.h"
#include "Const.h"
#include "ORCAStar/Init.h"
#include "ORCAStar/ORCAInput.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

#ifndef ROSAGENT_H
#define ROSAGENT_H

/*!
\defgroup ORCAStarROS ORCAStar ROS Modules
\brief  This module contains set of ROS modules for multiagent navigation system.

\details    The system currently supports ORCA and Theta* algorithms.
            It contains single agent navigation module, multiagent simulation and visualization modules

*/


/*!
* \brief Class is an implementation of single agent navigation ROS module based on ORCAStar algorithm.
* \details The main part in a navigation system is the single agent navigation module. 
* This component can be used both for embedding a separate robot and for centralized calculation of trajectories,
* but using distributed computing.
* The single agent navigation implemented as a ROS node. The node operation is divided into 2 stages.
* At the first stage, node is initialized, the algorithms parameters and static environment data are requested from ROS
* services, global path is created. 
* The node sends a request to service `initAgentServer_*id*` 
* (Service description: [here](https://haiot4105.github.io/ORCAStarROS/srv/Init.html), `id` is identifier of agent, 
* which sets as ROS private parameter) and awaits a response. 
* For using ROS Agent node you should launch such service. 
* After that, the node sends a request to the service `static_map` of `map_server` node. 
* You should launch this node before launching ROS Agent node. 
* At the second stage the process of following the global path starts. 
* The node waits for messages in topic `AgentInput_*id*`. 
* After a message with input data ([here](https://haiot4105.github.io/ORCAStarROS/msg/ORCAInput.html)) appears in the 
* topic `AgentInput_*id*`, new velocity computation procedure starts. 
* New velocity publishes in the topic `AgentOutput*id*` (in [geometry_msgs::Point32](http://docs.ros.org/api/geometry_msgs/html/msg/Point32.html) format).
* The second stage continues until the node is working. An illustration of the operation of the node is presented in the figure below.

<p align="center">
  <img width="300" src="./images/ROSAgent.png" alt="Map sheme" >
</p>
* \ingroup ORCAStarROS
*/


class ROSAgent
{

public:

    /*!
    ROSAgent default constructor
    */
    ROSAgent() = default;

    /*!
    ROSAgent constructor with agent id.
    Use this constructor for correct initialization.
    In this constructor ROS communication objects are created and initial input data is requested.
    \param id identifier for debug and topics names
    */
    ROSAgent(size_t id);

    /*!
    ROSAgent default destructor
    */
    ~ROSAgent() = default;


    /**
     * The method for calculating the new velocity.
     * Contains the correct start order of the ORCAStar algorithm procedures. ORCAStar algorithm see implemented in \ref ORCAStarLib.
     * This method is called automatically when a message appears in the AgentInput_*agentId* topic.
     * \param msg ORCAStar input data as ROS message
    */
    void DoStep(const ORCAStar::ORCAInput &msg);


private:
        //! \cond
    size_t agentId; ///< Identifier for debug and topics names
    ORCAAgent *agent; ///< ORCAAgent instance for navigation
    std::vector<Agent *> neighbous; ///< Vector of neighbous on current step


    ros::NodeHandle n; ///< ROS node object
    ros::Publisher ROSAgentPub; ///< ROS publisher object
    ros::Subscriber ROSAgentSub; ///< ROS subscriber object

    EnvironmentOptions *options; ///< Navigation environment options
    AgentParam *param; ///< Navigation parameters
    Map *map; ///< Map object for navigation
    //! \endcond




};

#endif // ROSAGENT_H
