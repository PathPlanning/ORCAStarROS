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

/**
* @brief The ScanNPlan class is a client of the vision and path plan servers.  The ScanNPLan class takes
* these services, computes transforms and published commands to the robot.
*/
class ROSAgent
{
public:
    ROSAgent() = default;
    ROSAgent(size_t id);
    ~ROSAgent() = default;

    void DoStep(const ORCAStar::ORCAInput &msg);


private:
    size_t agentId;
    ORCAAgent *agent;
    std::vector<Agent *> neighbous;


    ros::NodeHandle n;
    ros::Publisher ROSAgentPub;
    ros::Subscriber ROSAgentSub;

    EnvironmentOptions *options;
    AgentParam *param;
    Map *map;





};

#endif // ROSAGENT_H
