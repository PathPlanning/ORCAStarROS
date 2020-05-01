
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "ORCAStar/AgentState.h"
#include "ORCAStar/AgentVelocity.h"
#include "ORCAStar/Init.h"
#include "Geom.h"
#include "XMLReader.h"
#include "Agent.h"
#include "Summary.h"

#ifndef ORCASTAR_ROSMISSION_H
#define ORCASTAR_ROSMISSION_H




class ROSMission
{

    public:

        ROSMission(std::string fileName, size_t agNum);
        ~ROSMission();

        bool PrepareSimulation();
        void StartSimulation();
        void GenerateAgentStateMsg();
        void UpdateVelocity(const ORCAStar::AgentVelocity &msg);
        void UpdateState();

    private:

        bool ReadTask();
        bool InitAgent(ORCAStar::Init::Request  &req, ORCAStar::Init::Response &res);

        size_t agNumber;
        size_t agCount;
        unsigned int stepsCount;
        unsigned int stepsTreshhold;
        //      unsigned int collisionsCount;
        //      unsigned int collisionsObstCount;


        ORCAStar::AgentState agentStateMsg;
        ros::NodeHandle n;
        ros::Publisher ROSMissionPub;
        ros::Subscriber ROSMissionSub;
        ros::Rate *loopRate;


        Reader *taskReader;
        std::vector<Agent *> agents;
        EnvironmentOptions *options;
        Map *map;
        Summary missionResult;
        std::unordered_map<int, std::pair<bool, int>> resultsLog;
        bool initFlag;






};


#endif //ORCASTAR_ROSMISSION_H
