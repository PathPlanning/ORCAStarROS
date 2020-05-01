#include <vector>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "ORCAStar/AgentState.h"
#include "ORCAStar/AgentVelocity.h"
#include "ORCAStar/ORCAInput.h"
#include "Geom.h"
#include "ORCAStar/Init.h"

#ifndef ORCASTAR_ROSSIMACTOR_H
#define ORCASTAR_ROSSIMACTOR_H



class ROSSimActor
{
    public:
        ROSSimActor();
        ~ROSSimActor();
        void ReceiveAgentStates(const ORCAStar::AgentState &msg);
        void TransmitAgentVelocity(const geometry_msgs::Point32 &msg);
        void CreateORCAInputMsg();

    private:


        bool InitAgent(ORCAStar::Init::Request  &req, ORCAStar::Init::Response &res);


        size_t id;
        size_t agNumber ;
        float sightRad;
//        std::vector<Point> positions;
//        std::vector<Point> velocities;
//        std::vector<float>   radius;

        ORCAStar::AgentVelocity agentVelocityMsg;
        ORCAStar::ORCAInput inputORCAMsg;
        ORCAStar::Init::Response initData;

        ros::NodeHandle n;
        ros::Publisher ROSSimActorPub;
        ros::Subscriber ROSSimActorSub; 

        ros::Publisher ROSSimActorToAgentPub;
        ros::Subscriber ROSSimActorToAgentSub;

        bool initFlag;
};



#endif //ORCASTAR_ROSSIMACTOR_H
