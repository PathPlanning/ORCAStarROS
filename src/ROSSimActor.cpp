/*!
\file
\brief File contains ROSSimActor class implementation.
*/

#include "ROSSimActor.h"


ROSSimActor::ROSSimActor()
{
    agNumber = 0;
    id = 0;
    initFlag = false;
    sightRad = 0.0;


    ros::ServiceClient client = n.serviceClient<ORCAStar::Init>("initServer");
    ros::service::waitForService("initServer");
    ORCAStar::Init srv;

    if (client.call(srv))
    {

        client.shutdown();
        ROS_DEBUG("Parameters received!");
        id = srv.response.id;
        sightRad = srv.response.sightRadius;
        initData = srv.response;
    }
    else
    {
        ROS_ERROR("Parameters not received!");
        exit(-1);
    }

    std::stringstream serverName;
    serverName << "initAgentServer_" << id;
    ros::ServiceServer initServer = n.advertiseService(serverName.str(), &ROSSimActor::InitAgent, this);

    while(!initFlag && ros::ok())
    {
        ros::spinOnce();
    }

    initServer.shutdown();

    agentVelocityMsg.id = id;
    agentVelocityMsg.vel = geometry_msgs::Point32();

    ROSSimActorPub = n.advertise<ORCAStar::AgentVelocity>("AgentVelocities", 1000);
    ROSSimActorSub = n.subscribe("AgentStates", 1000, &ROSSimActor::ReceiveAgentStates, this);

    std::stringstream inpTopicName;
    inpTopicName << "AgentInput_" << id;
    std::stringstream outTopicName;
    outTopicName << "AgentOutput_" << id;

    ROSSimActorToAgentPub = n.advertise<ORCAStar::ORCAInput>(inpTopicName.str(), 1000);
    ROSSimActorToAgentSub = n.subscribe(outTopicName.str(), 1000, &ROSSimActor::TransmitAgentVelocity, this);

    ROS_DEBUG("ROS Actor Init: %lu!", id);

    ros::spin();
}



ROSSimActor::~ROSSimActor(){}



void ROSSimActor::ReceiveAgentStates(const ORCAStar::AgentState &msg)
{
    ROS_DEBUG("Actor %lu start step", id);
    inputORCAMsg.pos = msg.pos[id];
    inputORCAMsg.vel = msg.vel[id];

    Point curr = {msg.pos[id].x, msg.pos[id].y};

    inputORCAMsg.neighbours.pos.clear();
    inputORCAMsg.neighbours.vel.clear();
    inputORCAMsg.neighbours.rad.clear();

    for(size_t i = 0; i < msg.pos.size(); i++)
    {
        Point another = {msg.pos[i].x, msg.pos[i].y};

        if(id != i && (curr-another).SquaredEuclideanNorm() < sightRad * sightRad)
        {
            inputORCAMsg.neighbours.pos.push_back(msg.pos[i]);
            inputORCAMsg.neighbours.vel.push_back(msg.vel[i]);
            inputORCAMsg.neighbours.rad.push_back(msg.rad[i]);
        }
    }


    ROSSimActorToAgentPub.publish(inputORCAMsg);
}

void ROSSimActor::TransmitAgentVelocity(const geometry_msgs::Point32 &msg)
{
    ROS_DEBUG("Actor %lu end step", id);

    agentVelocityMsg.id = id;
    agentVelocityMsg.vel = msg;
    ROSSimActorPub.publish(agentVelocityMsg);
}


bool ROSSimActor::InitAgent(ORCAStar::Init::Request &req, ORCAStar::Init::Response &res)
{
    res = initData;

    ROS_DEBUG("Sim Agent %lu\n Init", id);

    initFlag = true;

    return true;
}

