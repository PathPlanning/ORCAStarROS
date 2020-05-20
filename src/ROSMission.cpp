/*!
\file
\brief File contains ROSMission class implementation.
*/


#include "ROSMission.h"


ROSMission::ROSMission(std::string fileName, size_t agNum, int threashold , bool endOnFin)
{
    ROS_DEBUG("Mission Init!");
    taskReader = new XMLReader(fileName);
    agNumber = agNum;
    agCount = 0;
    agents = std::vector<Agent*>();

    stepsCount = 0;
    stepsTreshhold = threashold;
    endOnFinish = endOnFin;

    options = nullptr;
    missionResult = Summary();
    resultsLog = std::unordered_map<int, std::pair<bool, int>>();
    resultsLog.reserve(agNumber);

    stepsCount = 0;

    agentStateMsg.pos = std::vector<geometry_msgs::Point32>();
    agentStateMsg.vel = std::vector<geometry_msgs::Point32>();
    agentStateMsg.rad = std::vector<float> ();

    ROSMissionPub = n.advertise<ORCAStar::AgentState>("AgentStates", 1000);
    loopRate = new ros::Rate(10);

    ROSMissionSub = n.subscribe("AgentVelocities", 1000, &ROSMission::UpdateVelocity, this);
}


ROSMission::~ROSMission()
{
   delete loopRate;
   delete taskReader;
}



void ROSMission::StartSimulation()
{

    ROS_DEBUG("Mission Start!");

    while (ros::ok() && !IsFinished())
    {
        ROS_DEBUG("Mission step!");

        UpdateState();
        GenerateAgentStateMsg();
        ROSMissionPub.publish(agentStateMsg);

        ros::spinOnce();
        stepsCount++;

        loopRate->sleep();

    }
    ROS_DEBUG("Mission End!");
}



void ROSMission::GenerateAgentStateMsg()
{
    geometry_msgs::Point32 tmp;
    int i = 0;

    for(auto &a : agents)
    {
        tmp.x = a->GetPosition().X();
        tmp.y = a->GetPosition().Y();
        agentStateMsg.pos[i] = tmp;

        tmp.x = a->GetVelocity().X();
        tmp.y = a->GetVelocity().Y();
        agentStateMsg.vel[i] = tmp;

        agentStateMsg.rad[i] = a->GetRadius();
        i++;
    }
}


void ROSMission::UpdateVelocity(const ORCAStar::AgentVelocity &msg)
{
  size_t id = msg.id;
  ROS_DEBUG("Agent %lu Update Velocity", id);
  agents[id]->SetVelocity(Point(msg.vel.x, msg.vel.y));
}

void ROSMission::UpdateState()
{
    for(auto &a : agents)
    {
        Point vel = a->GetVelocity();
        Point newPos = a->GetPosition() + vel * options->timestep;
        a->SetPosition(newPos);
    }
}


bool ROSMission::PrepareSimulation()
{

    if(!ReadTask()) return false;

    ROS_DEBUG("Task File is OK!");

    agentStateMsg.pos.resize(agNumber);
    agentStateMsg.vel.resize(agNumber);
    agentStateMsg.rad.resize(agNumber);

    ros::ServiceServer initServer = n.advertiseService("initServer", &ROSMission::InitAgent, this);
    initFlag = false;
    while(!initFlag && ros::ok())
    {
        ros::spinOnce();
    }

    ROS_DEBUG("Agents is OK!");
    agCount = 0;

    return true;
}



bool ROSMission::ReadTask()
{
    return taskReader->ReadData() && taskReader->GetMap(&map) &&
           taskReader->GetAgents(agents, agNumber) &&
           taskReader->GetEnvironmentOptions(&options);
}



bool ROSMission::InitAgent(ORCAStar::Init::Request  &req, ORCAStar::Init::Response &res)
{
    res.id = agCount;

    res.start.x = agents[agCount]->GetStart().X();
    res.start.y = agents[agCount]->GetStart().Y();
    res.goal.x = agents[agCount]->GetGoal().X();
    res.goal.y = agents[agCount]->GetGoal().Y();

    res.delta = options->delta;
    res.speed = agents[agCount]->GetParam().maxSpeed;
    res.radius = agents[agCount]->GetParam().radius;
    res.agentMaxNum = agents[agCount]->GetParam().agentsMaxNum;
    res.sightRadius = agents[agCount]->GetParam().sightRadius;
    res.timeBoundary = agents[agCount]->GetParam().timeBoundary;
    res.timeBoundaryObst = agents[agCount]->GetParam().timeBoundaryObst;

    for(auto &O : map->GetObstacles())
    {
        geometry_msgs::Polygon polygon;
        for(auto &o : O)
        {
            geometry_msgs::Point32 vertex;
            vertex.x = o.left.X();
            vertex.y = o.left.Y();
            polygon.points.push_back(vertex);
        }

        res.Obstacles.push_back(polygon);
    }

    ROS_DEBUG("Sim Actor %lu\n", agCount);

    agCount++;
    if(agCount == agNumber)
    {
        initFlag = true;
    }

    return true;
}



bool ROSMission::IsFinished()
{
    if(!endOnFinish)
    {
        return false;
    }

    if(stepsCount == stepsTreshhold)
    {
        return true;
    }

    bool result = true;
    for(auto &agent : agents)
    {
        result = result && agent->isFinished();
        if(!result)
            break;
    }

    return result;
}
