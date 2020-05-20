/*!
\file
\brief File contains ROSAgent class implementation.
*/


#include "ROSAgent.h"

ROSAgent::ROSAgent(size_t id)
{
    agentId = id;
    std::stringstream initServerName;
    initServerName << "initAgentServer_" << id;
    ros::ServiceClient client = n.serviceClient<ORCAStar::Init>(initServerName.str());
    ros::service::waitForService(initServerName.str());
    ORCAStar::Init srv;

    if (client.call(srv))
    {
        client.shutdown();
        ROS_DEBUG("Agent %lu Parameters received!", id);
    }
    else
    {
        ROS_ERROR("Agent %lu Parameters not received!", id);
        exit(-1);
    }

    options = new EnvironmentOptions(CN_DEFAULT_METRIC_TYPE, CN_DEFAULT_BREAKINGTIES,
                                     CN_DEFAULT_ALLOWSQUEEZE, CN_DEFAULT_CUTCORNERS,
                                     CN_DEFAULT_HWEIGHT, CN_DEFAULT_TIME_STEP, srv.response.delta);

    param = new AgentParam(srv.response.sightRadius,srv.response.timeBoundary,srv.response.timeBoundaryObst,
                           srv.response.radius, CN_DEFAULT_REPS, srv.response.speed, srv.response.agentMaxNum);

    Point start = {srv.response.start.x, srv.response.start.y};
    Point goal = {srv.response.goal.x, srv.response.goal.y};


    std::vector<std::vector<Point>> obstacles;

    for(auto &obst : srv.response.Obstacles)
    {
        std::vector<Point> tmp;
        for(auto &vert : obst.points)
        {
            tmp.push_back({vert.x, vert.y});
        }
        obstacles.push_back(tmp);
    }


    size_t w, h;
    float cs;
    std::vector<std::vector<int>> grid;

    ros::ServiceClient mapClint = n.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap mapSrv;

    if(mapClint.call(mapSrv))
    {
        mapClint.shutdown();
        ROS_DEBUG("Agent %lu Map received!", id);
    }
    else
    {
        ROS_ERROR("Agent %lu Map not received!", id);
        exit(-1);
    }

    w = mapSrv.response.map.info.width;
    h = mapSrv.response.map.info.height;
    cs = mapSrv.response.map.info.resolution;

    for(size_t i = 0; i < h; i++)
    {
        grid.push_back(std::vector<int>(w, 0));
        for(size_t j = 0; j < w; j++)
        {
            if(mapSrv.response.map.data[j + w * (h - i - 1)] == 100)
            {
                grid[i][j] = 1;
            }
            else
            {
                grid[i][j] = 0;
            }
        }
    }
    map = new Map(cs, grid, obstacles);


    ROS_DEBUG("Map %lu Created!", id);

    agent = new ORCAAgent(0, start, goal, *map, *options, *param);
    agent->SetPlanner(ThetaStar(*map, *options, agent->GetStart(), agent->GetGoal(), param->radius + param->rEps));
    agent->InitPath();

    ROS_DEBUG("Agent %lu Created!", id);

    std::stringstream inpTopicName;
    inpTopicName << "AgentInput_" << id;
    std::stringstream outTopicName;
    outTopicName << "AgentOutput_" << id;

    ROSAgentSub = n.subscribe(inpTopicName.str(), 1000, &ROSAgent::DoStep, this);;
    ROSAgentPub = n.advertise<geometry_msgs::Point32>(outTopicName.str(), 1000);;


    ROS_DEBUG("Agent %lu Topics Created!", id);

    ros::spin();

}




void ROSAgent::DoStep(const ORCAStar::ORCAInput &msg)
{


    agent->SetPosition({msg.pos.x, msg.pos.y});
    agent->SetVelocity({msg.vel.x, msg.vel.y});
    Map emptymap = Map();;

    for(size_t i = 0; i < msg.neighbours.pos.size(); i++)
    {
        AgentParam nParam = *param;
        nParam.radius = msg.neighbours.rad[i];
        Agent *tmpAgent = new ORCAAgent(static_cast<int>(i + 1),Point(), Point(), emptymap, *options, nParam );
        Point nPos = {msg.neighbours.pos[i].x, msg.neighbours.pos[i].y};
        Point nVel = {msg.neighbours.vel[i].x, msg.neighbours.vel[i].y};

        tmpAgent->SetPosition(nPos);
        tmpAgent->SetVelocity(nVel);

        float distSq = (agent->GetPosition()-nPos).SquaredEuclideanNorm();
        agent->AddNeighbour(*tmpAgent, distSq);
    }

    agent->UpdatePrefVelocity();
    agent->UpdateNeighbourObst();


    agent->ComputeNewVelocity();
    agent->ApplyNewVelocity();


    geometry_msgs::Point32 vel;
    vel.x = agent->GetVelocity().X();
    vel.y = agent->GetVelocity().Y();

    for(auto &n : neighbous)
    {
        delete n;
    }

    neighbous.clear();

    ROS_DEBUG("Agent %lu Do Step!", agentId);
    ROSAgentPub.publish(vel);
}
