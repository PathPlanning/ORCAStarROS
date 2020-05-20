/*!
\file
\brief File contains program for multiagent simulation launching.
*/


#include "ROSMission.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSMission");

    int agNum;
    std::string file;
    int t = -1;
    bool end = false;

    ros::param::get("~agents_number", agNum);
    ros::param::get("~task", file);
    ros::param::get("~threshhold", t);
    ros::param::get("~end", end);


    ROSMission task = ROSMission(file, agNum, t, end);
    if(!task.PrepareSimulation())
    {
        ROS_ERROR("File problem!");
        ros::shutdown();
    }

    task.StartSimulation();
}
