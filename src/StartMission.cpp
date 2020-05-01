#include "ROSMission.h"

// TODO step threshhold
// TODO filepath param
// TODO simulation results as file
// TODO summary

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSMission");

    int agNum;
    ros::param::get("~agents_number", agNum);

    ROSMission task = ROSMission("../examples/0_task.xml", agNum);
    if(!task.PrepareSimulation())
    {
        ROS_ERROR("File problem!");
        ros::shutdown();
    }

    task.StartSimulation();
}
