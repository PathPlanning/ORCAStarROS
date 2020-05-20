/*!
\file
\brief File contains program for single agent navigation node launching.
*/


#include "ROSAgent.h"
#include "ros/ros.h"


#include <sstream>
#include <ORCAStar/ORCAInput.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ROSAgent", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    int i;
    ros::param::get("~id", i);


   ROSAgent actor = ROSAgent(i);
}
