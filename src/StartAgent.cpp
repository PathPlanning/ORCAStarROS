#include "ROSAgent.h"
#include "ros/ros.h"


#include <sstream>
#include <ORCAStar/ORCAInput.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ROSAgent", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    int i;
    ros::param::get("~param", i);


   ROSAgent actor = ROSAgent(i);
}
