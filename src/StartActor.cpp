/*!
\file
\brief File contains program for single simulation actor launching.
*/

#include "ROSSimActor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSSimActor", ros::init_options::AnonymousName);
    ROSSimActor actor = ROSSimActor();
}
