#include "ROSSimActor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSSimActor", ros::init_options::AnonymousName);
    ROSSimActor actor = ROSSimActor();
}
