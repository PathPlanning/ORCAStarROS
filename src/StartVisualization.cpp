/*!
\file
\brief File contains program for visualization launching.
\ingroup ORCAStarROS
*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "ORCAStar/AgentState.h"

/*!
 * \brief ROS publisher for visualization markers
 *
 */
ros::Publisher marker_pub;


/*!
 * \brief Updates visualisation every time, when message appears
 * \param msg Agents state message
 */
void Update(const ORCAStar::AgentState &msg)
{
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
    visualization_msgs::MarkerArray agents;
    agents.markers.clear();

    for(size_t i = 0; i < msg.pos.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = ros::Time::now();

        marker.ns = "basic_shapes";
        marker.id = i;

        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = msg.pos[i].x;
        marker.pose.position.y = msg.pos[i].y;
        marker.pose.position.z = 0.15;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.3;
        marker.scale.y = 2 * msg.rad[i];
        marker.scale.x = 2 * msg.rad[i];

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        agents.markers.push_back(marker);
    }
    marker_pub.publish(agents);
}



int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(10);
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

  ros::Subscriber ROSSimActorSub = n.subscribe("AgentStates", 1000, Update);

  ros::spin();
}

