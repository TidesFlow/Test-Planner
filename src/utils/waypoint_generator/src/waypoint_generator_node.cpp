#include "waypoint_generator/waypoint_generator.h"

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"waypoint_generator");
    ros::NodeHandle nh("~");

    WaypointGenerator waypoint_generator;
    waypoint_generator.init(nh);

    ros::spin();

    return 0;
}