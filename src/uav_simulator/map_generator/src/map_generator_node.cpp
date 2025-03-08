#include "map_generator/map_generator.h"

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"map_generator_node");
    ros::NodeHandle nh("~");

    MapGenerator map_generator(nh);

    map_generator.randomMapGenerate();

    double sense_rate;
    nh.param("sensing/radius", sense_rate, 10.0);

    ros::Rate loop_rate(sense_rate);

    while (ros::ok()){
        map_generator.publishSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
    }

}