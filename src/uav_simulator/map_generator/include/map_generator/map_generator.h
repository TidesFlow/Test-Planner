#ifndef __MAP_GENERATOR_H__
#define __MAP_GENERATOR_H__

#include <random>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

class MapGenerator {
public:
    MapGenerator(ros::NodeHandle nh);
    ~MapGenerator(){};

    void randomMapGenerate(void);
    void publishSensedPoints(void);
    void process(void);
private:
    ros::Publisher _local_map_pub;
    ros::Publisher _global_map_pub;

    pcl::PointCloud<pcl::PointXYZ> cloudMap_global;
    sensor_msgs::PointCloud2 map_global;

    double _init_x,_init_y;

    double _resolution;
    double _x_size,_y_size,_z_size;
    double _x_l,_x_u,_y_l,_y_u,_w_l,_w_u,_h_l,_h_u;
    double _a_l,_a_u,_b_l,_b_u,_angle_l,_angle_u,_z_l,_z_u;
    int _obs_num,_cir_num;
};

#endif