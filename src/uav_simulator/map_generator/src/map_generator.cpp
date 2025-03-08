#include "map_generator/map_generator.h"

MapGenerator::MapGenerator(ros::NodeHandle nh)
{
    _local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud",1);
    _global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud",1);

    nh.param("init_state_x", _init_x, 0.0);
    nh.param("init_state_y", _init_y, 0.0);

    nh.param("map/resolution", _resolution, 0.1);

    nh.param("map/obs_num", _obs_num, 30);

    nh.param("map/x_size", _x_size, 50.0);
    nh.param("map/y_size", _y_size, 50.0);
        
    nh.param("ObstacleShape/lower_rad", _w_l, 0.3);
    nh.param("ObstacleShape/upper_rad", _w_u, 0.8);
    nh.param("ObstacleShape/lower_hei", _h_l, 3.0);
    nh.param("ObstacleShape/upper_hei", _h_u, 7.0);

    nh.param("map/circle_num", _cir_num, 30);

    nh.param("ObstacleShape/a_l", _a_l, 7.0);
    nh.param("ObstacleShape/a_h", _a_u, 7.0);
    nh.param("ObstacleShape/b_l", _b_l, 7.0);
    nh.param("ObstacleShape/b_h", _b_u, 7.0);

    nh.param("ObstacleShape/z_l", _z_l, 7.0);
    nh.param("ObstacleShape/z_h", _z_u, 7.0);
    nh.param("ObstacleShape/theta_l", _angle_l, 7.0);
    nh.param("ObstacleShape/theta_j", _angle_u, 7.0);

    _x_l = -_x_size / 2.0;
    _x_u = +_x_size / 2.0;

    _y_l = -_y_size / 2.0;
    _y_u = +_y_size / 2.0;

}

void MapGenerator::randomMapGenerate(void)
{
    std::random_device rd;
    std::default_random_engine eng(rd());

    std::uniform_real_distribution<double> rand_x(_x_l,_x_u);
    std::uniform_real_distribution<double> rand_y(_y_l,_y_u);
    std::uniform_real_distribution<double> rand_w(_w_l,_w_u);
    std::uniform_real_distribution<double> rand_h(_h_l,_h_u);

    std::uniform_real_distribution<double> rand_a(_a_l,_a_u);
    std::uniform_real_distribution<double> rand_b(_b_l,_b_u);
    std::uniform_real_distribution<double> rand_angel(_angle_l,_angle_u);
    std::uniform_real_distribution<double> rand_z(_z_l,_z_u);

    pcl::PointXYZ point;

    for(int i = 0;i < _obs_num;i++){
        double x = rand_x(eng);
        double y = rand_y(eng);

        if(std::sqrt(std::pow(_init_x - x,2) + std::pow(_init_y - y,2) < 2.0)){
            i--;
            continue;
        }

        x = floor(x/_resolution) * _resolution + 0.5 * _resolution;
        y = floor(y/_resolution) * _resolution + 0.5 * _resolution;

        double w = rand_w(eng);
        int wid_num = floor(w/_resolution);

        for(int r = -wid_num/2;r < wid_num/2;r++){
            for(int s = -wid_num/2;s < wid_num/2;s++){
                double h = rand_h(eng);
                int hei_num = floor(h/_resolution);
                for(int t = 0;t < hei_num;t++){
                    point.x = x + (r + 0.5) * _resolution + 1e-2;
                    point.y = y + (s + 0.5) * _resolution + 1e-2;
                    point.z = (t + 0.5)* _resolution + 1e-2;

                    cloudMap_global.push_back(point);
                }

            }
        }
    }

    for(int i = 0;i < _cir_num;i++){
        double x = rand_x(eng);
        double y = rand_y(eng);
        double z = rand_z(eng);

        if(std::sqrt(std::pow(_init_x - x,2) + std::pow(_init_x - x,2) < 2.0)){
            i--;
            continue;
        }

        x = floor(x/_resolution) * _resolution + 0.5 * _resolution;
        y = floor(y/_resolution) * _resolution + 0.5 * _resolution;
        z = floor(z/_resolution) * _resolution + 0.5 * _resolution;

        Eigen::Vector3d translate(x,y,z);

        double angle = rand_angel(eng);
        Eigen::Matrix3d rotation;

        rotation << cos(angle), -sin(angle), 0.0, 
                    sin(angle), cos(angle), 0.0, 
                    0, 0, 1;

        double a = rand_a(eng);
        double b = rand_b(eng);

        Eigen::Vector3d circle_point;

        for(double theta = 0;theta < 6.282;theta += _resolution){
            circle_point(0) = 0.0;
            circle_point(1) = a * cos(theta);
            circle_point(2) = b * sin(theta);

            Eigen::Vector3d new_point;
            
            new_point = rotation * circle_point + translate;
            point.x = new_point(0);
            point.y = new_point(1);
            point.z = new_point(2);

            cloudMap_global.push_back(point);
        }
    }

    cloudMap_global.width = cloudMap_global.points.size();
    cloudMap_global.height = 1;
    cloudMap_global.is_dense = true;

    ROS_WARN("Finish Generating Random Map");
}

void MapGenerator::publishSensedPoints(void)
{
    pcl::toROSMsg(cloudMap_global,map_global);

    map_global.header.frame_id = "world";
    _global_map_pub.publish(map_global);
}

void MapGenerator::process(void)
{
    publishSensedPoints();
}