//
// Created by yangt on 1/19/18.
//

#ifndef INCLUDE_PATH_SMOOTHING_DRIVABLE_REGION_NODE_H
#define INCLUDE_PATH_SMOOTHING_DRIVABLE_REGION_NODE_H

#include <ros/ros.h>
#include <string>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <interactive_markers/interactive_marker_server.h>
#include <opt_utils/opt_utils.hpp>
#include <dynamic_reconfigure/server.h>
#include <path_smoothing/smoothConfig.h>
#include "path_smoothing/chomp.hpp"
#include <ros/package.h>

class DrivableMap {
 public:
    explicit DrivableMap(const ros::NodeHandle &nh,
                       std::string node_name = "path_smoothing_node");
    ~DrivableMap(){};

 private:
    void initialize();
    void readParameters();

    void timerCb();
    
    void updateSignDistanceField();

    ros::NodeHandle nh_;
    ros::Timer timer_;
    
    ros::Publisher path_pub_;
    ros::Publisher ogm_pub_;
    ros::Publisher point_cloud_;

    ros::Subscriber goal_pose_sub_;

    hmpl::InternalGridMap in_gm;
    hmpl::InternalGridMap reverse_map_;

    geometry_msgs::Pose goal_pose_;
    hmpl::Pose2D start_pose_;


    bool is_save_map_;
    bool is_consider_boundary_;
    bool is_set_goal_;
    bool is_locate_;

    bool use_self_solver_;
    double w1_;
    double w2_;
    double w3_;
    
    void updateGridMap();

    // callback function
    void goalPoseCb(const geometry_msgs::PoseStamped &goal);

    // obstacle server related
    std::vector<hmpl::Circle> point_obstacles_;
    double circle_radius_{5};
    int circle_obstacle_num_{5};
    
    // interactive marker
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_ptr_;
    void makeObstacleMarker(const grid_map::Position& position, int id);
    void obstacleMakerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void fillCircleObstacles(const std::vector<hmpl::Circle> &obstacles);

    //dynamic reconfigure
    dynamic_reconfigure::Server<path_smoothing::smoothConfig> server;
    void reconfigureRequest(const path_smoothing::smoothConfig &config, uint32_t level);
};


#endif //PATH_SMOOTHING_DRIVABLE_REGION_NODE_H
