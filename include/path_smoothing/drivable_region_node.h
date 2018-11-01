//
// Created by yangt on 1/19/18.
//

#ifndef INCLUDE_PATH_SMOOTHING_DRIVABLE_REGION_NODE_H
#define INCLUDE_PATH_SMOOTHING_DRIVABLE_REGION_NODE_H

#include <ros/ros.h>
#include <string>
#include <lanelet_map_msgs/LaneletMap.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <anm_msgs/VehicleState.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <interactive_markers/interactive_marker_server.h>
#include <opt_utils/opt_utils.hpp>
#include <dynamic_reconfigure/server.h>
#include <path_smoothing/smoothConfig.h>
//#include <astar_planner/astar_search.h>
//#include <astar_planner/search_info_ros.h>
class DrivableMap {
 public:
    explicit DrivableMap(const ros::NodeHandle &nh,
                       std::string node_name = "path_smoothing_node");
    ~DrivableMap(){};

 private:
    void initialize();
    void readParameters();
    void laneletMapCb(const lanelet_map_msgs::LaneletMapConstPtr &lanelet);
    
    void vehicleStateCb(const anm_msgs::VehicleStateConstPtr &state);

    void globalPathCb(const nav_msgs::PathConstPtr &path);

    void goalCb(const geometry_msgs::PoseStamped &pose);

    void timerCb();

    void extractDrivableRegion(const lanelet_map_msgs::LaneletMapConstPtr &lanlet_map);

    void polygonExtractionCV(const grid_map::Polygon &polygon);

    void clearPolygonMapRegion(const lanelet_map_msgs::LaneletMapConstPtr &lanelet_map);

    void clearSingleLaneletRegion(const lanelet_map_msgs::Lanelet &lanelet);

    void wayRefine(const nav_msgs::Path &path, std::vector<grid_map::Position> *way);

    void updateSignDistanceField();

    ros::NodeHandle nh_;
    ros::Subscriber vehicle_state_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber laneletmap_sub_;
    ros::Subscriber global_path_sub_;
    ros::Timer timer_;
    std::string laneletmap_topic_{"/lanelet_mapserver_node/current_map_msg"};
    nav_msgs::Path global_path_;
    std::vector<geometry_msgs::Point> origin_path_;

    ros::Publisher poly_pub_;
    ros::Publisher path_pub_;
    ros::Publisher ogm_pub_;
    ros::Publisher point_cloud_;
    ros::Publisher rrt_path_pub_;
    hmpl::InternalGridMap internal_grid_map_;
    hmpl::InternalGridMap reverse_map_;
    anm_msgs::VehicleState vehicle_state_;

    lanelet_map_msgs::Lanelet last_lanelet_{};
    lanelet_map_msgs::Lanelet current_lanelet_{};

    geometry_msgs::Pose goal_pose_;


    bool is_save_map_;
    bool is_consider_boundary_;
    bool is_set_goal_;
    bool is_locate_;

    bool use_self_solver_;
    double w1_;
    double w2_;
    double w3_;

    // obstacle server related
    std::vector<hmpl::Circle> point_obstacles_;
    double circle_radius_{1.5};
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
