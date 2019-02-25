//
// Created by yangt on 19-2-22.
//
#include <ros/ros.h>
#include <ros/package.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include "opt_utils/opt_utils.hpp"
#include "path_smoothing/path_smoothing.hpp"

class DrivableMap {
 public:
  explicit DrivableMap(const ros::NodeHandle &nh,
                       std::string node_name = "path_smoothing_node");
  std::vector<geometry_msgs::Point> rough_path;
 private:
  void timerCb();
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Publisher original_path_pub_;
  ros::Publisher smooth_path_pub_;
  ros::Publisher smooth2_path_pub_;
  ros::Publisher ogm_pub_;
  ros::Publisher point_cloud_;
  hmpl::InternalGridMap map_;
  std::string base_dir_;
  std::string sdf_layer_ = "distance_cost";
};

DrivableMap::DrivableMap(const ros::NodeHandle &nh, std::string node_name)
        : nh_(nh) {
    // init grid map
    base_dir_ = ros::package::getPath("path_smoothing");
    cv::Mat img_src = cv::imread(base_dir_ + "/demo/obstacles.png", CV_8UC1);
    double resolution = 0.1;  // in meter

    map_.initializeFromImage(img_src,
                             resolution,
                             grid_map::Position::Zero());
    map_.maps.setFrameId("map");
    map_.addObstacleLayerFromImage(img_src, 0.5);
    map_.maps.add(sdf_layer_, 0.0);
    map_.updateDistanceLayerCV();
    hmpl::InternalGridMap inveres_map;
    inveres_map.init(map_.maps.getFrameId(),
                     map_.maps.getLength(),
                     map_.maps.getResolution());
    for (grid_map::GridMapIterator it(inveres_map.maps); !it.isPastEnd();
         ++it) {
        const auto &value = map_.maps.at(map_.obs, *it);
        if (value >= map_.FREE / 2) {
            inveres_map.maps.at(map_.obs, *it) = map_.OCCUPY;
        } else {
            inveres_map.maps.at(map_.obs, *it) = map_.FREE;
        }
    }
    inveres_map.updateDistanceLayerCV();
    double th = 2.8;
    for (grid_map::GridMapIterator it(inveres_map.maps); !it.isPastEnd();
         ++it) {
        const auto cost = map_.maps.at(map_.dis, *it)
                - inveres_map.maps.at(map_.dis, *it);
        map_.maps.at(sdf_layer_, *it) = cost;
//        if (cost < 0.0) {
//            map_.maps.at(sdf_layer_, *it) = th - cost;
//        } else if (cost <= th) {
//            map_.maps.at(sdf_layer_, *it) = pow(cost - th, 2) / th;
//        } else {
//            map_.maps.at(sdf_layer_, *it) = 0;
//        }
    }

    io::CSVReader<2> in(base_dir_ + "/demo/a_star_path.csv");
    in.read_header(io::ignore_extra_column, "x", "y");
    geometry_msgs::Point pt;
    while (in.read_row(pt.x, pt.y)) {
        rough_path.push_back(pt);
    }

    // init ros subscriber and publisher
    this->original_path_pub_ = this->nh_.advertise<nav_msgs::Path>(
            "original_path", 1, true);
    this->smooth_path_pub_ = this->nh_.advertise<nav_msgs::Path>(
            "cg_smooth_path", 1, true);

    this->smooth2_path_pub_ = this->nh_.advertise<nav_msgs::Path>(
            "gp_smooth_path", 1, true);

    this->ogm_pub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>(
            "global_path/grid_map",
            1,
            true);
    this->point_cloud_ =
            this->nh_.advertise<sensor_msgs::PointCloud2>(
                    "map_point_cloud",
                    1,
                    true);
    this->timer_ =
            this->nh_.createTimer(
                    ros::Duration(1),
                    boost::bind(&DrivableMap::timerCb, this));

}

void DrivableMap::timerCb() {

    ros::Time time = ros::Time::now();
    map_.maps.setTimestamp(time.toNSec());
    nav_msgs::OccupancyGrid message;
    grid_map::GridMapRosConverter::toOccupancyGrid(map_.maps,
                                                   map_.obs,
                                                   map_.FREE,
                                                   map_.OCCUPY,
                                                   message);
    this->ogm_pub_.publish(message);
    sensor_msgs::PointCloud2 pointcloud;
    grid_map::GridMapRosConverter::toPointCloud(map_.maps,
                                                sdf_layer_,
                                                pointcloud);

    this->point_cloud_.publish(pointcloud);

    nav_msgs::Path original_path;
    nav_msgs::Path smooth_path;
    geometry_msgs::PoseStamped pose;
    original_path.header.frame_id = map_.maps.getFrameId();
    original_path.header.stamp = ros::Time::now();
    smooth_path.header = original_path.header;
    pose.header = smooth_path.header;

    for (const auto &pt:rough_path) {
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        original_path.poses.push_back(pose);
    }
    original_path_pub_.publish(original_path);

    using namespace path_smoothing;

    DistanceFunction2D dis_function(map_.maps, sdf_layer_, 2.3);
    PathSmoothing::Options options;
    options.function = &(dis_function);
    //    options.solver = SELF_SOLVER;

    /// conjugate-gradient smoothing:
    auto t1 = hmpl::now();
    PathSmoothing *smoother =
            PathSmoothing::createSmoother(options, rough_path);
    smoother->smoothPath(options);
    auto t2 = hmpl::now();
    printf("cg smooth cost: %f\n", hmpl::getDurationInSecs(t1, t2));
    std::vector<geometry_msgs::Point> path;
    smoother->getPointPath(&path);
    for (const auto &state : path) {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        smooth_path.poses.push_back(pose);
    }
    smooth_path_pub_.publish(smooth_path);

    /// Gauss Process smoothing:
    options.smoother_type = GAUSS_PROCESS_METHOD;
    t1 = hmpl::now();
    PathSmoothing *smoother2 =
            PathSmoothing::createSmoother(options, rough_path);
    smoother2->smoothPath(options);
    t2 = hmpl::now();
    printf("gp smooth cost: %f\n", hmpl::getDurationInSecs(t1, t2));
    smoother2->getPointPath(&path);
    nav_msgs::Path smooth2_path;
    smooth2_path.header = smooth_path.header;
    for (const auto &state : path) {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        smooth2_path.poses.push_back(pose);
    }
    smooth2_path_pub_.publish(smooth2_path);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_smoothing_demo_node");
    ros::NodeHandle nh("");
    DrivableMap drivable_region_node(nh, "path_smoothing_node");
    ROS_INFO("Initialized drivable region node.");

    ros::spin();
    return 0;
}
