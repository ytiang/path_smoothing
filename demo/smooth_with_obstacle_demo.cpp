//
// Created by yangt on 19-2-22.
//
#include <memory>
#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include "opt_utils/opt_utils.hpp"
#include "path_smoothing/path_smoothing.hpp"
#include "path_smoothing/smoothing_demoParameters.h"

class DrivableMap {
 public:
  explicit DrivableMap(const ros::NodeHandle &nh,
                       const ros::NodeHandle &pnh);
  std::vector<geometry_msgs::Point> rough_path;
 private:
  void timerCb();
  void reconfigureRequest(path_smoothing::smoothing_demoConfig &config,
                          uint32_t level);
  path_smoothing::smoothing_demoParameters params_;
  dynamic_reconfigure::Server<path_smoothing::smoothing_demoConfig> reconfig_;
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Publisher original_path_pub_;
  ros::Publisher smooth_path_pub_;
  ros::Publisher smooth2_path_pub_;
  ros::Publisher ogm_pub_;
  ros::Publisher point_cloud_;
  hmpl::InternalGridMap map_;
  path_smoothing::PathSmoothing::Options options_;
  double distance_threshold = 2.5;
  std::string base_dir_;
  std::string sdf_layer_ = "distance_cost";
};

DrivableMap::DrivableMap(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
        : nh_(nh), reconfig_(pnh), params_(pnh), options_() {

    printf(">>>> distance threshold: %f\n", distance_threshold);
    printf(">>>> curvature coe: %f\n", options_.cg_curvature_term_coe);
    printf(">>>> heading coe: %f\n", options_.cg_heading_term_coe);
    printf(">>>> obstacle coe: %f\n", options_.cg_obstacle_term_coe);
    printf(">>>> gp delta t: %f\n", options_.gp_dt);
    printf(">>>> gp obstacle sigma: %f\n", options_.gp_obs_sigma);
    printf(">>>> gp vehicle dynamic sigma: %f\n",
           options_.gp_vehicle_dynamic_sigma);
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
//    params_.fromParamServer();
//    reconfig_.setCallback(boost::bind(&DrivableMap::reconfigureRequest,
//                                      this,
//                                      _1,
//                                      _2));

}

void DrivableMap::reconfigureRequest(path_smoothing::smoothing_demoConfig &config,
                                     uint32_t level) {
    params_.fromConfig(config);
    options_.cg_curvature_term_coe = params_.cg_curvature_term_coe;
    options_.cg_obstacle_term_coe = params_.cg_obstacle_term_coe;
    options_.cg_heading_term_coe = params_.cg_heading_term_coe;
    options_.gp_dt = params_.gp_dt;
    options_.gp_obs_sigma = params_.gp_obs_sigma;
    options_.gp_vehicle_dynamic_sigma = params_.gp_vehicle_dynamic_sigma;
    distance_threshold = params_.distance_threshold;

    printf(">>>> distance threshold: %f\n", distance_threshold);
    printf(">>>> curvature coe: %f\n", options_.cg_curvature_term_coe);
    printf(">>>> heading coe: %f\n", options_.cg_heading_term_coe);
    printf(">>>> obstacle coe: %f\n", options_.cg_obstacle_term_coe);
    printf(">>>> gp delta t: %f\n", options_.gp_dt);
    printf(">>>> gp obstacle sigma: %f\n", options_.gp_obs_sigma);
    printf(">>>> gp vehicle dynamic sigma: %f\n",
           options_.gp_vehicle_dynamic_sigma);

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

    DistanceFunction2D dis_function(map_.maps, sdf_layer_, distance_threshold);
    options_.function = &(dis_function);
    options_.cg_solver = SELF_SOLVER;

    /// conjugate-gradient smoothing:
    options_.smoother_type = CONJUGATE_GRADIENT_METHOD;
    auto t1 = hmpl::now();
    std::unique_ptr<PathSmoothing>
            smoother(PathSmoothing::createSmoother(options_, rough_path));
    smoother->smoothPath(options_);
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
    options_.smoother_type = GAUSS_PROCESS_METHOD;
    t1 = hmpl::now();
    std::unique_ptr<PathSmoothing>
            smoother2(PathSmoothing::createSmoother(options_, rough_path));
    smoother2->smoothPath(options_);
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
    ros::NodeHandle pnh("~");
    DrivableMap drivable_region_node(nh, pnh);
    ROS_INFO("Initialized drivable region node.");

    ros::spin();
    return 0;
}
