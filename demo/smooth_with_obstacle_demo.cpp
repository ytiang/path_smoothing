//
// Created by yangt on 19-2-22.
//
#include <memory>
#include <cv.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include "path_smoothing/path_smoothing.hpp"
#include "path_smoothing/smoothing_demoParameters.h"
#include "csv_reader.hpp"

class DrivableMap {
 public:
    explicit DrivableMap(const ros::NodeHandle &nh,
                         const ros::NodeHandle &pnh);
    std::vector<geometry_msgs::Point> rough_path;
 private:
    void initializeMap();
    void showDistanceField(const DistanceFunction2D &dis_func);
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
    grid_map::GridMap map_;
    path_smoothing::PathSmoothing::Options options_;
    double distance_threshold = 3.0;
    std::string base_dir_;
    std::string sdf_layer_ = "singed_distance";
    std::string obs_layer_ = "obstacle";
    std::string dis_layer_ = "distance";
};

DrivableMap::DrivableMap(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh),
      reconfig_(pnh),
      params_(pnh),
      options_() {

    printf(">>>> distance threshold: %f\n", distance_threshold);
    printf(">>>> curvature coe: %f\n", options_.cg_curvature_term_coe);
    printf(">>>> heading coe: %f\n", options_.cg_heading_term_coe);
    printf(">>>> obstacle coe: %f\n", options_.cg_obstacle_term_coe);
    printf(">>>> gp delta t: %f\n", options_.gp_dt);
    printf(">>>> gp obstacle sigma: %f\n", options_.gp_obs_sigma);
    printf(">>>> gp vehicle dynamic sigma: %f\n",
           options_.gp_vehicle_dynamic_sigma);
    base_dir_ = ros::package::getPath("path_smoothing");

    // init grid map
    this->initializeMap();
    // read rough path:
    io::CSVReader<2> in(base_dir_ + "/demo/rough_path.csv");
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
    reconfig_.setCallback(boost::bind(&DrivableMap::reconfigureRequest,
                                      this,
                                      _1,
                                      _2));

}

void DrivableMap::initializeMap() {
    cv::Mat img_src = cv::imread(base_dir_ + "/demo/obstacles.png", CV_8UC1);
    cv::Mat obs_dis, inverse_obs_dis;
    cv::distanceTransform(img_src, obs_dis, CV_DIST_L2, CV_DIST_MASK_PRECISE);
    cv::distanceTransform(~img_src,
                          inverse_obs_dis,
                          CV_DIST_L2,
                          CV_DIST_MASK_PRECISE);
    double resolution = 0.1;  // in meter
    grid_map::GridMapCvConverter::initializeFromImage(img_src,
                                                      resolution,
                                                      this->map_,
                                                      grid_map::Position::Zero());
    this->map_.setFrameId("map");
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(img_src,
                                                                      obs_layer_,
                                                                      this->map_);
    this->map_.add(dis_layer_);
    this->map_.add(sdf_layer_);
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
        const auto &dis = obs_dis.at<float>((*it)(0), (*it)(1));
        const auto &inverse_dis = inverse_obs_dis.at<float>((*it)(0), (*it)(1));
        this->map_.at(dis_layer_, *it) = dis * map_.getResolution();
        this->map_.at(sdf_layer_, *it) = (dis - inverse_dis) * map_.getResolution();
    }
}

void DrivableMap::showDistanceField(const DistanceFunction2D &dis_func) {
    sensor_msgs::PointCloud2 pcl_msg;
    grid_map::GridMap cost_map({"cost"});
    cost_map.setGeometry(map_.getLength(),
                         map_.getResolution(),
                         map_.getPosition());
    cost_map.setFrameId(map_.getFrameId());
    for(grid_map::GridMapIterator it(cost_map); !it.isPastEnd(); ++it) {
        grid_map::Position pos;
        cost_map.getPosition(*it, pos);
        cost_map.at("cost", *it) = dis_func.cost(pos(0), pos(1));
    }
    grid_map::GridMapRosConverter::toPointCloud(cost_map,
                                                "cost",
                                                pcl_msg);

    this->point_cloud_.publish(pcl_msg);
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
    if (params_.cg_solver_type == params_.cg_solver_type_Ceres) {
        options_.cg_solver = NonlinearSolverType::CERES_SOLVER;
        printf(">>>> cg nonlinear solver type : Ceres\n");
    } else if (params_.cg_solver_type == params_.cg_solver_type_Self) {
        options_.cg_solver = NonlinearSolverType::SELF_SOLVER;
        printf(">>>> cg nonlinear solver type : Self\n");
    }

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
    this->map_.setTimestamp(time.toNSec());
    //publish grid_map;
    nav_msgs::OccupancyGrid message;
    grid_map::GridMapRosConverter::toOccupancyGrid(map_,
                                                   obs_layer_,
                                                   1,
                                                   0,
                                                   message);
    this->ogm_pub_.publish(message);


    nav_msgs::Path original_path;
    nav_msgs::Path smooth_path;
    geometry_msgs::PoseStamped pose;
    original_path.header.frame_id = map_.getFrameId();
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
    DistanceFunction2D dis_function(map_, sdf_layer_, distance_threshold);
    showDistanceField(dis_function);
    options_.function = &(dis_function);
//    options_.cg_solver = SELF_SOLVER;

    options_.smoother_type = CONJUGATE_GRADIENT_METHOD;

    std::unique_ptr<PathSmoothing>
        smoother(PathSmoothing::createSmoother(options_, rough_path));
    smoother->smoothPath(options_);

    std::vector<geometry_msgs::Point> path;
    smoother->getSmoothPath(&path);
    for (const auto &state : path) {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        smooth_path.poses.push_back(pose);
    }
    smooth_path_pub_.publish(smooth_path);

    /// Gauss Process smoothing:
#ifdef GPMP2_SMOOTHING_ENABLE
    options_.smoother_type = GAUSS_PROCESS_METHOD;
    std::unique_ptr<PathSmoothing>
            smoother2(PathSmoothing::createSmoother(options_, rough_path));
    smoother2->smoothPath(options_);
    printf("    gp parameters: obs: %f, dynamic: %f, dt: %f\n",
           options_.gp_obs_sigma,
           options_.gp_vehicle_dynamic_sigma,
           options_.gp_dt);
    smoother2->getSmoothPath(&path);
    nav_msgs::Path smooth2_path;
    smooth2_path.header = smooth_path.header;
    for (const auto &state : path) {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        smooth2_path.poses.push_back(pose);
    }
    smooth2_path_pub_.publish(smooth2_path);
#endif
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
