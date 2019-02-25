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

#include <gpmp2/obstacle/PlanarSDF.h>
#include <gpmp2/kinematics/Pose2MobileBaseModel.h>
//#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h>
//#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h>
#include <path_smoothing/obstacle_factor.hpp>
#include <gpmp2/dynamics/VehicleDynamicsFactorPose2.h>
#include <gpmp2/gp/GaussianProcessPriorPose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

#include "opt_utils/opt_utils.hpp"
#include "internal_grid_map/internal_grid_map.hpp"

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
  ros::Publisher ogm_pub_;
  ros::Publisher point_cloud_;
  hmpl::InternalGridMap map_;
  gpmp2::PlanarSDF sdf_;
  std::string base_dir_;
  std::string sdf_layer_ = "distance_cost";
};

DrivableMap::DrivableMap(const ros::NodeHandle &nh, std::string node_name)
        : nh_(nh) {
    // init grid map
    base_dir_ = ros::package::getPath("path_smoothing");
    cv::Mat img_src = cv::imread(base_dir_ + "/test/obstacles.png", CV_8UC1);
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
    for (grid_map::GridMapIterator it(inveres_map.maps); !it.isPastEnd();
         ++it) {
        map_.maps.at(sdf_layer_, *it) = map_.maps.at(map_.dis, *it)
                - inveres_map.maps.at(map_.dis, *it);
    }
    gtsam::Vector2 origin;
    origin << (-map_.maps.getLength().x() / 2), (-map_.maps.getLength().y()
            / 2);
    Eigen::MatrixXd data(map_.maps.getSize()(0), map_.maps.getSize()(1));
    for (grid_map::GridMapIterator it(map_.maps); !it.isPastEnd(); ++it) {
        const int x = map_.maps.getSize()(0) - (*it)(0) - 1;
        const int y = map_.maps.getSize()(1) - (*it)(1) - 1;
        data(x, y) = map_.maps.at(sdf_layer_, *it);
    }
    sdf_ = gpmp2::PlanarSDF(gtsam::Point2(origin),
                            map_.maps.getResolution(),
                            data);

    io::CSVReader<2> in(base_dir_ + "/test/a_star_path.csv");
    in.read_header(io::ignore_extra_column, "x", "y");
    geometry_msgs::Point pt;
    while (in.read_row(pt.x, pt.y)) {
        rough_path.push_back(pt);
    }

    // init ros subscriber and publisher
    this->original_path_pub_ = this->nh_.advertise<nav_msgs::Path>(
            "original_global_path", 1, true);
    this->smooth_path_pub_ = this->nh_.advertise<nav_msgs::Path>(
            "smooth_global_path", 1, true);

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

    /// gpmp2 solver:
    double dt = 0.5;
    const int size = rough_path.size();
    double total_time = dt * size;
    gtsam::Vector3 origin;
    origin << 0.0, 0.0, 0.0;
    gpmp2::BodySphere sphere(0, 1.2, gtsam::Point3(origin));
    gpmp2::BodySphereVector sphere_vec;
    sphere_vec.push_back(sphere);
    gpmp2::Pose2MobileBaseModel robot(gpmp2::Pose2MobileBase(), sphere_vec);
    auto Qc_model =
            gtsam::noiseModel::Gaussian::Covariance(Eigen::Matrix3d::Identity());
    auto pose_fix = gtsam::noiseModel::Isotropic::Sigma(robot.dof(), 0.0001);
    auto vel_fix = gtsam::noiseModel::Isotropic::Sigma(robot.dof(), 0.0001);
    const double cost_sigma = 0.01;
    const double epsilon_dist = 0.1;

    DistanceFunction2D dis_function(map_.maps, sdf_layer_, 2.5);

    //1. set initial values:
    gtsam::Vector avg_vel(3);
    avg_vel << rough_path.back().x - rough_path.front().x,
            rough_path.back().y - rough_path.front().y, 0;
    avg_vel = avg_vel / total_time;
    gtsam::Values initial_guess;
    for (int i = 0; i < size; ++i) {
        gtsam::Key pose_key = i;
        gtsam::Key vel_key = i + size;
        double heading;
        if (i == size - 1) {
            heading = atan2(rough_path.at(i).y - rough_path.at(i - 1).y,
                            rough_path.at(i).x - rough_path.at(i - 1).x);
        } else {
            heading = atan2(rough_path.at(i + 1).y - rough_path.at(i).y,
                            rough_path.at(i + 1).x - rough_path.at(i).x);
        }
        initial_guess.insert(pose_key,
                             gtsam::Pose2(rough_path.at(i).x,
                                          rough_path.at(i).y,
                                          heading));
        initial_guess.insert(vel_key, avg_vel);
    }
    // build graph
    gtsam::NonlinearFactorGraph graph;

    for (int i = 0; i < size; ++i) {
        gtsam::Key pose_key = i;
        gtsam::Key vel_key = i + size;
        // start and end prior:
        if (i == 0) {
            double heading = atan2(rough_path.at(i + 1).y - rough_path.at(i).y,
                                   rough_path.at(i + 1).x - rough_path.at(i).x);
            gtsam::Pose2 start_pose
                    (rough_path.front().x, rough_path.front().y, heading);
            graph.add(gtsam::PriorFactor<gtsam::Pose2>(pose_key,
                                                       start_pose,
                                                       pose_fix));
            graph.add(gtsam::PriorFactor<gtsam::Vector>(vel_key,
                                                        gtsam::Vector::Zero(3),
                                                        vel_fix));

        } else if (i == size - 1) {
            double heading = atan2(rough_path.at(i).y - rough_path.at(i - 3).y,
                                   rough_path.at(i).x - rough_path.at(i - 3).x);
            gtsam::Pose2 end_pose
                    (rough_path.back().x, rough_path.back().y, heading);
            graph.add(gtsam::PriorFactor<gtsam::Pose2>(pose_key,
                                                       end_pose,
                                                       pose_fix));
            graph.add(gtsam::PriorFactor<gtsam::Vector>(vel_key,
                                                        gtsam::Vector::Zero(3),
                                                        vel_fix));
        }
        // cost factor
        graph.add(gpmp2::ObstacleFactor<gpmp2::Pose2MobileBaseModel>(pose_key,
                                                                     robot,
                                                                     dis_function,
                                                                     cost_sigma,
                                                                     epsilon_dist));
        // vehicle dynamic:
        graph.add(gpmp2::VehicleDynamicsFactorPose2(pose_key, vel_key, 0.01));

        if (i > 0) {
            gtsam::Key pose_key0 = pose_key - 1;
            gtsam::Key vel_key0 = vel_key - 1;

            //GP priors
            graph.add(gpmp2::GaussianProcessPriorPose2(pose_key0,
                                                       vel_key0,
                                                       pose_key,
                                                       vel_key,
                                                       dt,
                                                       Qc_model));

//            //obstacle
//            for (int j = 0; j < 5; ++j) {
//                const double tau = (j + 1) * total_time / 6 / size;
//                graph.add(gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase(
//                        pose_key0,
//                        vel_key0,
//                        pose_key,
//                        vel_key,
//                        robot,
//                        sdf_,
//                        cost_sigma,
//                        epsilon_dist,
//                        Qc_model,
//                        dt,
//                        tau));
//            }
        }
    }

    // optimize
//    gtsam::GaussNewtonParams params;
//    params.setVerbosity("ERROR");
//    gtsam::GaussNewtonOptimizer optimizer(graph, initial_guess, params);

    auto t1 = hmpl::now();
//    gtsam::LevenbergMarquardtParams params;
//    params.setVerbosity("ERROR");
//    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_guess, params);

    gtsam::DoglegParams params;
    params.setVerbosity("ERROR");
    gtsam::DoglegOptimizer optimizer(graph, initial_guess, params);

    optimizer.optimize();
    auto result = optimizer.values();
    auto t2 = hmpl::now();

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

    printf("smooth cost: %f\n", hmpl::getDurationInSecs(t1, t2));
    for (int i = 0; i < size; ++i) {
        const auto &state = result.at<gtsam::Pose2>(i);
        pose.pose.position.x = state.x();
        pose.pose.position.y = state.y();
        smooth_path.poses.push_back(pose);
    }
    smooth_path_pub_.publish(smooth_path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_smoothing_demo_node");
    ros::NodeHandle nh("");
    DrivableMap drivable_region_node(nh, "path_smoothing_node");
    ROS_INFO("Initialized drivable region node.");

    ros::spin();
    return 0;
}
