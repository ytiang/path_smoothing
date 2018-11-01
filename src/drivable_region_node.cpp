//
// Created by yangt on 1/19/18.
//
#include "path_smoothing/drivable_region_node.h"
#include "path_smoothing/cg_solver.h"
#include "path_smoothing/path_smoothing.h"
#include <ros/package.h>
//#include ""

void DrivableMap::readParameters() {
    this->use_self_solver_ =
            this->nh_.param("/path_smoothing_node/use_self_solver", true);
    this->is_save_map_ =
            this->nh_.param("/path_smoothing_node/is_save_map", false);
    this->is_consider_boundary_ =
            this->nh_.param("/path_smoothing_node/is_consider_boundary", false);
    this->w1_ = this->nh_.param("/path_smoothing_node/w1", 20.0);

    this->w2_ = this->nh_.param("/path_smoothing_node/w2", 0.0);
    this->w3_ = this->nh_.param("/path_smoothing_node/w3", 5.0);

    std::cout<<"use_self_solver= "<<this->use_self_solver_
             <<"\nw1= "<<this->w1_<<"\nw2= "
             <<this->w2_<<"\nw3= "<<this->w3_<<std::endl;
}

DrivableMap::DrivableMap(const ros::NodeHandle &nh, std::string node_name)
        : nh_(nh),
          is_set_goal_(false),
          is_save_map_(false),
          is_locate_(false),
          server_ptr_(new interactive_markers::
          InteractiveMarkerServer("obs_marker")) {
    // init grid map
    grid_map::Length map_length(80, 80);
    this->internal_grid_map_.init("odom", map_length, 0.05);
    this->reverse_map_.init("odom", map_length, 0.05);
    this->internal_grid_map_.maps[this->internal_grid_map_.obs].setZero();
    this->reverse_map_.maps[this->reverse_map_.obs].setConstant(255);
    // init parameters
    this->current_lanelet_.id = -1;
    this->last_lanelet_.id = -1;
    this->initialize();
    // obstacle related
    grid_map::Position pt(37, -4.3);
    hmpl::Circle obs;
    obs.position.x = pt.x();
    obs.position.y = pt.y();
    obs.r = this->circle_radius_;
    for (std::size_t i = 0; i < this->circle_obstacle_num_; i++) {
        this->point_obstacles_.push_back(obs);
        this->makeObstacleMarker(pt, i);
    }
    server_ptr_->applyChanges();
    // dynamic parameters
    this->server.setCallback(boost::bind(&DrivableMap::reconfigureRequest, this, _1, _2));
    //csv path
    std::string base_dir = ros::package::getPath("path_smoothing");
    hmpl::CSVReader path_file(base_dir + "/data/with_obs/astar_path.csv");
    path_file.data_list.erase(path_file.data_list.begin());
    for(const auto &pt : path_file.data_list) {
        size_t id;
        geometry_msgs::Point point;
        point.x = std::stod(pt[1], &id);
        point.y = std::stod(pt[2], &id);
        point.z = 0;

        if(origin_path_.empty()) {
            this->origin_path_.push_back(point);
        } else {
            double dx = origin_path_.back().x - point.x;
            double dy = origin_path_.back().y - point.y;
            double dist = sqrt(pow(dx, 2) + pow(dy, 2));
            if(dist > 1.5) {
                origin_path_.push_back(point);
            }
        }
    }
    int z = 10;
}

void DrivableMap::reconfigureRequest(const path_smoothing::smoothConfig &config,
                                     uint32_t level) {

    this->w1_ = config.w1;
    this->w2_ = config.w2;
    this->w3_ = config.w3;
    this->is_consider_boundary_ = config.is_consider_boundary;
    this->is_save_map_ = config.is_save_map;
    this->use_self_solver_ = config.use_self_solver;
    ROS_INFO("Reconfigure Request: %s %s %s %f %f %f",
             config.is_save_map?"True":"False",
             config.is_consider_boundary?"True":"False",
             config.use_self_solver?"True":"False",
             config.w1,
             config.w2,
             config.w3);
}
void DrivableMap::makeObstacleMarker(const grid_map::Position &pt, int id) {
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker i_marker;
    i_marker.header.frame_id = "odom";
    i_marker.header.stamp = ros::Time::now();
    i_marker.name = "maker_obstacle_" + std::to_string(id);
    i_marker.description = "obstacle_" + std::to_string(id);
    i_marker.pose.position.x = pt.x();
    i_marker.pose.position.y = pt.y();
    i_marker.pose.position.z = 0.1;

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.id = 0;
    box_marker.scale.x = 0.2;
    box_marker.scale.y = 0.2;
    box_marker.scale.z = 0.2;
    box_marker.color.r = 1.0;
    box_marker.color.g = 0.0;
    box_marker.color.b = 0.0;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( box_marker );


    // add the control to the interactive marker
    i_marker.controls.push_back( box_control );

    // create a control which will move the box, rviz will insert 2 arrows
    visualization_msgs::InteractiveMarkerControl move_control;
    move_control.name = "move_x";
    move_control.orientation.w = 1;
    move_control.orientation.x = 0;
    move_control.orientation.y = 1;
    move_control.orientation.z = 0;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


    // add the control to the interactive marker
    i_marker.controls.push_back(move_control);
    // add the interactive marker to our collection
    this->server_ptr_->insert(i_marker);
    this->server_ptr_->setCallback(i_marker.name,
                                   boost::bind(&DrivableMap::obstacleMakerCb, this, _1));
}

void DrivableMap::obstacleMakerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    // ascii to int
    std::size_t index = static_cast<std::size_t >(atoi(&feedback->marker_name.back()));
    this->point_obstacles_.at(index).position.x = feedback->pose.position.x;
    this->point_obstacles_.at(index).position.y = feedback->pose.position.y;
}

void DrivableMap::initialize() {
    this->readParameters();
    this->laneletmap_sub_ = this->nh_.subscribe(this->laneletmap_topic_,
                                                10,
                                                &DrivableMap::laneletMapCb,
                                                this);
    this->global_path_sub_ = this->nh_.subscribe("/astar_path",
                                          10,
                                          &DrivableMap::globalPathCb,
                                          this);
    this->vehicle_state_sub_ = this->nh_.subscribe("vehicle_state",
                                                   10,
                                                   &DrivableMap::vehicleStateCb,
                                                   this);
    this->poly_pub_ =
            this->nh_.advertise<geometry_msgs::PolygonStamped>(
                    "current_lanelet_polygon",
                    1,
                    true);
    this->path_pub_ = this->nh_.advertise<nav_msgs::Path>(
            "smooth_global_path", 1, true);

    this->rrt_path_pub_ =
            this->nh_.advertise<nav_msgs::Path>("rrt_path", 1, true);

    this->ogm_pub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>(
            "global_path/grid_map",
            1,
            true);

    this->point_cloud_ =
            this->nh_.advertise<sensor_msgs::PointCloud2>(
                    "map_point_cloud",
                    1,
                    true);

    this->goal_sub_ =
            this->nh_.subscribe("/move_base_simple/goal",
                                10,
                                &DrivableMap::goalCb,
                                this);

    /// 500ms
    this->timer_ =
            this->nh_.createTimer(
                    ros::Duration(0.2),
                    boost::bind(&DrivableMap::timerCb, this));
}

void DrivableMap::polygonExtractionCV(const grid_map::Polygon &polygon) {
    cv::Mat obs_f =
            hmpl::eigen2cv(internal_grid_map_.maps.get(internal_grid_map_.obs));

    cv::Mat obs_r = hmpl::eigen2cv(this->reverse_map_.maps.get(reverse_map_.obs));
    std::vector<cv::Point> fillContSingle;
    // one polygon
    for (const auto & pt : polygon.getVertices()) {
        grid_map::Index index;
        internal_grid_map_.maps.getIndex(pt, index);
        cv::Point point(index(0), index(1));
        fillContSingle.push_back(point);
    }
    // polygons vector
    std::vector<std::vector<cv::Point> > fillContAll;
    fillContAll.push_back(fillContSingle);
    cv::fillPoly(obs_f, fillContAll, this->internal_grid_map_.FREE);
    cv::fillPoly(obs_r, fillContAll, this->reverse_map_.OCCUPY);
}

void DrivableMap::clearPolygonMapRegion(
        const lanelet_map_msgs::LaneletMapConstPtr &lanelet_map) {
    if (!lanelet_map->forward_following.empty()) {
        this->clearSingleLaneletRegion(lanelet_map->forward_following.front());
    }
    if (!lanelet_map->forward_right.empty()) {
        this->clearSingleLaneletRegion(lanelet_map->forward_right.front());
    }
    if (!lanelet_map->forward_left.empty()) {
        this->clearSingleLaneletRegion(lanelet_map->forward_left.front());
    }
    if (!last_lanelet_.centerline.points.empty()) {
        this->clearSingleLaneletRegion(last_lanelet_);
    }
}

void DrivableMap::clearSingleLaneletRegion(
        const lanelet_map_msgs::Lanelet &lanelet) {
    std::vector<cv::Point> polygon;
    if (lanelet.centerline.points.empty()) {
        return;
    }
    for (const auto &itr : lanelet.left_bound.points) {
        grid_map::Index index;
        grid_map::Position pt(itr.point.x, itr.point.y);
        internal_grid_map_.maps.getIndex(pt, index);
        cv::Point pos(index(0), index(1));
        polygon.push_back(pos);
    }
    BOOST_REVERSE_FOREACH(const auto &itr,
                          lanelet.right_bound.points) {
                    grid_map::Index index;
                    grid_map::Position pt(itr.point.x, itr.point.y);
                    internal_grid_map_.maps.getIndex(pt, index);
                    cv::Point pos(index(0), index(1));
                    polygon.push_back(pos);
                }
    std::vector<std::vector<cv::Point> > polygons;
    polygons.push_back(polygon);
    cv::Mat obs_f = hmpl::eigen2cv(
            internal_grid_map_.maps.get(internal_grid_map_.obs));
    cv::Mat obs_r = hmpl::eigen2cv(this->reverse_map_.maps.get(reverse_map_.obs));

    cv::fillPoly(obs_f, polygons, this->internal_grid_map_.FREE);
    cv::fillPoly(obs_r,polygons,this->reverse_map_.OCCUPY);
}

void DrivableMap::extractDrivableRegion(
        const lanelet_map_msgs::LaneletMapConstPtr &lanlet_map) {
    grid_map::Position position(this->vehicle_state_.position.x,
                                this->vehicle_state_.position.y);
    // align map data with the position of the car
    this->internal_grid_map_.maps.setPosition(position);
    this->internal_grid_map_.maps.move(position);

    this->reverse_map_.maps.setPosition(position);
    this->reverse_map_.maps.move(position);

    this->internal_grid_map_.maps.get(internal_grid_map_.obs).setZero();

    this->reverse_map_.maps.get(reverse_map_.obs).setConstant(255);
    grid_map::Polygon polygon;
    polygon.setFrameId(this->internal_grid_map_.maps.getFrameId());
    for (const auto &itr : lanlet_map->current_lane.front().left_bound.points) {
        grid_map::Position pos(itr.point.x, itr.point.y);
        polygon.addVertex(pos);
    }
    BOOST_REVERSE_FOREACH(const auto &itr,
    lanlet_map->current_lane.front().right_bound.points) {
        grid_map::Position pos(itr.point.x, itr.point.y);
        polygon.addVertex(pos);
    }

    // lanelet map
    this->polygonExtractionCV(polygon);
    this->clearPolygonMapRegion(lanlet_map);
    geometry_msgs::PolygonStamped p_message;
    p_message.header.stamp = ros::Time::now();
    grid_map::PolygonRosConverter::toMessage(polygon, p_message);
    this->poly_pub_.publish(p_message);
}

void DrivableMap::updateSignDistanceField() {
    this->internal_grid_map_.updateDistanceLayerCV();
    this->reverse_map_.updateDistanceLayerCV();
    cv::Mat dis_p = hmpl::eigen2cv(
            this->internal_grid_map_.maps.get(internal_grid_map_.dis));
    cv::Mat dis_r = hmpl::eigen2cv(
            this->reverse_map_.maps.get(reverse_map_.dis));
    dis_p = dis_p - dis_r;
    double th = 3.0;
    for(grid_map::GridMapIterator it(internal_grid_map_.maps); !it.isPastEnd(); ++it) {
        grid_map::Position pt;
        this->internal_grid_map_.maps.getPosition(*it, pt);
        double dist = this->internal_grid_map_.maps.at("distance", *it);
        if(dist < 0) {
            this->internal_grid_map_.maps.at("distance", *it) = th - dist;
        } else if(dist <= th ) {
            this->internal_grid_map_.maps.at("distance", *it) = pow((dist - th),2)/th;
        } else {
            this->internal_grid_map_.maps.at("distance", *it) = 0.;
        }
    }
    sensor_msgs::PointCloud2 pointcloud;
    grid_map::GridMapRosConverter::toPointCloud(this->internal_grid_map_.maps,
                                                this->internal_grid_map_.dis,
                                                pointcloud);

    this->point_cloud_.publish(pointcloud);
}
void DrivableMap::fillCircleObstacles(const std::vector<hmpl::Circle> &obstacles) {
    cv::Mat obs_f =
            hmpl::eigen2cv(internal_grid_map_.maps.get(internal_grid_map_.obs));
    cv::Mat obs_r =
            hmpl::eigen2cv((this->reverse_map_.maps.get(reverse_map_.obs)));
    for (const auto & itr : obstacles) {
        grid_map::Index index;
        grid_map::Position pt(itr.position.x, itr.position.y);
        internal_grid_map_.maps.getIndex(pt, index);
        cv::Point point(index(0), index(1));
        int R = static_cast<int>(itr.r / this->internal_grid_map_.maps.getResolution());
        cv::circle(obs_f, point, R, this->internal_grid_map_.OCCUPY, -1);
        cv::circle(obs_r, point, R, this->reverse_map_.FREE, -1);
    }
}

void DrivableMap::laneletMapCb(
        const lanelet_map_msgs::LaneletMapConstPtr &lanelet) {
    if(lanelet->current_lane.empty()) {
        ROS_WARN_THROTTLE(2,
                          "drivable region node received empty lanelet message");
        return;
    } else{
        if (this->current_lanelet_.id != lanelet->current_lane.front().id) {
            this->last_lanelet_ = this->current_lanelet_;
            this->current_lanelet_ = lanelet->current_lane.front();
        }
    }
    // update the gride map(just the obstacle layer)
    this->extractDrivableRegion(lanelet);
    if (!this->point_obstacles_.empty()) {
        // fill the obstacles into the grid map
        this->fillCircleObstacles(this->point_obstacles_);
    }
}

void DrivableMap::globalPathCb(const nav_msgs::PathConstPtr &path) {
    this->global_path_.poses.clear();
    this->global_path_.header = path->header;
    for (auto it : path->poses) {
        if(!this->global_path_.poses.empty()) {
            double dist = sqrt(pow(it.pose.position.x-global_path_.poses.back().pose.position.x,2) +
                                       pow(it.pose.position.y-global_path_.poses.back().pose.position.y,2));
            if(dist > 1.5) {
//                geometry_msgs::PoseStamped pose = it;
//                pose.pose.position.x = (global_path_.poses.back().pose.position.x + it.pose.position.x)/2;
//                pose.pose.position.y = (global_path_.poses.back().pose.position.y + it.pose.position.y)/2;
//                this->global_path_.poses.push_back(pose);
                this->global_path_.poses.push_back(it);
            }
        } else {
            this->global_path_.poses.push_back(it);
        }
    }
//    this->global_path_ = *path;
    this->is_set_goal_ = true;
}

void DrivableMap::vehicleStateCb(const anm_msgs::VehicleStateConstPtr &state) {
    this->vehicle_state_ = *state;
    this->is_locate_ = true;
}

void DrivableMap::goalCb(const geometry_msgs::PoseStamped &pose) {
    this->goal_pose_ = pose.pose;
    this->is_set_goal_ = true;
}

void DrivableMap::timerCb() {
    updateSignDistanceField();
    CG_Solver smoother(this->origin_path_);
    smoother.Solve();
    this->path_pub_.publish(smoother.getSmoothPath());
//    if(is_locate_ && is_set_goal_) {
//        if(!global_path_.poses.empty()) {
//            std::cout << "timeCb" << std::endl;
//            this->use_self_solver_ = true;
//            this->is_consider_boundary_ = false;
//            if (this->use_self_solver_) {
//                auto start = hmpl::now();
////                CG_Solver smooth_solver(
////                        this->global_path_,
////                        this->w1_,
////                        this->w2_,
////                        this->w3_,
////                        this->is_consider_boundary_,
////                        this->internal_grid_map_.maps);
//                CG_Solver smooth_solver(this->origin_path_);
//                smooth_solver.Solve();
//                auto end = hmpl::now();
//                std::cout << "self_solver time: " << hmpl::getDurationInSecs(start, end)
//                          << std::endl;
////                std::cout <<"smooth path size: " << smooth_solver.getSmoothPath().poses.size() << std::endl;
//                this->path_pub_.publish(smooth_solver.getSmoothPath());
//                this->is_save_map_ = true;
//                if (this->is_save_map_) {
//                    std::string base_dir = ros::package::getPath("path_smoothing");
//                    hmpl::CSVFile dis_map(base_dir + "/astar_path.csv");
//                    dis_map << "x" << "y" << hmpl::endrow;
//                    for(const auto &pt:this->global_path_.poses) {
//                        dis_map << pt.pose.position.x << pt.pose.position.y << hmpl::endrow;
////                        dis_map << "dx" << "dy" << "cost" << hmpl::endrow;
////                for (grid_map::GridMapIterator it(this->internal_grid_map_.maps);
////                     !it.isPastEnd(); ++it) {
////                    grid_map::Position pos;
////                    this->internal_grid_map_.maps.getPosition(*it, pos);
////                    grid_map::Index index;
////                    this->internal_grid_map_.maps.getIndex(pos, index);
////                    dis_map << index(0)
////                            << index(1)
////                            << this->internal_grid_map_.maps.at(
////                                    internal_grid_map_.dis, *it)
////                            << hmpl::endrow;
//                    }
//
//                    this->is_save_map_ = false;
//                }
//            } else {
//                /// ceres solver
//                auto start = hmpl::now();
//                PathSmooth *path_smooth = new PathSmooth(global_path_,
//                                                         this->w1_,
//                                                         this->w2_,
//                                                         this->w3_,
//                                                         is_consider_boundary_,
//                                                         this->internal_grid_map_.maps);
//                ceres::GradientProblem problem(path_smooth);
//                ceres::GradientProblemSolver::Options options;
//                options.minimizer_progress_to_stdout = false;
//        //            options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
//                options.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE;
//        //            options.line_search_type = ceres::ARMIJO;
//                options.line_search_interpolation_type = ceres::BISECTION;
//                options.min_line_search_step_size = 1e-2;
//                options.function_tolerance = 1e-3;
//                options.gradient_tolerance = 1e-3;
//                options.parameter_tolerance = 1e-3;
//                options.line_search_sufficient_function_decrease = 0.1;
//                ceres::GradientProblemSolver::Summary summary;
//                ceres::Solve(options, problem, path_smooth->Xi, &summary);
//                auto end = hmpl::now();
//                std::cout << "ceres time: " << hmpl::getDurationInSecs(start, end)
//                          << std::endl;
//                this->path_pub_.publish(path_smooth->getSmoothPath());
//            }
//        }
//        global_path_.poses.clear();
//    }

    ros::Time time = ros::Time::now();
    this->internal_grid_map_.maps.setTimestamp(time.toNSec());
    nav_msgs::OccupancyGrid message;
    grid_map::GridMapRosConverter::toOccupancyGrid(
            this->internal_grid_map_.maps, this->internal_grid_map_.obs,
            this->internal_grid_map_.FREE, this->internal_grid_map_.OCCUPY,
            message);
    this->ogm_pub_.publish(message);

}

void DrivableMap::wayRefine(const nav_msgs::Path &path,
                                   std::vector<grid_map::Position> *way) {
    for (std::size_t i = 0; i < path.poses.size() - 2; i++) {
        double x1 = path.poses.at(i).pose.position.x;
        double y1 = path.poses.at(i).pose.position.y;
        double x2 = path.poses.at(i+1).pose.position.x;
        double y2 = path.poses.at(i+1).pose.position.y;
        // if both are outside the map, skip
        if (!internal_grid_map_.maps.isInside(grid_map::Position(x1, y1)) &&
            !internal_grid_map_.maps.isInside(grid_map::Position(x2, y2)) ) {
            continue;
        }
        double heading = std::atan2(y2 - y1, x2 - x1);
        double length = sqrt((x2-x1)*(x2-x1) + (y2 - y1)*(y2 - y1));
        double resolution = internal_grid_map_.maps.getResolution();
        double num = length / resolution;
        for (std::size_t j = 0; j < static_cast<int>(num); j++) {
            grid_map::Position pos;
            pos(0) = x1 + j * resolution * cos(heading);
            pos(1) = y1 + j * resolution * sin(heading);
            // skip the point outside the map
            if (!internal_grid_map_.maps.isInside(pos)) {
                continue;
            }
            way->push_back(pos);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_smoothing_node");
    ros::NodeHandle nh("");
    DrivableMap drivable_region_node(nh, "path_smoothing_node");
    ROS_INFO("Initialized drivable region node.");
    ros::spin();

//    ros::Publisher sas_path_pub = nh.advertise<nav_msgs::Path>(
//            "sas_path", 1, true);
//    ros::Publisher smooth_path_pub = nh.advertise<nav_msgs::Path>(
//            "smooth_path", 1, true);
//    std::string package_dir = ros::package::getPath("path_smoothing");
//    std::ifstream in(package_dir+"/sas_path.txt");
//    geometry_msgs::Point pt;
//    std::vector<geometry_msgs::Point> path;
//    while(!in.eof()) {
//        in >> pt.x;
//        in >> pt.y;
//        pt.z = 0;
//        if(!path.empty()) {
//            double dis = pow(pt.x-path.back().x,2) + pow(pt.y-path.back().y,2);
//            dis = sqrt(dis);
//            if(dis > 0.5) {
//                path.push_back(pt);
//            }
//        } else {
//            path.push_back(pt);
//        }
//    }
//    ros::Rate rate(1.0);
//    while(nh.ok()) {
////        PathSmooth *path_smooth = new PathSmooth(path);
////        nav_msgs::Path sas_path = path_smooth->getSmoothPath();
////        ceres::GradientProblem problem(path_smooth);
////        ceres::GradientProblemSolver::Options options;
////        options.minimizer_progress_to_stdout = false;
////                    options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
////        options.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE;
////                    options.line_search_type = ceres::ARMIJO;
////        options.line_search_interpolation_type = ceres::BISECTION;
//////        options.min_line_search_step_size = 1e-2;
//////        options.function_tolerance = 1e-3;
//////        options.gradient_tolerance = 1e-3;
//////        options.parameter_tolerance = 1e-3;
//////        options.line_search_sufficient_function_decrease = 0.1;
////        ceres::GradientProblemSolver::Summary summary;
////        auto start2 = hmpl::now();
////        ceres::Solve(options,problem,path_smooth->Xi,&summary);
////        auto end2 = hmpl::now();
////        std::cout<<"ceres: "<<hmpl::getDurationInSecs(start2,end2)<<std::endl;
////        nav_msgs::Path smooth_path = path_smooth->getSmoothPath();
//
//        CG_Solver sp(path);
//        nav_msgs::Path sas_path = sp.getSmoothPath();
//        auto start2 = hmpl::now();
//        sp.Solve();
//        auto end2 = hmpl::now();
//        nav_msgs::Path smooth_path = sp.getSmoothPath();
//        std::cout<<"solver time: "<<hmpl::getDurationInSecs(start2,end2)<<std::endl;
//        sas_path_pub.publish(sas_path);
//        smooth_path_pub.publish(smooth_path);
//        rate.sleep();
//    }
    return 0;
}