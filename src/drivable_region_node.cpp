//
// Created by yangt on 1/19/18.
//
#include "path_smoothing/drivable_region_node.h"
#include "opt_utils/csv_writer.hpp"
//#include "path_smoothing/cg_solver.h"
//#include "path_smoothing/path_smoothing.h"
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

	std::cout << "use_self_solver= " << this->use_self_solver_
			  << "\nw1= " << this->w1_ << "\nw2= "
			  << this->w2_ << "\nw3= " << this->w3_ << std::endl;
}

DrivableMap::DrivableMap(const ros::NodeHandle &nh, std::string node_name)
	: nh_(nh),
	  is_set_goal_(false),
	  is_save_map_(false),
	  is_locate_(false),
	  server_ptr_(new interactive_markers::
	  InteractiveMarkerServer("obs_marker")) {
	// init grid map
	grid_map::Length map_length(20, 20);
	this->in_gm.init("odom", map_length, 0.05);
	this->reverse_map_.init("odom", map_length, 0.05);
	this->in_gm.maps[this->in_gm.obs].setConstant(
		in_gm.FREE);
	this->reverse_map_.maps[this->reverse_map_.obs].setConstant(
		reverse_map_.OCCUPY);
	// init parameters
	this->initialize();
	this->start_pose_.position.x = 0.0;
	this->start_pose_.position.y = 0.0;
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
	this->server.setCallback(
		boost::bind(&DrivableMap::reconfigureRequest, this, _1, _2));
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
			 config.is_save_map ? "True" : "False",
			 config.is_consider_boundary ? "True" : "False",
			 config.use_self_solver ? "True" : "False",
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
	box_control.markers.push_back(box_marker);


	// add the control to the interactive marker
	i_marker.controls.push_back(box_control);

	// create a control which will move the box, rviz will insert 2 arrows
	visualization_msgs::InteractiveMarkerControl move_control;
	move_control.name = "move_x";
	move_control.orientation.w = 1;
	move_control.orientation.x = 0;
	move_control.orientation.y = 1;
	move_control.orientation.z = 0;
	move_control.interaction_mode =
		visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


	// add the control to the interactive marker
	i_marker.controls.push_back(move_control);
	// add the interactive marker to our collection
	this->server_ptr_->insert(i_marker);
	this->server_ptr_->setCallback(i_marker.name,
								   boost::bind(&DrivableMap::obstacleMakerCb,
											   this,
											   _1));
}

void DrivableMap::obstacleMakerCb(
	const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	// ascii to int
	std::size_t index = static_cast<std::size_t >(atoi(
		&feedback->marker_name.back()));
	this->point_obstacles_.at(index).position.x = feedback->pose.position.x;
	this->point_obstacles_.at(index).position.y = feedback->pose.position.y;
}

void DrivableMap::initialize() {
	this->readParameters();
	this->path_pub_ = this->nh_.advertise<nav_msgs::Path>(
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

	this->goal_pose_sub_ = this->nh_.subscribe("/move_base_simple/goal", 1,
											   &DrivableMap::goalPoseCb, this);

	/// 500ms
	this->timer_ =
		this->nh_.createTimer(
			ros::Duration(0.2),
			boost::bind(&DrivableMap::timerCb, this));
}

void DrivableMap::updateSignDistanceField() {
	this->in_gm.updateDistanceLayerCV();
	if (is_set_goal_) {
		hmpl::CSVWriter csv;
		csv.newRow() << "id" << "x" << "y" << "d";
		int id = 0;
		for (grid_map::GridMapIterator it(
			in_gm.maps); !(it.isPastEnd()); ++it) {
			grid_map::Position pt;
			in_gm.maps.getPosition(*it, pt);
			csv.newRow() << id << pt(0) << pt(1)
						 << in_gm.maps.atPosition("distance", pt);
			id++;
		}
		csv.writeToFile("/home/yangt/distance_map.csv");
		std::cout << "wite to file over!!!\n";
	}
	this->reverse_map_.updateDistanceLayerCV();
	cv::Mat dis_p = hmpl::eigen2cv(
		this->in_gm.maps.get(in_gm.dis));
	cv::Mat dis_r = hmpl::eigen2cv(
		this->reverse_map_.maps.get(reverse_map_.dis));
	dis_p = dis_p - dis_r;

	double th = 5.0;
	for (grid_map::GridMapIterator it(in_gm.maps); !it.isPastEnd(); ++it) {
		grid_map::Position pt;
		this->in_gm.maps.getPosition(*it, pt);
		double dist = this->in_gm.maps.at("distance", *it);
		if (dist < 0) {
			this->in_gm.maps.at("distance", *it) = th - dist;
		} else if (dist <= th) {
			this->in_gm.maps.at("distance", *it) = pow((dist - th), 2) / th;
		} else {
			this->in_gm.maps.at("distance", *it) = 0.;
		}
	}
	sensor_msgs::PointCloud2 pointcloud;
	grid_map::GridMapRosConverter::toPointCloud(this->in_gm.maps,
												this->in_gm.dis,
												pointcloud);

	this->point_cloud_.publish(pointcloud);
}

void
DrivableMap::fillCircleObstacles(const std::vector<hmpl::Circle> &obstacles) {
	cv::Mat obs_f =
		hmpl::eigen2cv(in_gm.maps.get(in_gm.obs));
	cv::Mat obs_r =
		hmpl::eigen2cv((this->reverse_map_.maps.get(reverse_map_.obs)));
	for (const auto &itr : obstacles) {
		grid_map::Index index;
		grid_map::Position pt(itr.position.x, itr.position.y);
		in_gm.maps.getIndex(pt, index);
		cv::Point point(index(0), index(1));
		int R = static_cast<int>(itr.r / this->in_gm.maps.getResolution());
		cv::circle(obs_f, point, R, this->in_gm.OCCUPY, -1);
		cv::circle(obs_r, point, R, this->reverse_map_.FREE, -1);
	}
}

void DrivableMap::goalPoseCb(const geometry_msgs::PoseStamped &goal) {
	this->goal_pose_ = goal.pose;
	this->is_set_goal_ = true;
}

void DrivableMap::updateGridMap() {
	auto &obs_layer = this->in_gm.obs;
	this->in_gm.maps[obs_layer].setConstant(in_gm.FREE);
	this->reverse_map_.maps[obs_layer].setConstant(in_gm.OCCUPY);
	if (!(this->point_obstacles_.empty())) {
		this->fillCircleObstacles(point_obstacles_);
	}
}

void DrivableMap::timerCb() {
	this->updateGridMap();
	this->updateSignDistanceField();
	if (is_set_goal_) {
		// initilize path
		geometry_msgs::Point tmp_pt;
		std::vector<geometry_msgs::Point> init_path;
		double init_obs_cost = 0.0;
		double dx = goal_pose_.position.x - start_pose_.position.x;
		double dy = goal_pose_.position.y - start_pose_.position.y;
		for (size_t i(0); i < 30; ++i) {
			tmp_pt.x = dx * i / 30.0;
			tmp_pt.y = dy * i / 30.0;
			init_path.push_back(tmp_pt);
			grid_map::Position grid_pt(tmp_pt.x, tmp_pt.y);
			init_obs_cost += this->in_gm.maps.atPosition("distance", grid_pt);
		}
		std::cout << "init obs cost: " << init_obs_cost << "\n";
		// optimize
		ChompOptimizer chomp_optimizer(init_path, this->in_gm.maps);
		auto t1 = hmpl::now();
		nav_msgs::Path final_path = chomp_optimizer.generatePath();
		auto t2 = hmpl::now();
		std::cout << "smooth time: " << hmpl::getDurationInSecs(t1, t2) << "\n";
		this->path_pub_.publish(final_path);
	}
	ros::Time time = ros::Time::now();
	this->in_gm.maps.setTimestamp(time.toNSec());
	nav_msgs::OccupancyGrid message;
	grid_map::GridMapRosConverter::toOccupancyGrid(
		this->in_gm.maps, this->in_gm.obs,
		this->in_gm.FREE, this->in_gm.OCCUPY,
		message);
	this->ogm_pub_.publish(message);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "path_smoothing_node");
	ros::NodeHandle nh("");
	DrivableMap drivable_region_node(nh, "path_smoothing_node");
	ROS_INFO("Initialized drivable region node.");
	cv::Mat img_src = cv::imread("/home/yangt/Pictures/obstacles.png", CV_8UC1);
	double resolution = 0.05;  // in meter
	hmpl::InternalGridMap in_gm;
	in_gm.initializeFromImage(img_src, resolution, grid_map::Position::Zero());
	in_gm.addObstacleLayerFromImage(img_src, 0.5);
	in_gm.updateDistanceLayerCV();
	hmpl::CSVWriter csv;
	csv.newRow() << "id" << "x" << "y" << "d";
	int id = 0;
	for (grid_map::GridMapIterator it(in_gm.maps); !(it.isPastEnd()); ++it) {
		grid_map::Position pt;
		in_gm.maps.getPosition(*it, pt);
		csv.newRow() << id << pt(0) << pt(1)
					 << in_gm.maps.atPosition("distance", pt);
		id++;
	}
	csv.writeToFile("/home/yangt/distance_map.csv");
	printf("map size: %d *%d, resolution: %f\n", in_gm.maps.getSize()(0),
		   in_gm.maps.getSize()(1), in_gm.maps.getResolution());
	ros::spin();
	return 0;
}