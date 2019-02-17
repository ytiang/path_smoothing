//
// Created by yangt on 12/17/17.
//
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <lanelet_map_msgs/GlobalPath.h>
#include "path_smoothing/cg_solver.h"
#include "path_smoothing/path_smoothing.h"
#include <opt_utils/opt_utils.hpp>

#include <stdio.h>
class PathSmoothTest{
public:
    PathSmoothTest();
    CG_Solver path_smoother;
private:
    std::vector<geometry_msgs::Point> path_point_;
    std::vector<geometry_msgs::Point> left_bound_;
    std::vector<geometry_msgs::Point> right_bound_;
    int32_t goal_lane_id_;
    int32_t current_lane_id_;
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Timer timer_;
    std::string global_path_topic_{"lanelet_mapserver_node/global_path"};
    std::string my_frame_{"odom"};
    std::string visualization_topic_{"path_smooth"};



    void globalPathCb(const lanelet_map_msgs::GlobalPath &path);
    void timeCb();
};
///constructor init
PathSmoothTest::PathSmoothTest() :path_smoother() {
    this->goal_lane_id_ = 0;
    this->current_lane_id_ = 0;
    this->path_sub_ = nh_.subscribe(this->global_path_topic_,
                                    1,
                                    &PathSmoothTest::globalPathCb,
                                    this);
    this->timer_ = this->nh_.createTimer(ros::Duration(0.1),
                                                 boost::bind(&PathSmoothTest::timeCb, this));

    this->visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(this->my_frame_,
                                                  this->visualization_topic_));
    this->visual_tools_->loadMarkerPub();  // create publisher before waiting
    this->visual_tools_->deleteAllMarkers();
    this->visual_tools_->enableBatchPublishing(true);

}
///callBack funchtion
void PathSmoothTest::globalPathCb(const lanelet_map_msgs::GlobalPath &path) {
    using namespace rviz_visual_tools;
    if(path.path.empty()) {
        return;
    }
    int32_t id1 = path.path.front().id;
    int32_t id2 = path.path.back().id;
    if(this->current_lane_id_ == id1 && this->goal_lane_id_ == id2){
        return;
    }
    std::cout<<"path size :"<<path.path.size()<<std::endl;
    this->current_lane_id_ = id1;
    this->goal_lane_id_ = id2;
    this->path_point_.clear();
    this->right_bound_.clear();
    this->left_bound_.clear();
    for(const auto &lane : path.path){
        for(const auto &pt : lane.centerline.points){
            if(!(this->path_point_.empty())){
                double dx = pt.point.x - this->path_point_[path_point_.size()-1].x;
                double dy = pt.point.y - this->path_point_[path_point_.size()-1].y;
                double dist = sqrt(dx*dx+dy*dy);
                if(dist > 0.5)
                    this->path_point_.push_back(pt.point);
            }
            else{
                this->path_point_.push_back(pt.point);
            }
        }
        for(const auto &pt : lane.left_bound.points) {
            if (!(this->left_bound_.empty())) {
                double dx = pt.point.x - this->left_bound_[left_bound_.size() - 1].x;
                double dy = pt.point.y - this->left_bound_[left_bound_.size() - 1].y;
                double dist = sqrt(dx * dx + dy * dy);
                if (dist > 0.0005)
                    this->left_bound_.push_back(pt.point);
            } else {
                this->left_bound_.push_back(pt.point);
            }
        }
        for(const auto &pt : lane.right_bound.points) {
            if (!(this->right_bound_.empty())) {
                double dx = pt.point.x - this->right_bound_[right_bound_.size() - 1].x;
                double dy = pt.point.y - this->right_bound_[right_bound_.size() - 1].y;
                double dist = sqrt(dx * dx + dy * dy);
                if (dist > 0.0005)
                    this->right_bound_.push_back(pt.point);
            } else {
                this->right_bound_.push_back(pt.point);
            }
        }
    }
//    FILE *fp;
//    fp = fopen("DATA1.txt","w+");
//    for(auto pt : path_point_){
//        fprintf(fp,"%.3f %.3f\n",pt.x,pt.y);
//    }
//    fclose(fp);
    this->visual_tools_->deleteAllMarkers();
    this->visual_tools_->setAlpha(0.6);
    this->visual_tools_->publishLineStrip(path_point_, PINK, XXXLARGE, "path");
//    this->visual_tools_->publishLineStrip(right_bound_, GREEN, XXXLARGE, "path");
//    this->visual_tools_->publishLineStrip(left_bound_, BLUE, XXXLARGE, "path");
    std::vector<geometry_msgs::Point> finalPath;
    geometry_msgs::Point pts;
    ///self solver
    this->path_smoother.set_variable(this->path_point_, this->left_bound_,this->right_bound_, true);
    auto start1 = hmpl::now();
    this->path_smoother.Solve();
    auto end1 = hmpl::now();

    std::cout<<"self solver : "<<hmpl::getDurationInSecs(start1,end1)<<std::endl;

    for (int i = 0; i < this->path_smoother.Xk.size()/2; i++) {
        pts.x = this->path_smoother.Xk[2*i];
        pts.y = this->path_smoother.Xk[2*i+1];
        pts.z = 0;
        finalPath.push_back(pts);
    }
    pts.x = this->path_smoother.Xn[0];
    pts.y = this->path_smoother.Xn[1];
    finalPath.push_back(pts);
    pts.x = path_smoother.X0[0];
    pts.y = path_smoother.X0[1];
    finalPath.insert(finalPath.begin(),pts);
    this->visual_tools_->setAlpha(0.8);
    this->visual_tools_->publishLineStrip(finalPath, YELLOW, XXXLARGE, "path");

    ///ceres slover
    PathSmooth *path_smooth = new PathSmooth(this->path_point_);
    ceres::GradientProblem problem(path_smooth);
    ceres::GradientProblemSolver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    options.nonlinear_conjugate_gradient_type = ceres::HESTENES_STIEFEL;
//    options.line_search_type = ceres::ARMIJO;
    options.line_search_interpolation_type = ceres::BISECTION;
//    options.min_line_search_step_size = 1e-2;
//    options.function_tolerance = 1e-5;
//    options.gradient_tolerance = 1e-5;
//    options.parameter_tolerance = 1e-5;
    options.line_search_sufficient_function_decrease = 0.1;
    ceres::GradientProblemSolver::Summary summary;
    std::vector<geometry_msgs::Point> ceresPath;
    auto start2 = hmpl::now();
    ceres::Solve(options,problem,path_smooth->Xi,&summary);
    auto end2 = hmpl::now();
    std::cout<<"ceres: "<<hmpl::getDurationInSecs(start2,end2)<<std::endl;
    std::cout<<"solve over, points num is: "<<path_smooth->NumParameters()/2<<std::endl;

    for(size_t i=0; i<path_smooth->NumParameters()/2; ++i){
        pts.x = path_smooth->Xi[2*i];
        pts.y = path_smooth->Xi[2*i+1];
        pts.z = 0;
        ceresPath.push_back(pts);
    }
    ///display in rviz
    this->visual_tools_->setAlpha(0.8);
    this->visual_tools_->publishLineStrip(ceresPath, GREEN, XXXLARGE, "path");
    this->visual_tools_->trigger();
}
void PathSmoothTest::timeCb() {

}
int main(int argc, char **argv) {
    ros::init(argc, argv, "path_smooth_demo");
    PathSmoothTest path_smoother;
    ros::spin();
    return 0;
}