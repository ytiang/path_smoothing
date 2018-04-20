#include "path_smoothing/path_smoothing.h"

int PathSmooth::NumParameters() const {
    return this->numParam_;
}

PathSmooth::PathSmooth(const std::vector<geometry_msgs::Point> &path)
        : is_consider_boundary_(false) {
    int numPoint = path.size() - 2;
    this->numParam_ = numPoint * 2;

    this->Xi = new double [this->numParam_];
    std::vector< CppAD::AD<double> > X_ad(this->numParam_);
    std::vector< CppAD::AD<double> > Y_ad(1);
    std::vector< CppAD::AD<double> > X_init_ad(2);
    std::vector< CppAD::AD<double> > X_goal_ad(2);
    this->X_init_.resize(2);
    this->X_goal_.resize(2);
    X_init_ad[0] = this->X_init_[0] = path[0].x;
    X_init_ad[1] = this->X_init_[1] = path[0].y;
    X_goal_ad[0] = this->X_goal_[0] = path[numPoint + 1].x;
    X_goal_ad[1] = this->X_goal_[1] = path[numPoint + 1].y;
    for(int i=0 ; i<numPoint; i++) {
        this->Xi[i*2] = path[i+1].x;
        this->Xi[i*2+1] = path[i+1].y;
        X_ad[2*i] = path[i+1].x;
        X_ad[2*i+1] = path[i+1].y;
    }
    CppAD::Independent(X_ad);
    Y_ad = this->Poly(X_ad, X_init_ad, X_goal_ad);
    this->Fun_ = new CppAD::ADFun<double>(X_ad, Y_ad);
    this->is_consider_boundary_ = false;
    this->w1_ = 10;
    this->w2_ = 1;
}
PathSmooth::PathSmooth(const nav_msgs::Path &path,
                       double w1,
                       double w2,
                       double w3,
                       bool boundary,
                       const grid_map::GridMap &map)
        : w1_(w1),
          w2_(w2),
          w3_(w3),
          is_consider_boundary_(boundary),
          gridmap_(map),
          gradient_map_(grid_map::GridMap(std::vector<std::string>{"grad_x", "grad_y"})) {
    int numPoint = path.poses.size() - 2;
    this->numParam_ = numPoint * 2;

    this->Xi = new double [this->numParam_];
    std::vector< CppAD::AD<double> > X_ad(this->numParam_);
    std::vector< CppAD::AD<double> > Y_ad(1);
    std::vector< CppAD::AD<double> > X0_ad(2);
    std::vector< CppAD::AD<double> > Xn_ad(2);
    this->X_init_.resize(2);
    this->X_goal_.resize(2);
    X0_ad[0] = this->X_init_[0] = path.poses.at(0).pose.position.x;
    X0_ad[1] = this->X_init_[1] = path.poses.at(0).pose.position.y;
    Xn_ad[0] = this->X_goal_[0] = path.poses.at(numPoint+1).pose.position.x;
    Xn_ad[1] = this->X_goal_[1] = path.poses.at(numPoint+1).pose.position.y;
    for(int i=0 ; i<numPoint; i++){
        this->Xi[i*2] = path.poses.at(i+1).pose.position.x;
        this->Xi[i*2+1] = path.poses.at(i+1).pose.position.y;
        X_ad[2*i] = path.poses.at(i+1).pose.position.x;
        X_ad[2*i+1] = path.poses.at(i+1).pose.position.y;
    }
    /// init cppAD function
    CppAD::Independent(X_ad);
    Y_ad = this->Poly(X_ad, X0_ad, Xn_ad);
    this->Fun_ = new CppAD::ADFun<double>(X_ad, Y_ad);
    if(is_consider_boundary_) {
        /// init enviroment gradient map
        this->gradient_map_.setFrameId(gridmap_.getFrameId());
        this->gradient_map_.setGeometry(gridmap_.getLength(),
                                         gridmap_.getResolution(),
                                         gridmap_.getPosition());
        /// init gradient map
        cv::Mat dis_field =
                hmpl::eigen2cv(this->gridmap_.get("distance"));
        cv::Mat grad_x = hmpl::eigen2cv(this->gradient_map_.get("grad_x"));
        cv::Mat grad_y = hmpl::eigen2cv(this->gradient_map_.get("grad_y"));
        // note: opencv's coordinate is different from eigen
        cv::Sobel(dis_field, grad_x, CV_32F, 0, 1);
        cv::Sobel(dis_field, grad_y, CV_32F, 1, 0);
    }
}

PathSmooth::PathSmooth(const nav_msgs::Path &path,
                       const grid_map::GridMap &distance_map,
                       const grid_map::GridMap &grad_map) :
        gridmap_(distance_map),
        gradient_map_(grad_map) {
    int numPoint = path.poses.size() - 2;
    this->numParam_ = numPoint * 2;

    this->Xi = new double [this->numParam_];
    std::vector< CppAD::AD<double> > X_ad(this->numParam_);
    std::vector< CppAD::AD<double> > Y_ad(1);
    std::vector< CppAD::AD<double> > X0_ad(2);
    std::vector< CppAD::AD<double> > Xn_ad(2);
    this->X_init_.resize(2);
    this->X_goal_.resize(2);
    X0_ad[0] = this->X_init_[0] = path.poses.at(0).pose.position.x;
    X0_ad[1] = this->X_init_[1] = path.poses.at(0).pose.position.y;
    Xn_ad[0] = this->X_goal_[0] = path.poses.at(numPoint+1).pose.position.x;
    Xn_ad[1] = this->X_goal_[1] = path.poses.at(numPoint+1).pose.position.y;
    for(int i=0 ; i<numPoint; i++){
        this->Xi[i*2] = path.poses.at(i+1).pose.position.x;
        this->Xi[i*2+1] = path.poses.at(i+1).pose.position.y;
        X_ad[2*i] = path.poses.at(i+1).pose.position.x;
        X_ad[2*i+1] = path.poses.at(i+1).pose.position.y;
    }
    /// init cppAD function
    CppAD::Independent(X_ad);
    Y_ad = this->Poly(X_ad, X0_ad, Xn_ad);
    this->Fun_ = new CppAD::ADFun<double>(X_ad, Y_ad);

    this->is_consider_boundary_ = false;
    std::cout << "ceres init over!"<<std::endl;
}
bool PathSmooth::Evaluate(const double *const parameters,
                          double *cost,
                          double *gradient) const {
    auto data_begin = parameters;
    auto data_end = parameters + this->numParam_;
    std::vector<double> X(data_begin, data_end);

    std::vector<double> F1(1);
    F1 = this->Poly(X, this->X_init_, this->X_goal_);
    double F2 = 0.0;
    if(is_consider_boundary_){
        for (size_t i = 0; i < this->numParam_/2; i++) {
            grid_map::Position pt(X.at(2 * i), X.at(2 * i + 1));
            if(!this->gridmap_.isInside(pt)) {
                continue;
            }
            double dis = this->gridmap_.atPosition("distance", pt);
//            F2 += this->w3_ * std::exp(-20*(dis-0.5));
//            F2 += this->w3_ * std::pow(dis-1.2, 2);
            F2 += this->w3_ * dis;
        }

    }
    cost[0] = F1[0] + F2;
    if(gradient != NULL) {
        std::vector<double> G(this->numParam_);
        G = this->Fun_->Jacobian(X);
        if(this->is_consider_boundary_) {

            for(size_t i = 0; i < this->numParam_/2; i++) {
                grid_map::Position pt(X.at(2 * i), X.at(2 * i + 1));
                if(!this->gridmap_.isInside(pt)){
                    std::cout<<"pt: "<<pt(0)<<" "<<pt(1)<<std::endl;
                    continue;
                }
                double grad_x = this->gradient_map_.atPosition("grad_x", pt);
                double grad_y = this->gradient_map_.atPosition("grad_y", pt);
                double dis = this->gridmap_.atPosition("distance", pt);
                gradient[2*i] = G[2*i] +grad_x * this->w3_;
                gradient[2*i+1] = G[2*i+1] +grad_y * this->w3_;
            }
        }
        else {
//            for (size_t i = 0; i < this->numParam_; i++) {
//                gradient[i] = G.at(i);
//            }
            gradient = &G[0];
        }
    }
}
void PathSmooth::setParams(double w1, double w2, double w3) {
    this->w1_ = w1;
    this->w2_ = w2;
    this->w3_ = w3;
}

nav_msgs::Path PathSmooth::getSmoothPath() {
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.header.frame_id = "odom";//this->gridmap_.getFrameId();
    path.header.stamp = ros::Time::now();
    pose.header = path.header;
    for (size_t i = 0; i < this->numParam_/2; i++) {
        pose.pose.position.x = this->Xi[2*i];
        pose.pose.position.y = this->Xi[2*i+1];
        pose.pose.position.z = 0;
        path.poses.push_back(pose);
    }
    pose.pose.position.x = this->X_goal_[0];
    pose.pose.position.y = this->X_goal_[1];
    path.poses.push_back(pose);
    pose.pose.position.x = this->X_init_[0];
    pose.pose.position.y = this->X_init_[1];
    path.poses.insert(path.poses.begin(), pose);
    return path;
}
