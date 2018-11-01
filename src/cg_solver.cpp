//
// Created by yt on 10/31/17.
//
#include <opt_utils/opt_utils.hpp>
#include <path_smoothing/cg_solver.h>
#include <internal_grid_map/internal_grid_map.hpp>

CG_Solver::CG_Solver(std::vector<geometry_msgs::Point> &path) {
    this->numPoint = path.size() - 2;
    std::vector< CppAD::AD<double> > X_ad(this->numPoint*2);
    this->Xk.resize(this->numPoint*2);
    this->X0.resize(2);
    this->Xn.resize(2);
    for (int i = 0; i < numPoint; i++)
    {
        X_ad[2*i] = path[i+1].x;
        X_ad[2*i+1] = path[i+1].y;
        this->Xk[2*i] = path[i+1].x;
        this->Xk[2*i+1] = path[i+1].y;
    }
    CppAD::Independent(X_ad);
    std::vector< CppAD::AD<double> > Y(1);
    std::vector< CppAD::AD<double> > x0(2);
    std::vector< CppAD::AD<double> > xn(2);
    x0[0] = this->X0[0] = path[0].x;
    x0[1] = this->X0[1] = path[0].y;
    xn[0] = this->Xn[0] = path[numPoint+1].x;
    xn[1] = this->Xn[1] = path[numPoint+1].y;
    Y = Poly(X_ad, x0, xn);
    CppAD::ADFun<double> tempFun(X_ad, Y);
    this->Fun = tempFun;
    this->is_consider_boundary_ = false;
    this->w1_ = 10;
    this->w2_ = 1;
    this->w3_ = 0;
    std::cout << "solver init: " << numPoint<<std::endl;
}
CG_Solver::CG_Solver(const nav_msgs::Path &path) {
    this->numPoint = path.poses.size() - 2;
    std::vector< CppAD::AD<double> > X_ad(this->numPoint*2);
    this->Xk.resize(this->numPoint*2);
    this->X0.resize(2);
    this->Xn.resize(2);
    for(int i = 0; i < numPoint; i++)
    {
        X_ad[2*i] = path.poses.at(i+1).pose.position.x;
        X_ad[2*i+1] = path.poses.at(i+1).pose.position.y;
        this->Xk[2*i] = path.poses.at(i+1).pose.position.x;
        this->Xk[2*i+1] = path.poses.at(i+1).pose.position.y;
    }
    /// init cppAD
    CppAD::Independent(X_ad);
    std::vector< CppAD::AD<double> > Y(1);
    std::vector< CppAD::AD<double> > x0(2);
    std::vector< CppAD::AD<double> > xn(2);
    x0[0] = this->X0[0] = path.poses.at(0).pose.position.x;
    x0[1] = this->X0[1] = path.poses.at(0).pose.position.y;
    xn[0] = this->Xn[0] = path.poses.at(numPoint+1).pose.position.x;
    xn[1] = this->Xn[1] = path.poses.at(numPoint+1).pose.position.y;
    Y = Poly(X_ad, x0, xn);
    CppAD::ADFun<double> tempFun(X_ad, Y);
    this->Fun = tempFun;
    this->is_consider_boundary_ = false;
    this->w1_ = 10;
    this->w2_ = 1;
    this->w3_ = 0;
    std::cout << "solver init: " << numPoint << std::endl;
}
CG_Solver::CG_Solver(const nav_msgs::Path &path,
                     double w1,
                     double w2,
                     double w3,
                     bool boundary,
                     const grid_map::GridMap &gridmap)
        : w1_(w1),
          w2_(w2),
          w3_(w3),
          is_consider_boundary_(boundary),
          map_grad(grid_map::GridMap(
                  std::vector<std::string>{"grad_x", "grad_y"})),
          gridmap_(gridmap) {
    this->numPoint = path.poses.size() - 2;
    std::vector< CppAD::AD<double> > X_ad(this->numPoint*2);
    this->Xk.resize(this->numPoint*2);
    this->X0.resize(2);
    this->Xn.resize(2);
    for(int i=0;i<numPoint;i++)
    {
        X_ad[2*i] = path.poses.at(i+1).pose.position.x;
        X_ad[2*i+1] = path.poses.at(i+1).pose.position.y;
        this->Xk[2*i] = path.poses.at(i+1).pose.position.x;
        this->Xk[2*i+1] = path.poses.at(i+1).pose.position.y;
    }
    /// init cppAD
    CppAD::Independent(X_ad);
    std::vector< CppAD::AD<double> > Y(1);
    std::vector< CppAD::AD<double> > x0(2);
    std::vector< CppAD::AD<double> > xn(2);
    x0[0] = this->X0[0] = path.poses.at(0).pose.position.x;
    x0[1] = this->X0[1] = path.poses.at(0).pose.position.y;
    xn[0] = this->Xn[0] = path.poses.at(numPoint+1).pose.position.x;
    xn[1] = this->Xn[1] = path.poses.at(numPoint+1).pose.position.y;
    Y = Poly(X_ad,x0,xn);
    CppAD::ADFun<double> tempFun(X_ad,Y);
    this->Fun = tempFun;
    if (is_consider_boundary_) {
        /// init enviroment gradient map
        this->map_grad.setFrameId(gridmap.getFrameId());
        this->map_grad.setGeometry(gridmap.getLength(),
                                   gridmap.getResolution(),
                                   gridmap.getPosition());
        /// calculate gradient
        cv::Mat dis_field =
                hmpl::eigen2cv(this->gridmap_.get("distance"));
        cv::Mat grad_x = hmpl::eigen2cv(this->map_grad.get("grad_x"));
        cv::Mat grad_y = hmpl::eigen2cv(this->map_grad.get("grad_y"));
        cv::Mat kernal_x(2, 2, CV_32FC1);
        cv::Mat kernal_y(2, 2, CV_32FC1);
        kernal_y.at<float>(0, 0) = 1;
        kernal_y.at<float>(0, 1) = 0;
        kernal_y.at<float>(1, 0) = -1;
        kernal_y.at<float>(1, 1) = 0;
        kernal_x.at<float>(0, 0) = 1;
        kernal_x.at<float>(0, 1) = -1;
        kernal_x.at<float>(1, 0) = 0;
        kernal_x.at<float>(1, 1) = 0;
        cv::filter2D(dis_field, grad_x, -1, kernal_x, cv::Point(0, 0), 0);
        cv::filter2D(dis_field, grad_y, -1, kernal_y, cv::Point(0, 0), 0);
        // note: opencv's coordinate is different from eigen
        std::cout << "self solver init: " << numPoint << std::endl;
    }
}
CG_Solver::CG_Solver(const nav_msgs::Path &path,
                     const grid_map::GridMap &dis_map,
                     const grid_map::GridMap &grad_map) :
        gridmap_(dis_map),
        map_grad(grad_map) {
    this->numPoint = path.poses.size() - 2;
    std::vector< CppAD::AD<double> > X_ad(this->numPoint*2);
    this->Xk.resize(this->numPoint*2);
    this->X0.resize(2);
    this->Xn.resize(2);
    for (int i=0; i<numPoint; i++)
    {
        X_ad[2*i] = path.poses.at(i+1).pose.position.x;
        X_ad[2*i+1] = path.poses.at(i+1).pose.position.y;
        this->Xk[2*i] = path.poses.at(i+1).pose.position.x;
        this->Xk[2*i+1] = path.poses.at(i+1).pose.position.y;
    }
    /// init cppAD
    CppAD::Independent(X_ad);
    std::vector< CppAD::AD<double> > Y(1);
    std::vector< CppAD::AD<double> > x0(2);
    std::vector< CppAD::AD<double> > xn(2);
    x0[0] = this->X0[0] = path.poses.at(0).pose.position.x;
    x0[1] = this->X0[1] = path.poses.at(0).pose.position.y;
    xn[0] = this->Xn[0] = path.poses.at(numPoint+1).pose.position.x;
    xn[1] = this->Xn[1] = path.poses.at(numPoint+1).pose.position.y;
    Y = Poly(X_ad, x0, xn);
    CppAD::ADFun<double> tempFun(X_ad, Y);
    this->Fun = tempFun;
    this->w3_ = 0;
    this->w1_ = 10;
    this->w2_ = 1;
    this->is_consider_boundary_ = false;
    std::cout << "cg solver init over, point num is: "
              << this->numPoint << std::endl;
}
CG_Solver::CG_Solver() {

}

nav_msgs::Path CG_Solver::getSmoothPath() {
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();
    pose.header = path.header;
    for (size_t i = 0; i < this->Xk.size()/2; i++) {
        pose.pose.position.x = this->Xk[2*i];
        pose.pose.position.y = this->Xk[2*i+1];
        pose.pose.position.z = 0;
        path.poses.push_back(pose);
    }
    pose.pose.position.x = this->Xn[0];
    pose.pose.position.y = this->Xn[1];
    path.poses.push_back(pose);
    pose.pose.position.x = this->X0[0];
    pose.pose.position.y = this->X0[1];
    path.poses.insert(path.poses.begin(), pose);
    return path;
}

void CG_Solver::Solve() {
    std::vector<double > F(1);
    F = this->getFuncValue(this->Xk, this->X0, this->Xn);
    std::vector<double> g(this->Xk.size());
    this->getGradient(this->Xk, &g);
    std::vector<double >deltag(g.size());
    auto g_old = g;
    auto d = this->multi_(-1, g);
    double alpha = 0;
    double belta = 0;
    int count = 0;
    int restar_count = 0;
    int stop_count = 0;
    double F_old = 0;
    int solve_count = 0;
    std::cout << "ready to itration" << std::endl;
    for(int i=0; i<400; i++) {
        alpha = this->amijoSearch(this->Xk, d, g, F[0]);
        this->Xk = add_(this->Xk , multi_(alpha , d));
        F_old = F[0];
        F = getFuncValue(this->Xk, this->X0, this->Xn);
        std::cout << "alpha: " << alpha
                   << " F_old: " << F_old
                   << " F: " << F[0] << std::endl;
        if (std::fabs(F[0]-F_old) < 0.01) {
            stop_count ++;
            if (stop_count > 5)
                break;
        } else {
            stop_count = 0;
        }
        this->getGradient(this->Xk, &g);
        if(alpha < 1e-6) {
            if(count > 1) {
                std::cout<<i<<" no steps!"<<std::endl;
                break;
            }
            d = multi_(-1,g);
            i--;
            restar_count ++;
            count ++;
            continue;
        }
        deltag = add_(g, multi_(-1 , g_old));
        belta = std::max(0.0, dot_(g, deltag)/dot_(g_old, g_old));
        d = add_(multi_(-1, g), multi_(belta, d));
        count = 0;
        g_old = g;
        solve_count ++;
//        std:: cout << "solve_count: " << solve_count << std::endl;
    }
    std::cout
            << "solver over, restart count: "
            << restar_count
            << " solve count: "
            << solve_count << std::endl;
}

double CG_Solver::amijoSearch(const std::vector<double> &X,
                              const std::vector<double> &d,
                              const std::vector<double> &g,
                              double F) {
    double s = 0.1;
    double rho = 0.5;
    std::vector<double> X_i;
    std::vector<double> F_i(1);
    double value = 1e-6*dot_(g,d);
    while(s > 1e-6){
        X_i = add_(X , multi_(s , d));
        F_i = getFuncValue(X_i, this->X0, this->Xn);
        if(F_i[0] < F + s * value){
            return s;
        }
        s *= rho;
    }
    return 0.0;
}
double CG_Solver::stepLineSearch(const std::vector<double> &X,
                      const std::vector<double> &d,
                      const std::vector<double> &g,
                      double F) {

    double s_l = 0;
    double s_h = 0.5;
    double s1 = s_h - (s_h-s_l) * 0.618;
    double s2 = s_l + (s_h-s_l) * 0.618;
    double s = 0;

    double value = 0.01*dot_(g, d);
    while (s2 - s1 >1e-6)
    {
        std::vector<double> X1 = add_(X , multi_(s1 , d));
        std::vector<double> X2 = add_(X , multi_(s2 , d));

        std::vector<double> F1 = getFuncValue(X1, this->X0, this->Xn);
        std::vector<double> F2 = getFuncValue(X2, this->X0, this->Xn);

        if(F1[0] < F /*+ value*s1*/){
            s = s1;
            return s;
        }
        else if(F2[0] < F /*+ value*s2*/)
        {
            s = s2;
            return s;
        }
        if(F1[0] < F2[0])
        {
            s_h = s2;
            s1 = s_h - (s_h - s_l)*0.618;
            s2 = s_l + (s_h - s_l)*0.618;
        }
        else
        {
            s_l = s1;
            s1 = s_h - (s_h - s_l)*0.618;
            s2 = s_l + (s_h - s_l)*0.618;
        }
    }
    return s;
}

void CG_Solver::getGradient(const std::vector<double> &x,
                            std::vector<double> *g) {
    *g = this->Fun.Jacobian(x);
    if(this->is_consider_boundary_){

        for(size_t i = 0; i < this->numPoint; i++){
            grid_map::Position pt(x.at(2 * i), x.at(2 * i + 1));
            if(!this->gridmap_.isInside(pt)){
                std::cout<<"grad: out side!"<<std::endl;
                continue;
            }
            double grad_x = this->map_grad.atPosition("grad_x", pt);
            double grad_y = this->map_grad.atPosition("grad_y", pt);

            g->at(2 * i) += -grad_x;
            g->at(2 * i + 1) += -grad_y;
        }
    }
}

std::vector<double> CG_Solver::getFuncValue(const std::vector<double> &x,
                                             const std::vector<double> &x0,
                                             const std::vector<double> &xn) {
    std::vector<double> fun1(1);
    fun1 = this->Poly(x, x0, xn);
    double fun2 = 0.0;
    if(this->is_consider_boundary_) {
        for (size_t i = 0; i < this->numPoint; i++) {
            grid_map::Position pt(x.at(2 * i), x.at(2 * i + 1));
            if(!this->gridmap_.isInside(pt)) {
//                std::cout<<"fun: out side: "<<i<<std::endl;
                continue;
            }
            double dis = this->gridmap_.atPosition("distance", pt);

            fun2 += this->w3_ * dis;
        }
    }
    fun1[0] += fun2;
    return fun1;
}

