//
// Created by yangt on 18-11-1.
//
#include "path_smoothing/chomp.hpp"
ChompOptimizer::ChompOptimizer(std::vector<geometry_msgs::Point> init_guses,
                               grid_map::GridMap &signedDistanceField)
        : cdim_(2),
          n_(init_guses.size() - 2),
          xdim_(cdim_ * n_),
          q0_(Vector::Zero(cdim_)),
          qe_(Vector::Zero(cdim_)),
          qi_(Vector::Zero(xdim_)),
          K_(Matrix::Zero(xdim_ + cdim_, xdim_)),
          e_(Vector::Zero(xdim_ + cdim_)),
          sdf_(signedDistanceField),
          sdfGradient_(grid_map::GridMap(
                  std::vector<std::string>{"grad_x", "grad_y"})) {
    if(n_ <= 0) {
        std::cout << "please input an logical initial guess!!!\n";
        return;
    }
    this->dt_ = 1.0 / (n_ + 1);
    this->q0_ << init_guses.front().x, init_guses.front().y;
    this->qe_ << init_guses.back().x, init_guses.back().y;

    for(size_t i(0); i < n_ + 1; ++i) {
        if(i < n_) {
            this->qi_(i * cdim_) = init_guses.at(i + 1).x;
            this->qi_(i * cdim_ + 1) = init_guses.at(i + 1).y;
            K_.block(cdim_ * i, cdim_ * i, cdim_, cdim_) =
                    Matrix::Identity(cdim_, cdim_);
        }
        if(i > 0) {
            K_.block(cdim_ * i, cdim_ * (i - 1), cdim_, cdim_) =
                    -1.0 * Matrix::Identity(cdim_, cdim_);
        }
    }
    this->K_ /= dt_;

    // init e
    this->e_.block(0, 0, cdim_, 1) = -q0_;
    this->e_.block(xdim_ , 0, cdim_, 1) = qe_;
    this->e_ /= dt_;

    /// calculate gradient
    this->sdfGradient_.setFrameId(sdf_.getFrameId());
    this->sdfGradient_.setGeometry(sdf_.getLength(),
                                   sdf_.getResolution(),
                                   sdf_.getPosition());
    cv::Mat dis_field =
            hmpl::eigen2cv(this->sdf_.get("gridMap"));
    cv::Mat grad_x = hmpl::eigen2cv(this->sdfGradient_.get("grad_x"));
    cv::Mat grad_y = hmpl::eigen2cv(this->sdfGradient_.get("grad_y"));
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
    grad_x /= sdf_.getResolution();
    grad_y /= sdf_.getResolution();
}

double ChompOptimizer::getObstacleCost(const Vector &xi) {
    grid_map::Position pt(xi(0), xi(1));
    if(this->sdf_.isInside(pt)) {
        return sdf_.atPosition("gridMap", pt);
    } else {
        Eigen::Vector2d dp;
        dp(0) = fabs(pt(0)) - sdf_.getLength().x() / 2.0;
        dp(1) = fabs(pt(1)) - sdf_.getLength().y() / 2.0;
        if(dp(0) > 0 && dp(1) > 0) {
            return dp.norm();
        } else if(dp(0) > 0) {
            return dp(0);
        } else if(dp(1) > 1) {
            return dp(1);
        }
    }
}

Vector ChompOptimizer::getObstacleCostGradient(const Vector &xi) {
    grid_map::Position pt(xi(0), xi(1));
    Vector gradient = Vector::Zero(cdim_);
    if(this->sdf_.isInside(pt)) {
        gradient(0) = sdfGradient_.atPosition("grad_x", pt);
        gradient(1) = sdfGradient_.atPosition("grad_y", pt);
    } else {
        auto dp = pt - sdf_.getPosition();
        double x_len =  sdf_.getLength().x() / 2.0;
        double y_len = sdf_.getLength().y() / 2.0;
        if(dp(0) > x_len && dp(1) > y_len) {
            gradient(0) = dp(0) - x_len;
            gradient(1) = dp(1) - y_len;
            gradient /= gradient.norm();
        } else if(dp(0) > x_len && fabs(dp(1)) < y_len) {
            gradient(0) = 1;
        } else if(dp(0) > x_len && dp(1) < -y_len) {
            gradient(0) = dp(0) - x_len;
            gradient(1) = dp(1) + y_len;
            gradient /= gradient.norm();
        } else if(fabs(dp(0)) < x_len && dp(1) < -y_len) {
            gradient(1) = -1;
        } else if(dp(0) < -x_len && dp(1) < -y_len) {
            gradient(0) = dp(0) + x_len;
            gradient(1) = dp(1) + y_len;
            gradient /= gradient.norm();
        } else if(dp(0) < -x_len && fabs(y_len) < y_len) {
            gradient(0) = -1;
        } else if(dp(0) < -x_len && dp(1) > y_len) {
            gradient(0) = dp(0) + x_len;
            gradient(1) = dp(1) - y_len;
            gradient /= gradient.norm();
        } else if (fabs(dp(0)) < x_len && y_len > y_len) {
            gradient(1) = 1;
        }
    }
    return gradient;
}

double ChompOptimizer::chompIteration(Vector *x, double *obs_cost,
                                      double namda) {
    // beginning of "the" CHOMP iteration
    const Matrix A = K_.transpose() * K_ ;
    const Vector b = K_.transpose() * e_ ;
    Vector nabla_smooth(dt_* (A * (*x) + b));
    Vector xdd(A * (*x) + b); // indeed, it is the same in this
    // formulation...

    Vector nabla_obs(Vector::Zero(xdim_));

    *obs_cost = 0.0;
    auto yt =
            (dt_ * (0.5 * (*x).transpose() * A * (*x) +
                    (*x).transpose() * b +
                    0.5 * e_.transpose() * e_));
    Vector xd = this->K_ * (*x) + this->e_;
    for (size_t i(0); i < n_; ++i) {
        Vector const &xi = x->block(i * cdim_, 0, cdim_, 1);

        // In this case, C and W are the same, Jacobian is identity.  We
        // still write more or less the full-fledged CHOMP expressions
        // (but  we only use one body point) to make subsequent extension
        // easier.
        //
        Vector const & xdi = xd.block(i * cdim_, 0, cdim_, 1);
        Matrix const JJ(Matrix::Identity(cdim_, cdim_));        // a little silly
        // here,
        // as noted above.
        double const vel(xdi.norm());
        if (vel < 1.0e-3)                               // avoid div by zero further down
            continue;
        Vector const xdin(xdi / vel);
        Vector const xddi(JJ * xdd.block(i * cdim_, 0, cdim_, 1));
        Matrix const prj(Matrix::Identity(cdim_, cdim_) -
                                 xdin * xdin.transpose());
        // hardcoded planar case
        Vector const kappa(prj * xddi / pow(vel, 2.0));

        const double c = this->getObstacleCost(xi);
        *obs_cost += c;
        Vector nabla_c = this->getObstacleCostGradient(xi);
        nabla_obs.block(i * cdim_, 0, cdim_, 1) = JJ.transpose() * vel *
                (prj * nabla_c - c * kappa);
    }
//    std::cout << "Jsmooth: " << smooth_cost << ", Jobs: " << obs_cost << "\n";
    Vector dxi(A.inverse() * (nabla_obs + 0*namda * nabla_smooth));
    (*x) -= dxi / 1.5;
    //return the error (in Euclidean sense ). Remeber that the difference is -dxi/eta
    return dxi.norm() / 1.5;

    // end of "the" CHOMP iteration
    //////////////////////////////////////////////////
}

nav_msgs::Path ChompOptimizer::generatePath() {
    double err = std::numeric_limits<double>::infinity();
    size_t optimize_count = 0;
    double obs_cost = std::numeric_limits<double>::infinity();
    double namda = 0.001;
    while(fabs(err) > 0.001 && optimize_count < 300) {
        if(obs_cost > 6) {
            namda = 0.001;
        } else {
            namda = 0.1;
        }
        err = this->chompIteration(&qi_, &obs_cost, namda);
        optimize_count ++;
//
    }
    std::cout << "count: " << optimize_count <<", err:" << err << "\n";
    nav_msgs::Path final_path;
    geometry_msgs::PoseStamped tmp_pose;
    final_path.header.frame_id = "odom";
    final_path.header.stamp = ros::Time::now();
    tmp_pose.header = final_path.header;
    for(size_t i(0); i < n_; ++i) {
        tmp_pose.pose.position.x = this->qi_(i * cdim_);
        tmp_pose.pose.position.y = this->qi_(i * cdim_ + 1);
        final_path.poses.push_back(tmp_pose);
    }
    return final_path;
}