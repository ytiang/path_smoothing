//
// Created by yangt on 19-2-25.
//


#include "path_smoothing/path_smoothing.hpp"

namespace path_smoothing {

PathSmoothing::PathSmoothing(const int path_size) : path_size_(path_size) {

}

PathSmoothing *PathSmoothing::createSmoother(const path_smoothing::PathSmoothing::Options &options,
                                             const std::vector<geometry_msgs::Point> &path) {
    CHECK_GT(path.size(), 2) << "path contains less than 3 points!";
    switch (options.smoother_type) {
        case CONJUGATE_GRADIENT_METHOD: {
            return new CgSmoothing(options, path);
        }
        case GAUSS_PROCESS_METHOD: {
#ifdef GPMP2_SMOOTHING_ENABLE
            return new GpSmoothing(options, path);
#else
            LOG(ERROR)
                << "gpmp2 smoothing is not supported unless you have installed gtsam and gpmp2 libraries!!";
            return nullptr;
#endif
        }
    }
}

void PathSmoothing::getSmoothPath(std::vector<geometry_msgs::Point> *path) const {
    path->clear();
    geometry_msgs::Point point;
    std::vector<double> ctrlp;
    if (pathSize() < 4) {
        LOG(WARNING)
            << "path size is less than 4, failed interpolate with spline!";
        for (int i(0); i < pathSize(); ++i) {
            path->at(i).x = x(i);
            path->at(i).y = y(i);
            path->push_back(point);
        }
        return;
    }
    for (int i(0); i < pathSize(); ++i) {
        ctrlp.push_back(x(i));
        ctrlp.push_back(y(i));
    }
    size_t ctrlpt_num = ctrlp.size() / 2;
    size_t degree = pathSize() > 7 ? 6 : pathSize() - 1;
    tinyspline::BSpline clamped_spline(ctrlpt_num, 2, degree);
    clamped_spline.setControlPoints(ctrlp);

    std::size_t sample_num = std::max((std::size_t) 100, ctrlpt_num * 5);
    for (std::size_t j = 0; j < sample_num; j++) {
        double size_f = static_cast<double>(sample_num - 1);
        double knot_percent =
            static_cast<double>(j) / size_f;  // range: [0, 1]

        point.x = clamped_spline.eval(knot_percent).result().at(0);
        point.y = clamped_spline.eval(knot_percent).result().at(1);
        path->push_back(point);
    }
}

CgSmoothing::CgSmoothing(const Options &options,
                         const std::vector<geometry_msgs::Point> &path)
    : PathSmoothing(path.size()) {
    settings_.heading_term_coe = options.cg_heading_term_coe;
    settings_.curvature_term_coe = options.cg_curvature_term_coe;
    settings_.obstacle_term_coe = options.cg_obstacle_term_coe;
    settings_.type = options.cg_difference_type;
    settings_.degree = 2;
    settings_.param_num = (path.size() - 2) * settings_.degree;
    settings_.start.resize(settings_.degree);
    settings_.start << path.front().x, path.front().y;
    settings_.end.resize(settings_.degree);
    settings_.end << path.back().x, path.back().y;
    settings_.function_ = options.function;

    params_.resize(settings_.param_num);
    for (int i(1); i < path.size() - 1; ++i) {
        const int j = i - 1;
        params_(j * settings_.degree) = path.at(i).x;
        params_(j * settings_.degree + 1) = path.at(i).y;
    }
}

void CgSmoothing::smoothPath(const Options &options) {
    CgSmoothingFunction *smooth_function =
        CgSmoothingFunction::createCgSmoothingFunction(settings_,
                                                       params_);
    switch (options.cg_solver) {
        case CERES_SOLVER: {
            ceres::GradientProblemSolver::Options option;
            ceres::GradientProblemSolver::Summary summary;
            ceres::GradientProblem problem(smooth_function);
            option.logging_type = ceres::SILENT;
            option.nonlinear_conjugate_gradient_type = ceres::FLETCHER_REEVES;
            option.line_search_interpolation_type = ceres::QUADRATIC;
            option.line_search_type = ceres::WOLFE;
            option.line_search_sufficient_function_decrease = 1e-4;
            option.line_search_sufficient_curvature_decrease = 0.2;
//            option.min_line_search_step_contraction = 0.92;
            option.max_line_search_step_contraction = 1e-4;
            option.line_search_direction_type =
                ceres::NONLINEAR_CONJUGATE_GRADIENT;//ceres::STEEPEST_DESCENT; //
            option.max_num_iterations = 20;
            ceres::Solve(option, problem, params_.data(), &summary);
//            std::cout << summary.FullReport();
            break;
        }
        case SELF_SOLVER: {
            ncopt::GradientProblemOption solver_options;
            ncopt::Summary summarys;
            ncopt::GradientProblemSolver solver(smooth_function);
            solver.Solve(params_.data(), solver_options, &summarys);
            ncopt::Summary::PrintSummary(summarys);
            break;
        }
    }
}

double CgSmoothing::x(int i) const {
    if (i > 0 && i < pathSize() - 1) {
        return params_((i - 1) * settings_.degree);
    } else if (i == 0) {
        return settings_.start(0);
    } else if (i == pathSize() - 1) {
        return settings_.end(0);
    }
}

double CgSmoothing::y(int i) const {
    if (i > 0 && i < pathSize() - 1) {
        return params_((i - 1) * settings_.degree + 1);
    } else if (i == 0) {
        return settings_.start(1);
    } else if (i == pathSize() - 1) {
        return settings_.end(1);
    }
}

#ifdef GPMP2_SMOOTHING_ENABLE

GpSmoothing::GpSmoothing(const Options &options,
                         const std::vector<geometry_msgs::Point> &path)
    : PathSmoothing(path.size()) {
    // set robot model
    gtsam::Vector3 zero_vec(0.0, 0.0, 0.0);
    gpmp2::BodySphere sphere(0, 1.2, gtsam::Point3(zero_vec));
    gpmp2::BodySphereVector sphere_vec;
    sphere_vec.push_back(sphere);
    gpmp2::Pose2MobileBaseModel robot(gpmp2::Pose2MobileBase(), sphere_vec);

    //set noise model:
    auto Qc_model =
        gtsam::noiseModel::Gaussian::Covariance(Eigen::Matrix3d::Identity());
    auto pose_fix = gtsam::noiseModel::Isotropic::Sigma(robot.dof(), 1e-5);
    auto vel_fix = gtsam::noiseModel::Isotropic::Sigma(robot.dof(), 1e-5);

    gtsam::Vector avg_vel(3);
    avg_vel << path.back().x - path.front().x,
        path.back().y - path.front().y, 0;
    avg_vel = avg_vel / pathSize();

    //set initial values and build graph
    for (int i = 0; i < pathSize(); ++i) {
        gtsam::Key pose_key = i;
        gtsam::Key vel_key = i + pathSize();
        double heading;
        if (i == pathSize() - 1) {
            heading = atan2(path.at(i).y - path.at(i - 2).y,
                            path.at(i).x - path.at(i - 2).x);
        } else {
            heading = atan2(path.at(i + 1).y - path.at(i).y,
                            path.at(i + 1).x - path.at(i).x);
        }
        gtsam::Pose2 current_pose(path.at(i).x, path.at(i).y, heading);
        initial_guess.insert(pose_key, current_pose);
        initial_guess.insert(vel_key, avg_vel);
        // start and goal fix factor
        if (i == 0) {
            graph_.add(gtsam::PriorFactor<gtsam::Pose2>(pose_key,
                                                        current_pose,
                                                        pose_fix));
            graph_.add(gtsam::PriorFactor<gtsam::Vector>(vel_key,
                                                         gtsam::Vector::Zero(3),
                                                         vel_fix));
        } else if (i == pathSize() - 1) {
            graph_.add(gtsam::PriorFactor<gtsam::Pose2>(pose_key,
                                                        current_pose,
                                                        pose_fix));
            graph_.add(gtsam::PriorFactor<gtsam::Vector>(vel_key,
                                                         gtsam::Vector::Zero(3),
                                                         vel_fix));
        }
        // obstacle factor
        graph_.add(gpmp2::ObstacleFactor<gpmp2::Pose2MobileBaseModel>(pose_key,
                                                                      robot,
                                                                      *(options.function),
                                                                      options.gp_obs_sigma));
        // vehicle dynamic:
        graph_.add(gpmp2::VehicleDynamicsFactorPose2(pose_key,
                                                     vel_key,
                                                     options.gp_vehicle_dynamic_sigma));
        if (i > 0) {
            gtsam::Key pose_key0 = pose_key - 1;
            gtsam::Key vel_key0 = vel_key - 1;
            //GP priors
            graph_.add(gpmp2::GaussianProcessPriorPose2(pose_key0,
                                                        vel_key0,
                                                        pose_key,
                                                        vel_key,
                                                        options.gp_dt,
                                                        Qc_model));
        }
    }
}

void GpSmoothing::smoothPath(const Options &options) {
    switch (options.gp_solver) {
        case GAUSS_NEWTON: {
            gtsam::GaussNewtonParams params;
            params.setVerbosity("SILENT");
            gtsam::GaussNewtonOptimizer
                optimizer(graph_, initial_guess, params);
            optimizer.optimize();
            result_ = optimizer.values();
            return;
        }
        case LEVENBERG_MARQUARDT: {
            gtsam::LevenbergMarquardtParams params;
            params.setVerbosity("SILENT");
            gtsam::LevenbergMarquardtOptimizer
                optimizer(graph_, initial_guess, params);
            optimizer.optimize();
            result_ = optimizer.values();
            return;
        }
        case DOGLEG: {
            gtsam::DoglegParams params;
            params.setVerbosity("SILENT");
            gtsam::DoglegOptimizer optimizer(graph_, initial_guess, params);
            optimizer.optimize();
            result_ = optimizer.values();
        }
    }
}

double GpSmoothing::x(int i) const {
    return result_.at<gtsam::Pose2>(i).x();
}
double GpSmoothing::y(int i) const {
    return result_.at<gtsam::Pose2>(i).y();
}
#endif
}