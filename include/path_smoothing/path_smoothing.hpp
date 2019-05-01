//
// Created by yangt on 19-2-18.
//

#ifndef PATH_SMOOTHING_PATH_SMOOTHING_HPP
#define PATH_SMOOTHING_PATH_SMOOTHING_HPP
#ifdef GPMP2_SMOOTHING_ENABLE
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gpmp2/dynamics/VehicleDynamicsFactorPose2.h>
#include <gpmp2/gp/GaussianProcessPriorPose2.h>
#include <gpmp2/kinematics/Pose2MobileBaseModel.h>
#include "path_smoothing/obstacle_factor.hpp"
#endif

#include <opt_utils/opt_utils.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tinyspline_ros/tinysplinecpp.h>

#include "path_smoothing/cg_smoothing_function.hpp"
#include "non_constrained_optimiztion/gradient_problem_solve.hpp"

namespace path_smoothing {

/**
 * static method for access the x value of a Point
 * @tparam ReturnType requires basic type, like double , int
 * @tparam PathElement requires point type containing x value and y value
 * @param point
 * @return the reference of x value of input point
 */
template<class ReturnType, class PathElement>
ReturnType &xRef(PathElement &point) {
    return point.x;
}

template<class ReturnType, class PathElement>
const ReturnType &xRef(const PathElement &point) {
    return point.x;
}
template<>
double &xRef(geometry_msgs::Pose &point);
template<>
double &xRef(hmpl::Circle &point);
template<>
const double &xRef(const geometry_msgs::Pose &point);
template<>
const double &xRef(const hmpl::Circle &point);

/**
 * static method for access the y value of a Point
 * @tparam ReturnType requires basic type, like double , int
 * @tparam PathElement requires point type containing x value and y value
 * @param point
 * @return the reference of y value of input point
 */
template<class ReturnType, class PathElement>
ReturnType &yRef(PathElement &point) {
    return point.y;
}
template<class ReturnType, class PathElement>
const ReturnType &yRef(const PathElement &point) {
    return point.y;
}
template<>
const double &yRef(const geometry_msgs::Pose &point);
template<>
const double &yRef(const hmpl::Circle &point);
template<>
double &yRef(geometry_msgs::Pose &point);
template<>
double &yRef(hmpl::Circle &point);

class PathSmoothing {
 public:
    struct Options {
        SmootherType smoother_type = CONJUGATE_GRADIENT_METHOD;
        // options for conjugate gradient method
        double cg_heading_term_coe = 1;
        double cg_curvature_term_coe = 1.0;
        double cg_obstacle_term_coe = 2.0;
        DifferenceType cg_difference_type = CPPAD;
        NonlinearSolverType cg_solver = CERES_SOLVER;
        // options for gp method
        double gp_obs_sigma = 0.05;
        double gp_vehicle_dynamic_sigma = 0.00;
        double gp_dt = 0.5;
        LeastSquaresSolver gp_solver = LEVENBERG_MARQUARDT;
        // options for non-derivative method
        double lower_boundary = 1.0;
        double safe_margin = 0.2;
        double max_curvature = 0.4;

        // options for signed distance field
        DistanceFunction2D *function = nullptr;
    };

    PathSmoothing(const int path_size);

    template<class PathElement>
    static PathSmoothing *createSmoother(const Options &options,
                                         const std::vector<PathElement> &path);

    template<class PathElement>
    static std::vector<hmpl::Circle> convertToCirclePath(
            const Options &options, const std::vector<PathElement> &path);

    template<class PathElement>
    void getSmoothPath(std::vector<PathElement> *path) const;

    inline const int &pathSize() const {
        return path_size_;
    }

    virtual void smoothPath(const Options &options) = 0;
    virtual double x(int i) const = 0;
    virtual double y(int i) const = 0;

    virtual ~PathSmoothing() {}

 private:
    const int path_size_;
};

class CgSmoothing : public PathSmoothing {
 public:

    template<class PathElement>
    CgSmoothing(const Options &options, const std::vector<PathElement> &path);

    virtual double x(int i) const;

    virtual double y(int i) const;

    virtual void smoothPath(const Options &options);
 private:
    CgSmoothingFunction::Settings settings_;
    CgSmoothingFunction::Vector params_;
};

class NonDerivativeSmoothing : public PathSmoothing {
 public:
    typedef hmpl::Circle Circle;
    typedef hmpl::Circle &CircleRef;
    typedef hmpl::Circle *CirclePtr;

    NonDerivativeSmoothing(const Options &option,
                           const std::vector<Circle> &circle_path);

    void optimizePathLength();

    void optimizePathImproved();

    void updateCircleCenter(const CircleRef parent,
                            const CircleRef first,
                            CirclePtr second,
                            const CircleRef third);

    void updateCircleCenterWithoutLimit(const CircleRef first,
                                        CirclePtr second,
                                        const CircleRef third);

    Circle getPerpendicularCircle(const CircleRef first,
                                  const CircleRef second,
                                  const CircleRef third);

    double getLengthOfPath() const;

    double getCirclePathEnergy() const;

    void smoothPath(const Options &options);

    virtual double x(int i) const {
        return circle_path_.at(i).position.x;
    }

    virtual double y(int i) const {
        return circle_path_.at(i).position.y;
    }

 private:
    std::vector<Circle> circle_path_;
    DistanceFunction2D *distance_func_;
    double lower_boundary_;
};

#ifdef GPMP2_SMOOTHING_ENABLE
class GpSmoothing : public PathSmoothing {
 public:
  template<class PathElement>
  GpSmoothing(const Options &options, const std::vector<PathElement> &path);

  virtual void smoothPath(const Options &options);

  virtual double x(int i) const;
  virtual double y(int i) const;

 private:
  gtsam::Values result_;
  gtsam::Values initial_guess;
  gtsam::NonlinearFactorGraph graph_;
};
#endif

}

#include "path_smoothing-in.hpp"

#endif //PATH_SMOOTHING_PATH_SMOOTHING_HPP
