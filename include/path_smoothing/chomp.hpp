//
// Created by yangt on 18-11-1.
//

#ifndef PATH_SMOOTHING_CHOMP_HPP
#define PATH_SMOOTHING_CHOMP_HPP

#include <geometry_msgs/Point.h>
//#include <Eigen/Core>
#include <Eigen/Dense>
#include <internal_grid_map/internal_grid_map.hpp>
#include <nav_msgs/Path.h>
typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
class ChompOptimizer {
 public:
    //! A normal member taking two arguments and returning an integer value.
    /**
     *
     * @param init_guses
     * @param signedDistanceField
     */
    ChompOptimizer(std::vector<geometry_msgs::Point> init_guses,
                   grid_map::GridMap &signedDistanceField);
    nav_msgs::Path generatePath();
 private:
    //! number of sampled points
    size_t n_;

    //! dimension of configuration space
    /*!
     * detailed description
     */
    size_t cdim_;

    // dimension of trajectory
    size_t xdim_;

    // discrete interval
    double dt_;

    // start configuration
    Vector q0_;

    // goal configuration
    Vector qe_;

    // discrete path
    Vector qi_;

    // difference Matrix
    Matrix K_;
    Matrix e_;

    // signed distance field
    grid_map::GridMap & sdf_;

    // gradient of distance field
    grid_map::GridMap sdfGradient_;

    double chompIteration(Vector *x, double *obs_cost,
                          double namda);
    double getObstacleCost(const Vector &xi);
    Vector getObstacleCostGradient(const Vector &xi);
};

#endif //PATH_SMOOTHING_CHOMP_HPP
