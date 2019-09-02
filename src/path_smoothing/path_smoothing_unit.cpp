//
// Created by yangt on 2019/9/2.
//
#include "path_smoothing/path_smoothing_unit.hpp"
namespace path_smoothing {
template<>
double &xRef(geometry_msgs::Pose &point) {
    return point.position.x;
}

template<>
double &yRef(geometry_msgs::Pose &point) {
    return point.position.y;
}

template<>
const double &xRef(const geometry_msgs::Pose &point) {
    return point.position.x;
}

template<>
const double &yRef(const geometry_msgs::Pose &point) {
    return point.position.y;
}

}