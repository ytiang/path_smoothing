//
// Created by yangt on 2019/9/2.
//

#ifndef PATH_SMOOTHING_PATH_SMOOTHING_UNIT_HPP
#define PATH_SMOOTHING_PATH_SMOOTHING_UNIT_HPP

#include <geometry_msgs/PoseStamped.h>

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
const double &xRef(const geometry_msgs::Pose &point);

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
double &yRef(geometry_msgs::Pose &point);
}

#endif //PATH_SMOOTHING_PATH_SMOOTHING_UNIT_HPP
