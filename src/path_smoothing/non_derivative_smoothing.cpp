//
// Created by yangt on 19-4-25.
//

#include "path_smoothing/path_smoothing.hpp"
#include <opt_utils/base.hpp>

namespace path_smoothing {
using namespace hmpl;
NonDerivativeSmoothing::NonDerivativeSmoothing(const Options &options,
                                               const std::vector<hmpl::CircleNode *> &circle_path)
        : PathSmoothing(circle_path.size()),
          circle_path_(circle_path),
          options_(options) {
}

void NonDerivativeSmoothing::smoothPath(const Options &options) {
    this->optimizeLength();
}

hmpl::Circle NonDerivativeSmoothing::getPerpendicularCircle(
        const CircleRef first, const CircleRef second,
        const CircleRef third) {
    Circle c;
    double t = 0;
    double x1 = first.position.x;
    double y1 = first.position.y;
    double x2 = second.position.x;
    double y2 = second.position.y;
    double x3 = third.position.x;
    double y3 = third.position.y;

    // parameterize the point between the start point and the end point
    double a = (x3 - x1) * (x1 - x2) + (y3 - y1) * (y1 - y2);
    double b = pow(y3 - y1, 2.0) + pow(x3 - x1, 2.0);
    t = -a / b;
    // perpendicular point
    c.position.x = x1 * (1 - t) + x3 * t;
    c.position.y = y1 * (1 - t) + y3 * t;
    grid_map::Position pos(c.position.x, c.position.y);
    c.r = distance_func_->getObstacleDistance(pos);

    return c;
}

void NonDerivativeSmoothing::updateCircleCenter(const CircleRef parent,
                                                const CircleRef first,
                                                CirclePtr second,
                                                const CircleRef third) {
    hmpl::Circle perpendicular = getPerpendicularCircle(first, *second, third);
    /* // the code section doesn't work as expect disable now
    if (getCurvature(parent.position, first.position, second->position) >
    this->curvature_max_) {
        // get the straight line support point
        double heading1 = atan2(first.position.y - parent.position.y,
                               first.position.x - parent.position.x);
        double heading2 = atan2(second->position.y - first.position.y,
                               second->position.x - parent.position.x);
        // get the current heading change
        double delta_heading = (heading2 - heading1) / 2.0;


        // binary search on the arc to find an update rule
        do {

            astar::Circle loose;
            loose.position.x = first.position.x + cos(delta_heading +
    heading1)*first.r;
            loose.position.y = first.position.y + sin(delta_heading +
    heading1)*first.r;
            grid_map::Position pos(loose.position.x, loose.position.y);
            double clearance = internal_grid.getObstacleDistance(pos);
            if (getCurvature(parent.position, first.position, loose.position) <
    this->curvature_max_
                    && clearance > this->lower_boundary_ + safety_margin_
                    ) {
                *second = loose;
                break;
            } else {
                delta_heading = delta_heading / 2.0;
            }
        } while (delta_heading > 0.01);

        // update the point
    }*/
    while (second->position.Distance(perpendicular.position) > 0.00001) {
        grid_map::Position pos(perpendicular.position.x,
                               perpendicular.position.y);
        double clearance = distance_func_->getObstacleDistance(pos);
        if (clearance > options_.lower_boundary + options_.safe_margin &&
                hmpl::getCurvature(parent.position, first.position,
                                   perpendicular.position) <=
                        options_.max_curvature) {
            *second = perpendicular;
        } else {
            perpendicular.position.x =
                    (perpendicular.position.x + second->position.x) / 2.0;
            perpendicular.position.y =
                    (perpendicular.position.y + second->position.y) / 2.0;
            perpendicular.r = clearance;
        }
    }
}

void NonDerivativeSmoothing::updateCircleCenterWithoutLimit(
        const CircleRef first, CirclePtr second,
        const CircleRef third) {
    Circle perpendicular = getPerpendicularCircle(first, *second, third);
    while (second->position.Distance(perpendicular.position) > 0.00001) {
        grid_map::Position pos(perpendicular.position.x,
                               perpendicular.position.y);
        double clearance = distance_func_->getObstacleDistance(pos);
        if (clearance > options_.lower_boundary + options_.safe_margin) {
            *second = perpendicular;
        } else {
            perpendicular.position.x =
                    (perpendicular.position.x + second->position.x) / 2.0;
            perpendicular.position.y =
                    (perpendicular.position.y + second->position.y) / 2.0;
            perpendicular.r = clearance;
        }
    }
}

void NonDerivativeSmoothing::optimizePathLength() {
    std::size_t size = this->circle_path_.size();
    if (size > 3) {
        double length, new_length;
        do {
            length = this->getLengthOfPath();
            for (std::size_t i = 0; i < size - 2; i++) {
                CircleRef first =
                        this->circle_path_.at(i)->circle;
                CircleRef second =
                        this->circle_path_.at(i + 1)->circle;
                CircleRef third =
                        this->circle_path_.at(i + 2)->circle;
                updateCircleCenterWithoutLimit(first, &second, third);
            }
            new_length = this->getLengthOfPath();
            double improvement = fabs(new_length - length) / length;

            if (improvement == 0) {
//                this->optimized_ = true;
                break;
            }

            if (new_length < length && improvement < 0.00000001) {
//                this->optimized_ = true;
                break;
            }
        } while (new_length < length);
    }
}

void NonDerivativeSmoothing::optimizePathImproved() {
    std::size_t size = circle_path_.size();
    if (size > 3) {
        double energy, new_energy;
        do {
            energy = this->getCirclePathEnergy();
            for (std::size_t i = 1; i < size - 2; i++) {
                CircleRef parent =
                        this->circle_path_.at(i - 1)->circle;
                CircleRef first =
                        this->circle_path_.at(i)->circle;
                CircleRef second =
                        this->circle_path_.at(i + 1)->circle;
                CircleRef third =
                        this->circle_path_.at(i + 2)->circle;

                updateCircleCenter(parent, first, &second, third);
                // updateCircleCenterWithoutLimit(parent, first, &second,
                // third);
            }
            new_energy = this->getCirclePathEnergy();
            double improvement = fabs(new_energy - energy) / energy;

            if (improvement == 0) {
//                this->optimized_ = true;
                break;
            }

            if (new_energy < energy && improvement < 0.0001) {
//                this->optimized_ = true;
                break;
            }
        } while (new_energy < energy);
    }
}

double NonDerivativeSmoothing::getCirclePathEnergy() {
    std::size_t size = this->circle_path_.size();
    if (size < 3) {
        std::cout << "The size of the circle path points is below 3. "
                "Can't get smoothness.";
        return 0;
    }

    double f = 0;
    double smoothness = 0;
    double length = 0;
    double energy = 0;
    for (std::size_t i = 1; i < size - 2; i++) {
        Vector2D<double> &first =
                this->circle_path_.at(i - 1)->circle.position;
        Vector2D<double> &second =
                this->circle_path_.at(i)->circle.position;
        Vector2D<double> &third =
                this->circle_path_.at(i + 1)->circle.position;

        smoothness = this->getSmoothness(first, second, third);
        length = second.Distance(first) + third.Distance(second);

        energy = pow(smoothness, 2.0) * length;
        f = f + energy;
    }
    return f;
}

double NonDerivativeSmoothing::getSmoothness(Vector2D<double> &first,
                                             Vector2D<double> &second,
                                             Vector2D<double> &third) {
    return hmpl::getCurvature(first, second, third);
}

double NonDerivativeSmoothing::getLengthOfPath() {
    std::size_t size = this->circle_path_.size();
    double length = 0;
    if (size > 1) {
        for (std::size_t i = 0; i < size - 2; i++) {
            double x_1 = this->circle_path_.at(i)->circle.position.x;
            double x_2 = this->circle_path_.at(i + 1)->circle.position.x;
            double y_1 = this->circle_path_.at(i)->circle.position.y;
            double y_2 = this->circle_path_.at(i + 1)->circle.position.y;
            double delta =
                    std::sqrt((x_1 - x_2) * (x_1 - x_2)
                                      + (y_1 - y_2) * (y_1 - y_2));
            length += delta;
        }
        return length;
    } else {
        LOG(WARNING) << "The circle path size is 0 or 1.";
        return length;
    }
}

}