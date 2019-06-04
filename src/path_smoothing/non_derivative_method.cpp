//
// Created by yangt on 19-4-27.
//
#include "path_smoothing/path_smoothing.hpp"
namespace path_smoothing {

NonDerivativeSmoothing::NonDerivativeSmoothing(const Options &option,
                                               const std::vector<Circle> &circle_path)
        : PathSmoothing(circle_path.size()) {
    this->circle_path_ = circle_path;
    this->distance_func_ = option.function;
    this->lower_boundary_ = option.lower_boundary;
    CHECK(distance_func_ != nullptr)
    << "non-derivative method requires an distance funcion!";

}

hmpl::Circle NonDerivativeSmoothing::getPerpendicularCircle(const CircleRef first,
                                                            const CircleRef second,
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
    c.r = this->distance_func_->getObstacleDistance(pos);

    return c;
}

void NonDerivativeSmoothing::updateCircleCenter(const CircleRef parent,
                                                const CircleRef first,
                                                CirclePtr second,
                                                const CircleRef third) {
    Circle perpendicular = getPerpendicularCircle(first, *second, third);
    while (second->position.Distance(perpendicular.position) > 0.00001) {
        grid_map::Position pos(perpendicular.position.x,
                               perpendicular.position.y);
        double clearance = distance_func_->getObstacleDistance(pos);
        const double curvature_new = hmpl::getCurvature(parent.position,
                                                        first.position,
                                                        perpendicular.position);
        const double curvature = hmpl::getCurvature(parent.position,
                                                    first.position,
                                                    second->position);
//        if (clearance > second->r || clearance > options_.lower_boundary + 0.8) {
        if (clearance > this->lower_boundary_ && curvature <= curvature_new) {
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
    while (second->position.Distance(perpendicular.position) > 1e-4) {
        grid_map::Position pos(perpendicular.position.x,
                               perpendicular.position.y);
        double clearance = this->distance_func_->getObstacleDistance(pos);
        if (clearance > second->r
                || clearance > this->lower_boundary_ + 0.8) {
//        if (clearance > this->lower_boundary_ + safety_margin_) {
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

double NonDerivativeSmoothing::getLengthOfPath() const {
    std::size_t size = this->circle_path_.size();
    double length = 0;
    if (size > 1) {
        for (std::size_t i = 0; i < size - 2; i++) {
            double x_1 = this->circle_path_.at(i).position.x;
            double x_2 = this->circle_path_.at(i + 1).position.x;
            double y_1 = this->circle_path_.at(i).position.y;
            double y_2 = this->circle_path_.at(i + 1).position.y;
            double delta = std::sqrt(
                    (x_1 - x_2) * (x_1 - x_2) + (y_1 - y_2) * (y_1 - y_2));
            length += delta;
        }
        return length;
    } else {
        std::cout << "The circle path size is 0 or 1.";
        return length;
    }
}

double NonDerivativeSmoothing::getCirclePathEnergy() const {
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
        const hmpl::Vector2D<double>
                &first = this->circle_path_.at(i - 1).position;
        const hmpl::Vector2D<double>
                &second = this->circle_path_.at(i).position;
        const hmpl::Vector2D<double>
                &third = this->circle_path_.at(i + 1).position;

        smoothness = hmpl::getCurvature(first, second, third);
        length = second.Distance(first) + third.Distance(second);

        energy = pow(smoothness, 2.0) * length;
        f = f + energy;
    }
    return f;
}

void NonDerivativeSmoothing::optimizePathLength() {
    std::size_t size = this->circle_path_.size();
    int count = 0;
    if (size > 3) {
        double length, new_length;
        do {
            length = this->getLengthOfPath();
            if (size > 5) {
                for (std::size_t i = 0; i < size - 4; i += 1) {
                    CircleRef first = this->circle_path_.at(i);
                    CircleRef second = this->circle_path_.at(i + 1);
                    CircleRef third = this->circle_path_.at(i + 2);
                    CircleRef forth = this->circle_path_.at(i + 3);
                    CircleRef fifth = this->circle_path_.at(i + 4);
                    updateCircleCenterWithoutLimit(first, &second, fifth);
                    updateCircleCenterWithoutLimit(first, &third, fifth);
                    updateCircleCenterWithoutLimit(first, &forth, fifth);
                }
            }

            for (std::size_t i = 0; i < size - 2; i++) {
//                CircleRef parent =
//                        this->circle_path_.at(i - 1)->circle;
                CircleRef first = this->circle_path_.at(i);
                CircleRef second = this->circle_path_.at(i + 1);
                CircleRef third = this->circle_path_.at(i + 2);
//                updateCircleCenter(parent, first, &second, third);
                updateCircleCenterWithoutLimit(first, &second, third);
            }
            new_length = this->getLengthOfPath();
            double improvement = fabs(new_length - length) / length;

            if (improvement == 0) {
                break;
            }

            if (new_length < length && improvement < 1e-5) {
                break;
            }
            count ++;
        } while (new_length < length && count < 10);
    }
}

void NonDerivativeSmoothing::optimizePathImproved() {
    std::size_t size = circle_path_.size();
    if (size > 3) {
        double energy, new_energy;
        do {
            energy = this->getCirclePathEnergy();
            for (std::size_t i = 1; i < size - 2; i++) {
                CircleRef parent = this->circle_path_.at(i - 1);
                CircleRef first = this->circle_path_.at(i);
                CircleRef second = this->circle_path_.at(i + 1);
                CircleRef third = this->circle_path_.at(i + 2);

                updateCircleCenter(parent, first, &second, third);
                // updateCircleCenterWithoutLimit(parent, first, &second,
                // third);
            }
            new_energy = this->getCirclePathEnergy();
            double improvement = fabs(new_energy - energy) / energy;

            if (improvement == 0) {
                break;
            }

            if (new_energy < energy && improvement < 0.0001) {
                break;
            }
        } while (new_energy < energy);
    }
}

}