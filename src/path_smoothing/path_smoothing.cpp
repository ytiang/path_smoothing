//
// Created by yangt on 19-2-18.
//
#include "path_smoothing/path_smoothing.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
namespace path_smoothing {

//template<class PointType>
PathSmoothing::PathSmoothing(const std::vector<PointType> &path,
                             const Options &options) {
    settings_.heading_term_coe = options.heading_term_coe;
    settings_.curvature_term_coe = options.curvature_term_coe;
    settings_.obstacle_term_coe = options.obstacle_term_coe;
    settings_.type = options.type;
    settings_.degree = 2;
    settings_.param_num = (path.size() - 2) * settings_.degree;
    settings_.start.resize(settings_.degree);
    settings_.start << path.front().x, path.front().y;
    settings_.end.resize(settings_.degree);
    settings_.end << path.back().x, path.back().y;
    this->evaluator_ = Evaluator::createEvaluator(settings_, path);
}

bool PathSmoothing::Evaluate(double *parameters,
                             double *cost,
                             double *gradient) {
    this->evaluator_->evaluate(parameters, cost, gradient);
}

}
//SmoothingCostFunction::SmoothingCostFunction(int number_of_parameter,
//                                             int degree)
//        : degree_(degree),
//          param_num_(number_of_parameter) {
//    // initialize differetial matrix
//    w1_ = 0.1;
//    w2_ = 50;
//}
//
//bool SmoothingCostFunction::Evaluate(double *current_state,
//                                     double *cost,
//                                     double *gradient) {
//    int size = param_num_ / degree_;
//    Vector dp0;
//    Vector dp_i(degree_);
//    Vector dp_i1(degree_);
//    Vector dp_i2(degree_);
//    dp_i = Block(0, current_state) - start_point_;
//    dp_i1 = Block(1, current_state) - Block(0, current_state);
//    dp_i2 = Block(2, current_state) - Block(1, current_state);
//    dp0 = dp_i;
//    *cost = 0;
////    printf("before evaluate, gradient address: %x\n", gradient);
//    for (int i(0); i < size; ++i) {
//        const double cos_phi_i = Cos(dp_i1, dp_i);
//        const double phi_i = std::acos(cos_phi_i);
//        const double curvature_i = phi_i / dp_i.norm();
//        const double heading_error_i = (dp_i1 - dp_i).norm();
//        *cost = *cost + w1_ * pow(heading_error_i, 2)
//                + w2_ * pow(curvature_i, 2);
//        if (gradient != NULL) {
//            auto g_i = Block(i, gradient);
//            const double cos_phi0 = Cos(dp_i, dp0);
//            const double phi0 = std::acos(cos_phi0);
//            const double curvature0 = phi0 / dp0.norm();
//
//            const double cos_phi_i1 = Cos(dp_i2, dp_i1);
//            const double phi_i1 = std::acos(cos_phi_i1);
//            const double curvature_i1 = phi_i1 / dp_i1.norm();
//
//            Vector g = w1_ * (2 * (dp_i - dp0) - 4 * (dp_i1 - dp_i)
//                    + 2 * (dp_i2 - dp_i1));
//
//            double coe1, coe2;
//            coe1 = 1 / dp0.norm() * (-1 / std::sqrt(1.01 - pow(cos_phi0, 2)));
//            Vector gc1 = 2 * curvature0 * coe1 * Cross(dp0, dp_i);
//
//            coe1 = 1 / dp_i.norm() * (-1 / std::sqrt(1.01 - pow(cos_phi_i, 2)));
//            coe2 = -phi_i / pow(dp_i.norm(), 3);
//            Vector gc2 = 2 * curvature_i
//                    * (coe1 * (Cross(dp_i1, dp_i) - Cross(dp_i, dp_i1))
//                            + coe2 * dp_i);
//
//            coe1 = 1 / dp_i1.norm()
//                    * (-1 / std::sqrt(1.01 - pow(cos_phi_i1, 2)));
//            coe2 = phi_i1 / pow(dp_i1.norm(), 3);
//            Vector gc3 = 2 * curvature_i1
//                    * (-coe1 * Cross(dp_i2, dp_i1) + coe2 * dp_i1);
//
//            g = g + w2_ * (gc1 + gc2 + gc3);
//            for (int j(0); j < degree_; ++j) {
////                printf("g[%d][%d]: %f\n", i, j, g(j));
//                g_i(j) = g(j);
//                if (fabs(g(j)) > 1e10) {
//                    LOG(ERROR) << "gradient [" << i << "] is too large!"
//                               << " phi0: " << phi0 << ", phi_i: " << phi_i
//                               << ", phi_i1: " << phi_i1
//                               << ". dp0 norm: " << dp0.norm()
//                               << ", dp_i norm: " << dp_i.norm()
//                               << ", dp_i1 norm: " << dp_i1.norm()
//                               << ", dp_i2 norm: " << dp_i2.norm();
//                }
//            }
//        }
//        dp0 = dp_i;
//        dp_i = dp_i1;
//        dp_i1 = dp_i2;
//        if (i + 3 < size) {
//            dp_i2 = Block(i + 3, current_state) - Block(i + 2, current_state);
//        } else if (i + 3 == size) {
//            dp_i2 = end_point_ - Block(i + 2, current_state);
//        }
//    }
//}