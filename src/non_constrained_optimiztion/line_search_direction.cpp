//
// Created by yangt on 18-12-21.
//
#include "non_constrained_optimiztion/line_search_direction.hpp"

namespace ncopt {

LineSearchDirection *LineSearchDirection::
Create(const LineSearchOption &options, int dim) {
    switch (options.line_search_direction_type) {
        case STEEPEST_DESCENT:return new SteepdestDescentDirection();
        case NONLINEAR_CONJUGATE_GRADIENT:
            return new NonConjugateDirection(
                    options.nonlinear_conjugate_gradient_type);
        case QUASI_NEWTON:
            return new QuasiNewtonDirection(options.quasi_nweton_type,
                                            dim);
    }
}

NonConjugateDirection::
NonConjugateDirection(const NonlinearConjugateGradientType type)
        : type_(type) {

}

void NonConjugateDirection::NextDirection(
        const LineSearchMinimizer::State &previous,
        const LineSearchMinimizer::State &current, Vector *search_direction) {
//    if (previous.gradient.dot(current.gradient) /
//            pow(current.gradient_norm, 2) < 0.1) {
////        LOG(WARNING) << "Restarting non-linear conjugate gradients";
//        *search_direction = -current.gradient;
//    }
    double beta = 0.0;
    switch (type_) {
        case FLETCHER_REEVES: {
            beta = current.gradient.dot(current.gradient)
                    / previous.gradient.dot(previous.gradient);
            break;
        }
        case POLAK_RIBIERE: {
            beta = current.gradient.dot(current.gradient - previous.gradient)
                    / std::pow(previous.gradient_norm, 2);
            break;
        }
        case POLAK_RIBIERE_PLUS: {
            beta = current.gradient.dot(current.gradient - previous.gradient)
                    / std::pow(previous.gradient_norm, 2);
            beta = std::max(beta, 0.0);
            break;
        }
        case FR_PR: {
            double beta_fr = pow(current.gradient_norm, 2)
                    / pow(previous.gradient_norm, 2);
            double beta_pr =
                    current.gradient.dot(current.gradient - previous.gradient)
                            / std::pow(previous.gradient_norm, 2);
            if (beta_pr < -beta_fr) {
                beta = -beta_fr;
            } else if (fabs(beta_pr) <= beta_fr) {
                beta = beta_pr;
            } else if (beta_pr > beta_fr) {
                beta = beta_fr;
            }
            break;
        }
    }
    *search_direction = -current.gradient + beta * previous.search_direction;

    if (current.gradient.dot(*search_direction) > -1e-6) {
        *search_direction = -current.gradient;
//        LOG(WARNING) << "Restarting non-linear conjugate gradients: ";
    }
}

QuasiNewtonDirection::QuasiNewtonDirection(const QuasiNewtonType type, int dim)
        : type_(type),
          dim_(dim) {
    H_inv_ = Matrix::Identity(dim, dim);
    identity_ = Matrix::Identity(dim, dim);
}

void QuasiNewtonDirection::NextDirection(const LineSearchMinimizer::State &previous,
                                         const LineSearchMinimizer::State &current,
                                         Vector *search_direction) {
    const auto s_k = previous.step_length * previous.search_direction;
    const auto y_k = current.gradient - previous.gradient;
    const double p = 1.0 / (s_k.transpose() * y_k);
    switch (type_) {
        case BFGS: {
            const Matrix M = identity_ - p * s_k * y_k.transpose();
            H_inv_ = M * H_inv_ * M.transpose() + p * s_k * s_k.transpose();
            *search_direction = -H_inv_ * current.gradient;
            break;
        }
    }
}

}

