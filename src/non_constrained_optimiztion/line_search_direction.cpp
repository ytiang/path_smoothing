//
// Created by yangt on 18-12-21.
//
#include "non_constrained_optimiztion/line_search_direction.hpp"

namespace ncopt {

LineSearchDirection *LineSearchDirection::
Create(const LineSearchOption &options) {
    switch (options.line_search_direction_type) {
        case STEEPEST_DESCENT:return new SteepdestDescentDirection();
        case NONLINEAR_CONJUGATE_GRADIENT:
            return new NonConjugateDirection(
                    options.nonlinear_conjugate_gradient_type);
    }
}

NonConjugateDirection::
NonConjugateDirection(const NonlinearConjugateGradientType type)
        : type_(type) {

}

void NonConjugateDirection::NextDirection(
        const LineSearchMinimizer::State &previous,
        const LineSearchMinimizer::State &current, Vector *search_direction) {
    double beta = 0.0;
    switch (type_) {
        case FLETCHER_REEVES: {
            beta = current.gradient_norm / previous.gradient_norm;
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
            double beta_fr = current.gradient_norm / previous.gradient_norm;
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
//    if (current.gradient.dot(*search_direction) > -1e-6) {
//        *search_direction = -current.gradient;
//    }
}

}

