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
        case BFGS:return new BFGSDirection(dim);
        case LBFGS: return new LBFGSDirection(dim);
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

    if (fabs(current.gradient.dot(*search_direction)) < 1e-6) {
        *search_direction = -current.gradient;
//        LOG(WARNING) << "Restarting non-linear conjugate gradients: ";
    }
}

BFGSDirection::BFGSDirection(int dim)
        : dim_(dim) {
    H_inv_ = Matrix::Identity(dim, dim);
    identity_ = Matrix::Identity(dim, dim);
}

void BFGSDirection::NextDirection(const LineSearchMinimizer::State &previous,
                                  const LineSearchMinimizer::State &current,
                                  Vector *search_direction) {
    const auto s = previous.step_length * previous.search_direction;
    const auto y = current.gradient - previous.gradient;
    const double p = 1.0 / (s.transpose() * y);
    const Matrix M = identity_ - p * s * y.transpose();
    H_inv_ = M * H_inv_ * M.transpose() + p * s * s.transpose();
    *search_direction = -H_inv_ * current.gradient;
}

LBFGSDirection::LBFGSDirection(int dim)
        : dim_(dim),
          length_(15),
          s_(length_),
          y_(length_),
          rho_(length_) {
}

void LBFGSDirection::NextDirection(const LineSearchMinimizer::State &previous,
                                   const LineSearchMinimizer::State &current,
                                   Vector *search_direction) {
    // s_{k-1} = x_{k} - x_{k-1} = alpha * p_{k-1}
    // y_{k-1} = g_{k} - g_{k-1};
    const Vector s = previous.step_length * previous.search_direction;
    const Vector y = current.gradient - previous.gradient;
    this->s_.push_back(s);
    this->y_.push_back(y);
    this->rho_.push_back(1.0 / (s.transpose() * y));
    const auto size = s_.size();

    auto q = current.gradient;
    double alpha[size];
    for (int i(0); i < size; ++i) {
        const int j = size - i - 1;
        alpha[j] = rho_[j] * s_[j].transpose() * q;
        q = q - alpha[j] * y_[j];
    }

    Vector r = rho_.back() / y_.back().squaredNorm() * q;
//    auto r = q;

    for (int i(0); i < size; ++i) {
        const double beta = rho_[i] * y_[i].transpose() * r;
        r = r + (alpha[i] - beta) * s_[i];
    }
    *search_direction = -r;
}

}



