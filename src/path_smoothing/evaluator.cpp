//
// Created by yangt on 19-2-21.
//
#include "path_smoothing/evaluator.hpp"
#include <glog/logging.h>
namespace path_smoothing {

Evaluator::Evaluator(const Settings &settings) : settings_(settings) {

}

//template<class PointType>
Evaluator *Evaluator::createEvaluator(const Settings &settings,
                                      const Vector &param) {
    CHECK_EQ(settings.param_num, param.size())
        << "setting parameters' number is different from the size of input param!!!";
    switch (settings.type) {
        case CPPAD:return new CppADEvalator(settings, param);
        case CASADI: return new CasadiEvaluator(settings);
    }
}

template<class ScalarType, class VectorType>
ScalarType Evaluator::targetFunction(VectorType x) const {
    CHECK_GT(degree(), 0) << "invalid degree information :"
                          << degree();
    CHECK_GT(settings_.param_num, degree())
        << "invalid parameter numbers " << settings_.param_num;
    const int size = settings_.param_num / degree();
    ScalarType dx_i, dy_i, dx_ii, dy_ii;
    ScalarType heading_error;
    ScalarType value = 0;
    dx_i = x(0) - settings_.start(0);
    dy_i = x(1) - settings_.start(1);
    for (int i(1); i < size + 1; ++i) {
        if (i < size) {
            dx_ii = x(i * degree()) - x((i - 1) * degree());
            dy_ii = x(i * degree() + 1) - x((i - 1) * degree() + 1);
        } else {
            dx_ii = settings_.end(0) - x((i - 1) * degree());
            dy_ii = settings_.end(1) - x((i - 1) * degree() + 1);
        }
        heading_error = (dx_ii - dx_i) * (dx_ii - dx_i)
                + (dy_ii - dy_i) * (dy_ii - dy_i);
        const ScalarType dot = dx_ii * dx_i + dy_ii * dy_i;
        const ScalarType norm_i = sqrt(dx_i * dx_i + dy_i * dy_i) + 0.01;
        const ScalarType norm_ii = sqrt(dx_ii * dx_ii + dy_ii * dy_ii) + 0.01;
        const ScalarType theta = acos(dot / norm_i / norm_ii);
        value = value + settings_.heading_term_coe * heading_error;
        +settings_.curvature_term_coe * theta * theta / norm_i / norm_i;
        dx_i = dx_ii;
        dy_i = dy_ii;
    }
    return value;
}

CppADEvalator::CppADEvalator(const Settings &settings,
                             const Vector &param)
        : Evaluator(settings) {
    NewVector<CppADScalar> x(settings.param_num);
    NewVector<CppADScalar> y(1);
    for (int i = 0; i < NumParameters(); ++i) {
        x.at(i) = param(i);
        x.at(i) = param(i);
    }
    CppAD::Independent(x);
    y[0] = this->targetFunction<CppADScalar>(x);
    gradient_ = new CppAD::ADFun<double>(x, y);
}
bool CppADEvalator::Evaluate(const double *x,
                             double *cost,
                             double *gradient) const {
    if (cost != NULL) {
        VectorRef x_ref(const_cast<double *>(x), NumParameters());
        *cost = targetFunction<double>(x_ref);
    }
    if (gradient != NULL) {
        std::vector<double> var(x, x + NumParameters());
        std::vector<double> value = gradient_->Jacobian(var);
        for (int i(0); i < value.size(); ++i) {
            *(gradient + i) = value.at(i);
        }
    }
    return true;
}

bool CasadiEvaluator::Evaluate(const double *x,
                               double *cost,
                               double *gradient) const {
    if (cost != NULL) {
        VectorRef x_ref(const_cast<double *>(x), NumParameters());
        *cost = targetFunction<double>(x_ref);
    }
    if (gradient != NULL) {
        std::vector<double> x_vec(x, x + NumParameters());
        std::vector<casadi::DM> value = gradient_(casadi::DM(x_vec));
        for (int i(0); i < value[0]->size(); ++i) {
            *(gradient + i) = value[0]->at(i);
        }
    }
    return true;
}

CasadiEvaluator::CasadiEvaluator(const Settings &settings)
        : Evaluator(settings) {
    casadi::MX x = casadi::MX::sym("x", settings.param_num);
    casadi::MX y = this->targetFunction<casadi::MX>(x);
    casadi::MX g = gradient(y, x);
    gradient_ = casadi::Function("g", {x}, {g});
}

}