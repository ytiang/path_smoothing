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
                                      const std::vector<PointType> &param) {
    CHECK_EQ(settings.param_num, (param.size() - 2) * 2)
        << "setting parameters' number is different from the size of input param!!!";
    switch (settings.type) {
        case CPPAD:return new CppADEvalator(settings, param);
        case CASADI: return new CasadiEvaluator(settings);
    }
}

template<class ScalarType, class VectorType>
ScalarType Evaluator::targetFunction(VectorType x) {
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

//template<class PointType>
CppADEvalator::CppADEvalator(const Settings &settings,
                             const std::vector<PointType> &param)
        : Evaluator(settings) {
    NewVector<CppADScalar> x(settings.param_num);
    NewVector<CppADScalar> y(1);
    for (int i = 1; i < param.size() - 1; ++i) {
        const int j = i - 1;
        x.at(j * degree()) = param.at(i).x;
        x.at(j * degree() + 1) = param.at(i).y;
    }
    CppAD::Independent(x);
    y[0] = this->targetFunction<CppADScalar>(x);
    gradient_ = CppAD::ADFun<double>(x, y);
}
void CppADEvalator::evaluate(double *x, double *cost, double *gradient) {
    if (cost != NULL) {
        VectorRef x_ref(x, numParameters());
        *cost = targetFunction<double>(x_ref);
    }
    if (gradient != NULL) {
        std::vector<double> var(x, x + numParameters());
        std::vector<double> value = gradient_.Jacobian(var);
        for (int i(0); i < value.size(); ++i) {
            *(gradient + i) = value.at(i);
        }
    }
}

void CasadiEvaluator::evaluate(double *x, double *cost, double *gradient) {
    if (cost != NULL) {
        VectorRef x_ref(x, numParameters());
        *cost = targetFunction<double>(x_ref);
    }
    if (gradient != NULL) {
        std::vector<double> x_vec(x, x + numParameters());
        std::vector<casadi::DM> value = gradient_(casadi::DM(x_vec));
        for (int i(0); i < value[0]->size(); ++i) {
            *(gradient + i) = value[0]->at(i);
        }
    }
}

CasadiEvaluator::CasadiEvaluator(const Settings &settings)
        : Evaluator(settings) {
    casadi::MX x = casadi::MX::sym("x", settings.param_num);
    casadi::MX y = this->targetFunction<casadi::MX>(x);
    casadi::MX g = gradient(y, x);
    gradient_ = casadi::Function("g", {x}, {g});
}

}