//
// Created by yangt on 19-4-4.
//

#include "non_constrained_optimiztion/line_search_step_length.hpp"
#include "non_constrained_optimiztion/matplotlibcpp.h"
#include "non_constrained_optimiztion/polynomial.hpp"
namespace ncopt {

PlotFunction::PlotFunction() :optimal_func_(1), optimal_step_(1) {}

void PlotFunction::SetFunc(StepLengthFunction *function,
                           double initial_step) {
    StepLengthFunction::Samples tmp_smaple;
    double min_cost = std::numeric_limits<double>::infinity();
    double min_cost_t = -1;
    double min_gradient = std::numeric_limits<double>::infinity();
    double min_gradient_t = -1;
    step_.clear();
    gradient_.clear();
    func_.clear();
    for (double t = 0.0; t < 1.0 * initial_step;
         t += initial_step / 1000.0) {
        function->Evaluate(t, true, &tmp_smaple);
        step_.push_back(t);
        func_.push_back(tmp_smaple.value);
        gradient_.push_back(tmp_smaple.gradient / 10.0);
        if (tmp_smaple.value < min_cost) {
            min_cost_t = t;
            min_cost = tmp_smaple.value;
        }
        if (fabs(tmp_smaple.gradient) < fabs(min_gradient)) {
            min_gradient_t = t;
            min_gradient = tmp_smaple.gradient;
        }
    }
    printf("    optimal cost :(%f, %f), optimal gradient: (%f, %f)\n",
           min_cost_t,
           min_cost,
           min_gradient_t,
           min_gradient);
    optimal_func_[0] = min_cost;
    optimal_step_[0] = min_cost_t;
}

void PlotFunction::SetApproxFunc(const Vector &poly) {
    this->approx_funx_.clear();
//    double lower_bound = this->optimal_func_.front() - (optimal_func_.back() - optimal_func_.front()) / 2.0;
    for(int i(0); i < step_.size(); ++i) {
        double approx_val = EvaluatePolynomial(poly, step_.at(i));
//        approx_val = std::max(lower_bound, approx_val);
//        approx_val = std::min(approx_val, 2*this->func_.front());
        approx_funx_.push_back(approx_val);
    }
}

void PlotFunction::plot() {
    namespace plt = matplotlibcpp;
    plt::figure_size(1200, 780);
    plt::plot(step_, func_, "r",
              step_, approx_funx_, "b-.");
    plt::show();
}

}