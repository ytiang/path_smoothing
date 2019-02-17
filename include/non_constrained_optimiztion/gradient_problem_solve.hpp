//
// Created by yangt on 18-12-23.
//

#ifndef PATH_SMOOTHING_GRADIENT_PROBLEM_SOLVE_HPP
#define PATH_SMOOTHING_GRADIENT_PROBLEM_SOLVE_HPP

#pragma once
#include <memory>
#include "gradient_problem_options.hpp"
#include "gradient_problem.hpp"
#include "minimizer.hpp"

namespace ncopt {

class GradientProblemSolver {
 public:

    explicit GradientProblemSolver(GradientProblem *problem);

    bool Solve(double *param_ptr, const GradientProblemOption &options,
               Summary *summary);


 private:
    std::shared_ptr<GradientProblem> problem_;
};

}
#endif //PATH_SMOOTHING_GRADIENT_PROBLEM_SOLVE_HPP
