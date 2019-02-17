//
// Created by yangt on 18-12-23.
//
#include "non_constrained_optimiztion/gradient_problem_solve.hpp"
#include "non_constrained_optimiztion/line_search_minimizer.hpp"

namespace ncopt {

GradientProblemSolver::GradientProblemSolver(GradientProblem *problem)
        : problem_(problem) {
}



bool GradientProblemSolver::Solve(double *param_ptr,
                                  const GradientProblemOption &options,
                                  Summary *summary) {
    CHECK(param_ptr) << "Empty parameter ptr!";
    summary->step_length_type = options.line_search_type;
    summary->line_search_direction_type = options.line_search_direction_type;

    std::unique_ptr<Minimizer> minimizer(Minimizer::Create(options));
    minimizer->Minimize(param_ptr, problem_.get(), summary);
}

}
