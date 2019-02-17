//
// Created by yangt on 18-12-23.
//

#ifndef PATH_SMOOTHING_MINIMIZER_HPP
#define PATH_SMOOTHING_MINIMIZER_HPP

#include "gradient_problem_options.hpp"
#include "gradient_problem.hpp"
#include "eigen_typedef.hpp"
#include "summary.hpp"

namespace ncopt {

class Minimizer {
 public:
    struct State {
        State(int num_effective_parameters)
                : cost(0.0),
                  gradient(num_effective_parameters),
                  gradient_norm(0.0),
                  search_direction(num_effective_parameters),
                  directional_derivative(0.0),
                  step_length(0.0) {
        }

        double cost;
        Vector gradient;
        double gradient_norm;
        Vector search_direction;
        double directional_derivative;
        double step_length;
    };

    static Minimizer * Create(const GradientProblemOption &options);

    virtual bool Minimize(double *param_ptr, GradientProblem *problem,
                          Summary *summary) = 0;

    virtual ~Minimizer() {}
};

}

#endif //PATH_SMOOTHING_MINIMIZER_HPP
