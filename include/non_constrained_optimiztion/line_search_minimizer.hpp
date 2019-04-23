//
// Created by yangt on 18-12-21.
//

#ifndef PATH_SMOOTHING_LINESEARCHMINIMIZER_HPP
#define PATH_SMOOTHING_LINESEARCHMINIMIZER_HPP

#include "eigen_typedef.hpp"
#include "minimizer.hpp"

namespace ncopt {

class LineSearchMinimizer : public Minimizer {
 public:

  explicit LineSearchMinimizer(const GradientProblemOption &options);

  virtual bool Minimize(double *param_ptr, GradientProblem *problem,
                        Summary *summary);


  double GetInitialStepLength(const State &previous_state,
                              const State &current_state,
                              const Summary &summary) const;

  virtual ~LineSearchMinimizer() {}

 private:
  const GradientProblemOption &minimizer_option_;

  inline bool IsConvergent(const State &current_state,
                           const State &previous_state,
                           Summary *summary) const {
      summary->cost_function_decrement =
              previous_state.cost - current_state.cost;
      summary->step_norm = previous_state.search_direction.norm()
              * current_state.step_length;
      const double step_norm_tolerance =
              minimizer_option_.step_norm_torelance_coe * (previous_state.x_norm
                      + minimizer_option_.step_norm_torelance_coe);

      if (current_state.gradient_max_norm <
              minimizer_option_.gradient_norm_tolerance) {
          summary->termination_type = GRADIENT_NORM_CONVERGENCE;
          return true;
      } else if (summary->solve_iteration_count > 0) {
          if (fabs(summary->cost_function_decrement) <
                  minimizer_option_.function_tolerance * previous_state.cost) {
              summary->termination_type = COST_DECREMENT_CONVERGENCE;
              return true;
          } else if (summary->step_norm < step_norm_tolerance) {
              summary->termination_type = STEP_NORM_CONVERGENCE;
              return true;
          }
      }
      return false;
  }

};

}

#endif //PATH_SMOOTHING_LINESEARCHMINIMIZER_HPP
