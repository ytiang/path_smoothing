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

  void GetSummaryFromState(const State &state, Summary *summary) const;

  double GetInitialStepLength(const State &previous_state,
                                   const State &current_state,
                                   const Summary &summary) const;

  virtual ~LineSearchMinimizer() {}

 private:
  const GradientProblemOption &minimizer_option_;

  inline bool IsConvergent(const State &current_state,
                           const State &previous_state,
                           Summary *summary) const {
      summary->gradien_norm_decrement =
              current_state.gradient_norm - previous_state.gradient_norm;
      summary->cost_function_decrement =
              current_state.cost - previous_state.cost;
      double cost_decrement_rate =
              summary->cost_function_decrement / previous_state.cost;
      double gradient_norm_decrement_rate =
              summary->gradien_norm_decrement / previous_state.gradient_norm;

      if (current_state.gradient_norm <
              minimizer_option_.gradine_norm_threshold) {
          summary->termination_type = GRADIENT_NORM_CONVERGENCE;
          return true;
      } else if (summary->solve_iteration_count > 0) {
          if (fabs(cost_decrement_rate) <
                  minimizer_option_.function_tolerance) {
              summary->termination_type = COST_DECREMENT_CONVERGENCE;
              return true;
          }/* else if(fabs(gradient_norm_decrement_rate) <
                    minimizer_option_.gradient_norm_tolerance) {
                summary->termination_type = GRADIENT_NORM_DECREMENT_CONVERGENCE;
                return true;
            }*/
      }
      return false;
  }

};

}

#endif //PATH_SMOOTHING_LINESEARCHMINIMIZER_HPP
