//
// Created by yangt on 18-12-29.
//

#ifndef PATH_SMOOTHING_SUMMARY_HPP
#define PATH_SMOOTHING_SUMMARY_HPP

#include <string>
#include <limits>
#include <iostream>
#include "type.hpp"

#ifdef DEBUG
#include <vector>
#endif

namespace ncopt {

class Summary {
 public:
  Summary() {
      cost_function_value = std::numeric_limits<double>::infinity();
      cost_function_decrement = std::numeric_limits<double>::infinity();
      gradient_norm_value = std::numeric_limits<double>::infinity();
      gradien_norm_decrement = std::numeric_limits<double>::infinity();
      solve_iteration_count = 0;
      line_search_iteration_count = 0;
      line_search_direction_type = STEEPEST_DESCENT;
      step_length_type = ARMIJO;
      termination_type = NO_CONVERGENCE;
  }

  std::string message;
  double step;
  double initial_step;
  double cost_function_value;
  double cost_function_decrement;
  double gradient_norm_value;
  double gradien_norm_decrement;
  int solve_iteration_count;
  int line_search_iteration_count;
  LineSearchDirectionType line_search_direction_type;
  LineSearchType step_length_type;
  TerminationType termination_type;
#ifdef DEBUG
  std::vector<double> step_length_vec;
  std::vector<double> initial_step_length_vec;
  std::vector<int> line_search_iterations_vec;
  std::vector<double> cost_vec;
  std::vector<double> gradient_norm_vec;
#endif

  static void PrintSummary(const Summary &summary) {
      printf("Solver Report:\n"
                     "  line search type: %s\n"
                     "  step length type: %s\n"
                     "  Iterations: %d\n"
                     "  Cost: %f\n"
                     "  Cost decrement: %f\n"
                     "  Gradient norm: %f\n"
                     "  Gradient decrement: %f\n"
                     "  Termination type: %s\n",
             LineSearchDirectionTypeToString(summary.line_search_direction_type),
             LIneSearchStepTypeToString(summary.step_length_type),
             summary.solve_iteration_count,
             summary.cost_function_value,
             summary.cost_function_decrement,
             summary.gradient_norm_value,
             summary.gradien_norm_decrement,
             TerminationTypeToString(summary.termination_type));
  }
};

}
#endif //PATH_SMOOTHING_SUMMARY_HPP
