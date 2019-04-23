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
      final_cost = std::numeric_limits<double>::infinity();
      cost_function_decrement = std::numeric_limits<double>::infinity();
      final_gradient_norm = std::numeric_limits<double>::infinity();
      step_norm = std::numeric_limits<double>::infinity();
      solve_iteration_count = 0;
      line_search_iteration_count = 0;
      line_search_direction_type = STEEPEST_DESCENT;
      step_length_type = ARMIJO;
      termination_type = NO_CONVERGENCE;
  }

  std::string message;
  double step;
  double initial_step;
  double initial_cost;
  double final_cost;
  double cost_function_decrement;
  double initial_gradient_norm;
  double final_gradient_norm;
  double step_norm;
  int solve_iteration_count;
  int line_search_iteration_count;
  int search_step_fail_count;
  LineSearchDirectionType line_search_direction_type;
  LineSearchType step_length_type;
  TerminationType termination_type;
#ifdef DEBUG
  std::vector<double> step_length_vec;
  std::vector<double> initial_step_length_vec;
  std::vector<int> line_search_iterations_vec;
  std::vector<double> cost_vec;
  std::vector<double> dir_norm_vec;
  std::vector<double> gradient_norm_vec;
  std::vector<double> gradient_max_norm_vec;
  std::vector<Eigen::Vector2d> param_vec;
  std::vector<Eigen::Vector2d> gradient_vec;
  std::vector<Eigen::Vector2d> dir_vec;
#endif

  static void PrintSummary(const Summary &summary) {
      printf("Solver Report:\n"
                     "  line search type: %s\n"
                     "  step length type: %s\n"
                     "  Iterations: %d\n"
                     "  Initial Cost: %.12f\n"
                     "  Final Cost: %.12f\n"
                     "  Initial Gradient norm: %.12f\n"
                     "  Final Gradient norm: %.12f\n"
                     "  Step norm: %.12f\n"
                     "  Termination type: %s\n",
             LineSearchDirectionTypeToString(summary.line_search_direction_type),
             LIneSearchStepTypeToString(summary.step_length_type),
             summary.solve_iteration_count,
             summary.initial_cost,
             summary.final_cost,
             summary.initial_gradient_norm,
             summary.final_gradient_norm,
             summary.step_norm,
             TerminationTypeToString(summary.termination_type));
  }
};

}
#endif //PATH_SMOOTHING_SUMMARY_HPP
