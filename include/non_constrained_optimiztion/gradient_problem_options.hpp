//
// Created by yangt on 18-12-24.
//

#ifndef PATH_SMOOTHING_GRADIENT_PROBLEM_OPTIONS_HPP
#define PATH_SMOOTHING_GRADIENT_PROBLEM_OPTIONS_HPP

#include "type.hpp"

namespace ncopt {

class BasicOption {
 public:
  BasicOption() {
      minimizer_type = LINE_SEARCH;
      function_tolerance = 1e-6;
      gradient_norm_tolerance = 1e-10;
      max_solve_iterations_num = 70;
      step_norm_torelance_coe = 1e-6;
  }
  MinimizerType minimizer_type;
  double function_tolerance;
  double gradient_norm_tolerance;
  double gradine_norm_threshold;
  double step_norm_torelance_coe;
  int max_solve_iterations_num;
};

class LineSearchOption {
 public:
  LineSearchType line_search_type = WOLFE;
  LineSearchInterpolationType interpolation_type = QUADRATIC;
  LineSearchDirectionType line_search_direction_type = //STEEPEST_DESCENT;
          NONLINEAR_CONJUGATE_GRADIENT;
  NonlinearConjugateGradientType nonlinear_conjugate_gradient_type =
          FLETCHER_REEVES;
  double sufficient_decrease = 1e-4;
  double sufficient_curvature_decrease = 0.3;
  double min_line_search_step_length = 1e-16;
  double max_step_decrease_rate = 1e-3;
  double min_step_decrease_rate = 0.92;
};

class GradientProblemOption : public BasicOption, public LineSearchOption {

};

}

#endif //PATH_SMOOTHING_GRADIENT_PROBLEM_OPTIONS_HPP
