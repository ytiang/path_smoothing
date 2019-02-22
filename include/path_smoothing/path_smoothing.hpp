//
// Created by yangt on 19-2-18.
//

#ifndef PATH_SMOOTHING_PATH_SMOOTHING_HPP
#define PATH_SMOOTHING_PATH_SMOOTHING_HPP

#include "path_smoothing/evaluator.hpp"
#include "non_constrained_optimiztion/gradient_problem_solve.hpp"

namespace path_smoothing {

class PathSmoothing {
 public:
  struct Options {
    double heading_term_coe = 1;
    double curvature_term_coe = 1.0;
    double obstacle_term_coe = 1.0;
    DifferenceType type = CPPAD;
    SolverType solver = CERES_SOLVER;
  };

  template<class PointType>
  void smoothPath(const Options &options,
                  std::vector<PointType> *path) {
      Evaluator::Settings settings;
      settings.heading_term_coe = options.heading_term_coe;
      settings.curvature_term_coe = options.curvature_term_coe;
      settings.obstacle_term_coe = options.obstacle_term_coe;
      settings.type = options.type;
      settings.degree = 2;
      settings.param_num = (path->size() - 2) * settings.degree;
      settings.start.resize(settings.degree);
      settings.start << path->front().x, path->front().y;
      settings.end.resize(settings.degree);
      settings.end << path->back().x, path->back().y;

      Evaluator::Vector initial_param(settings.param_num);
      for (int i(1); i < path->size() - 1; ++i) {
          const int j = i - 1;
          initial_param(j * settings.degree) = path->at(i).x;
          initial_param(j * settings.degree + 1) = path->at(i).y;
      }

      Evaluator *smooth_function =
              Evaluator::createEvaluator(settings, initial_param);

      switch (options.solver) {
          case CERES_SOLVER: {
              ceres::GradientProblemSolver::Options option;
              ceres::GradientProblemSolver::Summary summary;
              ceres::GradientProblem problem(smooth_function);
              option.nonlinear_conjugate_gradient_type = ceres::FLETCHER_REEVES;
              option.line_search_interpolation_type = ceres::QUADRATIC;
              option.line_search_type = ceres::WOLFE;
              option.line_search_sufficient_function_decrease = 1e-4;
              option.line_search_sufficient_curvature_decrease = 0.3;
              option.min_line_search_step_contraction = 0.92;
              option.max_line_search_step_contraction = 1e-4;
              option.line_search_direction_type =
                      ceres::NONLINEAR_CONJUGATE_GRADIENT;//ceres::STEEPEST_DESCENT; //
              ceres::Solve(option, problem, initial_param.data(), &summary);
              break;
          }
          case SELF_SOLVER: {
              ncopt::GradientProblemOption solver_options;
              ncopt::Summary summarys;
              ncopt::GradientProblemSolver solver(smooth_function);
              solver.Solve(initial_param.data(), solver_options, &summarys);
              ncopt::Summary::PrintSummary(summarys);
              break;
          }
      }

      for (int i(1); i < path->size() - 1; ++i) {
          const int j = i - 1;
          path->at(i).x = initial_param(j * settings.degree);
          path->at(i).y = initial_param(j * settings.degree + 1);
      }
  }
};

}

#endif //PATH_SMOOTHING_PATH_SMOOTHING_HPP
