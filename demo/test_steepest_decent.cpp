//
// Created by yangt on 18-12-27.
//
#include "non_constrained_optimiztion/gradient_problem.hpp"
#include "non_constrained_optimiztion/gradient_problem_solve.hpp"
#include <sys/time.h>
#include "ceres/ceres.h"

using namespace ncopt;
const double w1 = 1;
const double w2 = 100;
class MyFunction : public GradientProblem {
 public:
  MyFunction(int param_num) {
      param_num_ = param_num;
  }
  int NumParameters() const {
      return this->param_num_;
  }
  bool Evaluate(const double *current_state, double *cost,
                double *gradient) const {
      const double x = current_state[0];
      const double y = current_state[1];
      if (cost != NULL) {
          cost[0] = w1 * (1.0 - x) * (1.0 - x) + w2 * (y - x * x) * (y - x * x);
      }
      if (gradient != NULL) {
          gradient[0] = -2.0 * w1 * (1.0 - x) - 2 * w2 * (y - x * x) * 2.0 * x;
          gradient[1] = 2 * w2 * (y - x * x);
      }
      return true;
  }

 private:
  int param_num_;
};

class MyFunction2 : public ceres::FirstOrderFunction {
 public:
  virtual ~MyFunction2() {}
  virtual bool Evaluate(const double *parameters,
                        double *cost,
                        double *gradient) const {
      const double x = parameters[0];
      const double y = parameters[1];
      cost[0] = w1 * (1.0 - x) * (1.0 - x) + w2 * (y - x * x) * (y - x * x);
      if (gradient != NULL) {
          gradient[0] =
                  -2.0 * w1 * (1.0 - x) - 2.0 * w2 * (y - x * x) * 2.0 * x;
          gradient[1] = 2 * w2 * (y - x * x);
      }
      return true;
  }
  virtual int NumParameters() const { return 2; }
};

double GetTimeInteral(const timeval &start, const timeval &end) {
    return (end.tv_sec - start.tv_sec) + 1e-6 * (end.tv_usec - start.tv_usec);
}

int main() {
    double initial_guess[2] = {1.2, 1.2};
    timeval ts;
    timeval te;
    GradientProblemSolver solver(new MyFunction(2));
    GradientProblemOption options;
    options.max_solve_iterations_num = 10000;
    Summary summary;
    gettimeofday(&ts, NULL);
    solver.Solve(initial_guess, options, &summary);
    gettimeofday(&te, NULL);
    Summary::PrintSummary(summary);
    std::cout << "prama: " << initial_guess[0] << ", " << initial_guess[1]
              << "\n";
    printf("self total time: %f\n", GetTimeInteral(ts, te));

    printf("\n*********************\n");
    ceres::GradientProblemSolver::Options option2;
    ceres::GradientProblemSolver::Summary summary2;
    ceres::GradientProblem problem(new MyFunction2());
    option2.nonlinear_conjugate_gradient_type = ceres::FLETCHER_REEVES;
    option2.line_search_interpolation_type = ceres::QUADRATIC;
    option2.line_search_type = ceres::WOLFE;
    option2.line_search_sufficient_function_decrease = 1e-4;
    option2.line_search_sufficient_curvature_decrease = 0.3;
    option2.min_line_search_step_contraction = 0.92;
    option2.max_line_search_step_contraction = 1e-4;
    option2.line_search_direction_type =
            ceres::NONLINEAR_CONJUGATE_GRADIENT;//ceres::STEEPEST_DESCENT; //
    initial_guess[0] = 1.2;
    initial_guess[1] = 1.2;
    gettimeofday(&ts, NULL);
    ceres::Solve(option2, problem, initial_guess, &summary2);
    gettimeofday(&te, NULL);
    printf("param: %f, %f, total cost: %f\n", initial_guess[0],
           initial_guess[1], GetTimeInteral(ts, te));
    std::cout << summary2.FullReport() << "\n";
#ifdef DEBUG
    printf("\n\n******** step size: \n");
    for (int i = 0; i < summary.cost_vec.size(); ++i) {
        if(i<summary2.iterations.size()) {
            printf("[%d]: ceres a: %7f, f: %7f, |g|: %7f; self: a: %7f, f: %7f, |g|: %7f\n",
                   summary2.iterations.at(i).step_size,
                   summary2.iterations.at(i).cost,
                   summary2.iterations.at(i).gradient_norm,
                   summary.step_length_vec.at(i),
                   summary.cost_vec.at(i),
                   summary.gradient_norm_vec.at(i));
        } else {
            printf("[%d]: ceres a: %7f, f: %7f, |g|: %7f; self: a: %7f, f: %7f, |g|: %7f\n",
                   0.0,
                   0.0,
                   0.0,
                   summary.step_length_vec.at(i),
                   summary.cost_vec.at(i),
                   summary.gradient_norm_vec.at(i));
        }
    }
#endif
    return 0;
}