//
// Created by yangt on 19-2-18.
//

#ifndef PATH_SMOOTHING_PATH_SMOOTHING_HPP
#define PATH_SMOOTHING_PATH_SMOOTHING_HPP
#include "non_constrained_optimiztion/gradient_problem.hpp"
#include "non_constrained_optimiztion/gradient_problem_solve.hpp"
#include "path_smoothing/evaluator.hpp"

namespace path_smoothing {

//template<SolverType>
//class PathSmoothing : public SolverType {
class PathSmoothing : public ncopt::GradientProblem {
 public:
  struct Options {
    double heading_term_coe = 1;
    double curvature_term_coe = 100.0;
    double obstacle_term_coe = 1.0;
    EvaluatorType type = CPPAD;
  };
//  template<class PointType>
  typedef geometry_msgs::Point PointType;
  PathSmoothing(const std::vector<PointType> &path,
                const Options &options);

  virtual bool Evaluate(double *parameters,
                        double *cost,
                        double *gradient);

  virtual inline int NumberOfParams() {
      return settings_.param_num;
  }

//  virtual ~PathSmoothing() {
//      delete evaluator_;
//      evaluator_ = NULL;
//  }

 private:
  Evaluator *evaluator_;
  Evaluator::Settings settings_;
};

}


//using namespace ncopt;
//
//class SmoothingCostFunction : public ncopt::GradientProblem {
//
// public:
//  virtual ~SmoothingCostFunction() {}
//
//  SmoothingCostFunction(int number_of_parameter, int degree);
//
//  virtual int NumberOfParams() { return param_num_; }
//
//  virtual bool Evaluate(double *current_state, double *cost,
//                        double *gradient);
//
//  VectorRef Block(int i, double *vec) const {
//      return VectorRef(vec + i * degree_, degree_);
//  }
//
//  inline Vector Cross(const Vector &vec1, const Vector &vec2) const {
//      return (vec1 - vec1.dot(vec2) / pow(vec2.norm(), 2) * vec2) / vec1.norm()
//              / vec2.norm();
//  }
//
//  inline double Cos(const Vector &vec1, const Vector &vec2) const {
//      return std::min(std::max(vec1.dot(vec2) / vec1.norm() / vec2.norm(),
//                               -1.0), 1.0);
//  }
//
//  inline void SetEndPoints(const Vector &start_pt, const Vector &end_pt) {
//      start_point_ = start_pt;
//      end_point_ = end_pt;
//  }
//
// private:
//  double w1_;
//  double w2_;
//  double w3_;
//  const int param_num_;
//  const int degree_;
//  Vector start_point_;
//  Vector end_point_;
////  Eigen::Matrix<double, Dynamic, Dynamic> diff_matrix;
//};

#endif //PATH_SMOOTHING_PATH_SMOOTHING_HPP
