//
// Created by yangt on 19-2-21.
//

#ifndef PATH_SMOOTHING_EVALUATOR_HPP
#define PATH_SMOOTHING_EVALUATOR_HPP

#include <cppad/cppad.hpp>
#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>

namespace path_smoothing {

enum EvaluatorType {
  CPPAD,
  CASADI,
};

class Evaluator {
 public:
  typedef CppAD::AD<double> CppADScalar;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;
  typedef Eigen::Map<Vector> VectorRef;

  struct Settings {
    Settings()
            : param_num(-1),
              degree(2),
              type(CPPAD),
              start(0),
              end(0),
              heading_term_coe(1.0),
              curvature_term_coe(1.0),
              obstacle_term_coe(1.0) {}
    int param_num;
    int degree;
    double heading_term_coe;
    double curvature_term_coe;
    double obstacle_term_coe;
    EvaluatorType type;
    Vector start;
    Vector end;
  };

  template<class ScalarType>
  class NewVector : public std::vector<ScalarType> {
   public:
    inline ScalarType operator()(size_t n) {
        return this->at(n);
    }
    using std::vector<ScalarType>::vector;
  };

  Evaluator(const Settings &settings);

  virtual ~Evaluator() {}

//  template<class PointType>
  typedef geometry_msgs::Point PointType;
  static Evaluator *createEvaluator(const Settings &settings,
                                    const std::vector<PointType> &param);

  friend inline CppADScalar cos(const CppADScalar &x) {
      return x.cos_me();
  }
  friend inline CppADScalar sin(const CppADScalar &x) {
      return x.sin_me();
  }
  friend inline CppADScalar sqrt(const CppADScalar &x) {
      return x.sqrt_me();
  }
  friend inline CppADScalar acos(const CppADScalar &x) {
      return x.acos_me();
  }

  inline int numParameters() const {
      return settings_.param_num;
  }

  inline int degree() const {
      return settings_.degree;
  }

  template<class ScalarType, class VectorType>
  ScalarType targetFunction(VectorType x);

  virtual void evaluate(double *x, double *cost, double *gradient) = 0;

 private:
  const Settings &settings_;
};

class CppADEvalator : public Evaluator {
 public:
//  template<class PointType>
  CppADEvalator(const Settings &settings, const std::vector<PointType> &param);
  virtual void evaluate(double *x, double *cost, double *gradient);

 private:
  CppAD::ADFun<double> gradient_;
};

class CasadiEvaluator : public Evaluator {
 public:
  CasadiEvaluator(const Settings &settings);
  virtual void evaluate(double *x, double *cost, double *gradient);
 private:
  casadi::Function gradient_;
};

}
#endif //PATH_SMOOTHING_EVALUATOR_HPP
