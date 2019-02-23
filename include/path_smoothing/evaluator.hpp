//
// Created by yangt on 19-2-21.
//

#ifndef PATH_SMOOTHING_EVALUATOR_HPP
#define PATH_SMOOTHING_EVALUATOR_HPP

#include <cppad/cppad.hpp>
#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include "non_constrained_optimiztion/gradient_problem.hpp"
#include "internal_grid_map/internal_grid_map.hpp"

namespace path_smoothing {

enum DifferenceType {
  CPPAD,
  CASADI,
};

enum SolverType {
  SELF_SOLVER,
  CERES_SOLVER
};

class Evaluator
        : public ncopt::GradientProblem, public ceres::FirstOrderFunction {
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
              obstacle_term_coe(1.0),
              map(NULL) {
    }
    int param_num;
    int degree;
    double heading_term_coe;
    double curvature_term_coe;
    double obstacle_term_coe;
    DifferenceType type;
    Vector start;
    Vector end;
    grid_map::GridMap *map;
    std::string sdf_layer;
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

  static Evaluator *createEvaluator(const Settings &settings,
                                    const Vector &param);

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

  inline int degree() const {
      return settings_.degree;
  }

  template<class ScalarType, class VectorType>
  ScalarType targetFunction(VectorType x) const;

  void addObstacleTerm(const VectorRef &x,
                       double *cost,
                       double *gradient) const;

  virtual bool Evaluate(const double *x,
                        double *cost,
                        double *gradient) const = 0;

  virtual inline int NumParameters() const {
      return settings_.param_num;
  }

  grid_map::GridMap *gridMap() const {
      return settings_.map;
  }

 private:
  const Settings &settings_;
};

class CppADEvalator : public Evaluator {
 public:
  CppADEvalator(const Settings &settings, const Vector &param);
  virtual bool Evaluate(const double *x, double *cost, double *gradient) const;

 private:
  CppAD::ADFun<double> *gradient_;
};

class CasadiEvaluator : public Evaluator {
 public:
  CasadiEvaluator(const Settings &settings);
  virtual bool Evaluate(const double *x, double *cost, double *gradient) const;
 private:
  casadi::Function gradient_;
};

}
#endif //PATH_SMOOTHING_EVALUATOR_HPP
