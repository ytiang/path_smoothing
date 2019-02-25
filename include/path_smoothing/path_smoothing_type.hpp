//
// Created by yangt on 19-2-25.
//

#ifndef PATH_SMOOTHING_TYPE_HPP
#define PATH_SMOOTHING_TYPE_HPP

namespace path_smoothing {

enum SmootherType {
  CONJUGATE_GRADIENT_METHOD,
  GAUSS_PROCESS_METHOD,
};
enum DifferenceType {
  CPPAD,
  CASADI,
};

enum NonlinearSolverType {
  SELF_SOLVER,
  CERES_SOLVER,
};

enum LeastSquaresSolver {
  GAUSS_NEWTON,
  LEVENBERG_MARQUARDT,
  DOGLEG,
};

}

#endif //PATH_SMOOTHING_TYPE_HPP
