//
// Created by yangt on 18-12-23.
//

#ifndef PATH_SMOOTHING_FIRST_ORDER_PROBLEM_HPP
#define PATH_SMOOTHING_FIRST_ORDER_PROBLEM_HPP
#include <iostream>
namespace ncopt {

class GradientProblem {
 public:
  virtual int NumParameters() const = 0;

  virtual bool Evaluate(const double *current_state, double *cost,
                        double *gradient) const = 0;

  virtual ~GradientProblem() {
  }
};

}
#endif //PATH_SMOOTHING_FIRST_ORDER_PROBLEM_HPP
