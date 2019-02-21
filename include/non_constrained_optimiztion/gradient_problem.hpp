//
// Created by yangt on 18-12-23.
//

#ifndef PATH_SMOOTHING_FIRST_ORDER_PROBLEM_HPP
#define PATH_SMOOTHING_FIRST_ORDER_PROBLEM_HPP
#include <iostream>
namespace ncopt {

class GradientProblem {
 public:
  virtual int NumberOfParams() = 0;

  virtual bool Evaluate(double *current_state, double *cost,
                        double *gradient) = 0;

  virtual ~GradientProblem() {
      std::cout << "GradientProblem Over!!!\n";
  }
};

}
#endif //PATH_SMOOTHING_FIRST_ORDER_PROBLEM_HPP
