//
// Created by yangt on 18-12-21.
//

#ifndef PATH_SMOOTHING_LINE_SEARCH_DIRECTION_HPP
#define PATH_SMOOTHING_LINE_SEARCH_DIRECTION_HPP

#include "type.hpp"
#include "line_search_minimizer.hpp"

namespace ncopt {

class LineSearchDirection {
public:

    static LineSearchDirection *Create(const LineSearchOption &options, int dim);

    virtual ~LineSearchDirection() {}

    virtual void NextDirection(const LineSearchMinimizer::State &previous,
                               const LineSearchMinimizer::State &current,
                               Vector *search_direction) = 0;
};

class SteepdestDescentDirection : public LineSearchDirection {
public:
    virtual ~SteepdestDescentDirection() {}
    virtual void NextDirection(const LineSearchMinimizer::State &previous,
                               const LineSearchMinimizer::State &current,
                               Vector *search_direction) {
        *search_direction = -current.gradient;
    }
};

class NonConjugateDirection : public LineSearchDirection {
 public:
    NonConjugateDirection(const NonlinearConjugateGradientType type);
    virtual ~NonConjugateDirection() {}
    virtual void NextDirection(const LineSearchMinimizer::State &previous,
                               const LineSearchMinimizer::State &current,
                               Vector *search_direction);

 private:
    const NonlinearConjugateGradientType type_;
};

class QuasiNewtonDirection : public LineSearchDirection {
 public:
  QuasiNewtonDirection(const QuasiNewtonType type, int dim);
  virtual ~QuasiNewtonDirection() {}
  virtual void NextDirection(const LineSearchMinimizer::State &previous,
                             const LineSearchMinimizer::State &current,
                             Vector *search_direction);

 private:
  const QuasiNewtonType type_;
  Matrix H_inv_;
  Matrix identity_;
  const int dim_;
};

}
#endif //PATH_SMOOTHING_LINE_SEARCH_DIRECTION_HPP
