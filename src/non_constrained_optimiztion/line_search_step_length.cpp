//
// Created by yangt on 18-12-21.
//
#include "non_constrained_optimiztion/line_search_step_length.hpp"
#include "opt_utils/csv_writer.hpp"
namespace ncopt {
StepLengthFunction::StepLengthFunction(GradientProblem *problem)
        : problem_(problem),
          position_(problem->NumParameters()),
          direction_(problem->NumParameters()) {
}

void StepLengthFunction::Evaluate(double step,
                                  bool is_evaluate_gradient,
                                  Samples *output) {
    output->a = step;
    output->vector_x = position_ + step * direction_;

    if (!is_evaluate_gradient) {
        problem_->Evaluate(output->vector_x.data(), &(output->value), NULL);
        output->is_gradient_valid = false;
    } else {
        output->vector_gradient.resize(position_.size());
        problem_->Evaluate(output->vector_x.data(),
                           &(output->value),
                           output->vector_gradient.data());
        output->gradient = output->vector_gradient.dot(direction_);
        output->is_gradient_valid = true;
    }
    output->is_value_valid = true;
}

LineSearchStepLength::LineSearchStepLength(const LineSearchOption &option,
                                           StepLengthFunction *function)
        : options_(option), function_(function) {}

LineSearchStepLength *LineSearchStepLength::
Create(const LineSearchOption &options, StepLengthFunction *function) {
    switch (options.line_search_type) {
        case ARMIJO:return new ArimjoSearch(options, function);
        case WOLFE:return new WolfSearch(options, function);
    }
}

double LineSearchStepLength::EvaluatePolynomial(const Vector &polynomial,
                                                double x) const {
    double v = 0.0;
    for (int i(0); i < polynomial.size(); ++i) {
        v = v * x + polynomial(i);
    }
    return v;
}

void LineSearchStepLength::FindPolynomialRoots(
        const Vector &polynomial, Vector *roots) const {
    int degree = polynomial.size() - 1;
//    CHECK_EQ(degree - 1, roots->size())
//        << "polynomial degree " << degree
//        << " is not coincide with roots degree " << roots->size();
    if (degree == 2) {
        roots->resize(1);
        if (fabs(polynomial(0)) < 1e-6) {
            (*roots)(0) = std::numeric_limits<double>::infinity();
        } else {
            (*roots)(0) = -polynomial(1) / 2 / polynomial(0);
        }
    }
    if (degree == 3) {
        if (fabs(polynomial(0)) < 1e-6 && fabs(polynomial(1)) < 1e-6) {
            roots->resize(1);
            (*roots)(0) = std::numeric_limits<double>::infinity();
        } else if (fabs(polynomial(0)) < 1e-6) {
            roots->resize(1);
            (*roots)(0) = -polynomial(1) / 2 / polynomial(0);
        } else {
            double judge =
                    pow(polynomial(1), 2) - 3 * polynomial(0) * polynomial(2);
            if (judge <= 0) {
                roots->resize(1);
                (*roots)(0) = -polynomial(1) / 3 / polynomial(0);
            } else {
                roots->resize(2);
                (*roots)(0) = -polynomial(1) / 3 / polynomial(0) + sqrt(judge);
                (*roots)(1) = -polynomial(1) / 3 / polynomial(0) - sqrt(judge);
            }
        }
    }
}
Vector LineSearchStepLength::PolynomialInterpolating(
        const std::vector<Samples> &samples) const {
    int degree = 0;
    if (options().interpolation_type == CUBIC) {
        degree = 4;
    } else if (options().interpolation_type == QUADRATIC) {
        degree = 3;
    }
    Matrix M(degree, degree);
    Vector rhs(degree);
    int row = 0;
    for (int i(0); i < samples.size(); ++i) {
        const auto &sample = samples.at(i);
        if (sample.is_value_valid) {
            for (int j(0); j < degree; ++j) {
                M(row, j) = pow(sample.a, degree - 1 - j);
            }
            rhs(row) = sample.value;
            row++;
            if (row == degree) {
                break;
            }
        }
        if (sample.is_gradient_valid) {
            for (int j(0); j < degree - 1; ++j) {
                M(row, j) = (degree - 1 - j) * pow(sample.a, degree - 2 - j);
            }
            rhs(row) = sample.gradient;
            row++;
            if (row == degree) {
                break;
            }
        }
    }
    return M.lu().solve(rhs);
//    Eigen::FullPivLU<Matrix> lu(M);
//    return lu.setThreshold(0.0).solve(rhs);
}

double LineSearchStepLength::InterpolateMinimizingStepLength(
        const State &initial_state,
        const Samples &sample0,
        const Samples &sample1,
        const double lower_step,
        const double upper_step) const {
    if (options().interpolation_type == BISECTION) {
        return (sample0.a + sample1.a) / 2.0;
    }
    std::vector<Samples> samples;
    samples.push_back(sample0);
    samples.push_back(sample1);
    samples.push_back(Samples(0, initial_state.cost,
                              initial_state.directional_derivative));

    const Vector polynomial = PolynomialInterpolating(samples);

    if (polynomial.size() < 3) {
        std::cout << polynomial << "\n";
    }

    const double lower_step_value = EvaluatePolynomial(polynomial, lower_step);
    const double upper_step_value = EvaluatePolynomial(polynomial, upper_step);
    double step_optimal = (lower_step + upper_step) / 2.0;
    double step_optimal_valuel = EvaluatePolynomial(polynomial, step_optimal);

    if (step_optimal_valuel > lower_step_value) {
        step_optimal_valuel = lower_step_value;
        step_optimal = lower_step;
    }
    if (step_optimal_valuel > upper_step_value) {
        step_optimal_valuel = upper_step_value;
        step_optimal = upper_step;
    }
    Vector roots;
    FindPolynomialRoots(polynomial, &roots);
    for (int i(0); i < roots.size(); ++i) {
        if (roots(i) < lower_step || roots(i) > upper_step) {
            continue;
        }
        const double value = EvaluatePolynomial(polynomial, roots(i));
        if (value < step_optimal_valuel) {
            step_optimal_valuel = value;
            step_optimal = roots(i);
        }
    }
    return step_optimal;
}

ArimjoSearch::ArimjoSearch(const LineSearchOption &option,
                           StepLengthFunction *function)
        : LineSearchStepLength(option, function) {

}

bool ArimjoSearch::DoSearch(const State &initial_state,
                            Summary *summary) {
    // set some parameters of inputing initial state
    summary->line_search_iteration_count = 0;
    StepLengthFunction::Samples current, previous;
    bool is_evaluate_gradient = false;
    if (options().interpolation_type == CUBIC) {
        is_evaluate_gradient = true;
    }
    function()->Evaluate(summary->initial_step, is_evaluate_gradient, &current);

#ifdef DEBUG
    summary->initial_step_length_vec.push_back(summary->initial_step);
#endif

    while (true) {
        // sufficient decrease condition:
        if (current.value < initial_state.cost + options().sufficient_decrease
                * current.a * initial_state.directional_derivative) {
            break;
        }
        // step length searching limit:
        if (current.a < options().min_line_search_step_length) {
            LOG(WARNING) << "step length " << current.a
                         << " is less than setting threshold "
                         << options().min_line_search_step_length;
            return false;
        }
        // choose optimal step length by funciton approximation
        double new_step = InterpolateMinimizingStepLength(
                initial_state,
                previous,
                current,
                current.a * options().max_step_decrease_rate,
                current.a * options().min_step_decrease_rate);

        // update samples
        previous = current;
        function()->Evaluate(new_step, is_evaluate_gradient, &current);
        summary->line_search_iteration_count++;
    }
    summary->step = current.a;
#ifdef DEBUG
    summary->step_length_vec.push_back(summary->step);
    summary->line_search_iterations_vec.push_back(summary->line_search_iteration_count);
    summary->cost_vec.push_back(current.value);
    summary->gradient_norm_vec.push_back(current.vector_gradient.norm());
#endif
    return true;
}

WolfSearch::WolfSearch(const LineSearchOption &option,
                       StepLengthFunction *function)
        : LineSearchStepLength(option, function) {

}

bool WolfSearch::Zoom(const State &initial_state,
                        Samples *s_lo,
                        Samples *s_hi,
                        double *step,
                        Summary *summary) {
    // interpolate between al and ah, than find min ai
    const double &cost0 = initial_state.cost;
    const double &dird0 = initial_state.directional_derivative;
    const double &c1 = options().sufficient_decrease;
    const double &c2 = options().sufficient_curvature_decrease;
    double lower_step;
    double upper_step;
    Samples current;
    while (true) {
        // choose optimal step length by approximation
        lower_step =
                std::min(s_lo->a, s_hi->a) / options().min_step_decrease_rate;
        upper_step =
                std::max(s_lo->a, s_hi->a) * options().min_step_decrease_rate;
        *step =
                InterpolateMinimizingStepLength(initial_state, *s_lo, *s_hi,
                                                lower_step, upper_step);
        function()->Evaluate(*step, true, &current);
        if(fabs(s_hi->a - s_lo->a) < 1e-7) {
            LOG(WARNING)
                << "Zoom Section: [" << s_lo->a << ", "
                << s_hi->a << "] doesn't contain aviable step length!!";
            return false;
        }
        if (current.value > cost0 + c1 * current.a * dird0
                || current.value >= s_lo->value) {
            *s_hi = current;
        } else {
            // curvature sufficient decrease condition:
            if (fabs(current.gradient) <= -c2 * dird0) {
                *step = current.a;
                return true;
            }
            if (current.gradient * (s_hi->a - s_lo->a) >= 0) {
                *s_hi = *s_lo;
            }
            *s_lo = current;
        }
        summary->line_search_iteration_count++;
    }
}

bool WolfSearch::DoSearch(const State &initial_state, Summary *summary) {
    const double &cost0 = initial_state.cost;
    const double &dird0 = initial_state.directional_derivative;
    const Vector &direction = initial_state.search_direction;
    const double &c1 = options().sufficient_decrease;
    const double &c2 = options().sufficient_curvature_decrease;

    summary->line_search_iteration_count = 1;
    Samples current;//, upper;
    Samples previous
            (0, initial_state.cost, initial_state.directional_derivative);
    function()->Evaluate(summary->initial_step, true, &current);
//    function()->Evaluate(summary->initial_step, true, &upper);
    double step = 0.0;

    while (true) {
        if (current.value > cost0 + c1 * current.a * dird0 ||
                (current.value >= previous.value &&
                        summary->line_search_iteration_count > 1)) {
            if(!Zoom(initial_state, &previous, &current, &step, summary)) {
                return false;
            }
            break;
        }

        // curvature sufficient decrease condition
        if (fabs(current.gradient) <= -c2 * dird0) {
            step = current.a;
            break;
        }
        if (current.gradient >= 0) {
            if(!Zoom(initial_state, &current, &previous, &step, summary)) {
                return false;
            }
            break;
        }
        // choose step length by approximation
        double new_step = InterpolateMinimizingStepLength(
                initial_state,
                previous,
                current,
                current.a,
                current.a * 10);
        previous = current;
        function()->Evaluate(new_step, true, &current);
        summary->line_search_iteration_count++;
    }
    summary->step = step;
    return true;
}

}
