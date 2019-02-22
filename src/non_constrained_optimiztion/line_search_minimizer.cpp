//
// Created by yangt on 18-12-21.
//
#pragma once
#include "non_constrained_optimiztion/line_search_minimizer.hpp"
#include "non_constrained_optimiztion/line_search_step_length.hpp"
#include "non_constrained_optimiztion/line_search_direction.hpp"

namespace ncopt {

LineSearchMinimizer::LineSearchMinimizer(
        const GradientProblemOption &options)
        : minimizer_option_(options) {

}

void LineSearchMinimizer::GetSummaryFromState(const State &state,
                                              Summary *summary) const {
    summary->gradient_norm_value = state.gradient_norm;
    summary->cost_function_value = state.cost;
}

double LineSearchMinimizer::GetInitialStepLength(const State &previous_state,
                                                 const State &current_state,
                                                 const Summary &summary) const {
    double step_length = 0.0;
    switch (minimizer_option_.line_search_direction_type) {
        case BFGS:
        case LBFGS:step_length = 1.0;
            break;
        case STEEPEST_DESCENT:
        case NONLINEAR_CONJUGATE_GRADIENT:
            if (summary.solve_iteration_count > 0) {
//                state->step_length = previous_state.step_length *
//                        previous_state.directional_derivative /
//                        current_state.directional_derivative;
                step_length =
                        2 * (current_state.cost - previous_state.cost)
                                / current_state.directional_derivative;
//                state->step_length = std::min(10*state->step_length,
//                                              1.0/current_state.gradient_norm);
            } else {
                step_length = 1.0 / current_state.gradient_norm;
            }
            step_length = std::min(step_length, 1.0);
            break;
    }
    return step_length;
}

bool LineSearchMinimizer::Minimize(double *param_ptr,
                                   GradientProblem *problem,
                                   Summary *summary) {
    LineSearchOption line_search_option = minimizer_option_;
    StepLengthFunction function(problem);
    LineSearchStepLength *step_length_sercher =
            LineSearchStepLength::Create(line_search_option, &function);
    LineSearchDirection *direction_sercher =
            LineSearchDirection::Create(line_search_option);

    State current_state(problem->NumParameters());
    State previous_state(problem->NumParameters());
    problem->Evaluate(param_ptr, &(current_state.cost),
                      current_state.gradient.data());
    current_state.gradient_norm = current_state.gradient.norm();
    previous_state.step_length = -1;

    VectorRef param(param_ptr, problem->NumParameters());
    summary->solve_iteration_count = 0;
    summary->search_step_fail_count = 0;

    while (summary->solve_iteration_count <
            minimizer_option_.max_solve_iterations_num) {
        // terminating conditions:
        if (IsConvergent(current_state, previous_state, summary)) {
            return true;
        }

        // calculate search direction:
        if (summary->solve_iteration_count == 0) {
            current_state.search_direction = -current_state.gradient;
        } else {
            // restart strategy
            if (previous_state.gradient.dot(current_state.gradient) /
                    pow(current_state.gradient_norm, 2) < 0.1) {
                current_state.search_direction = -current_state.gradient;
            } else {
                direction_sercher->
                        NextDirection(previous_state, current_state,
                                      &(current_state.search_direction));
            }
        }
        current_state.directional_derivative =
                current_state.gradient.dot(current_state.search_direction);

        // calculate search step length:
        // estimate initial step length. This is improtant for bad scaling
        // problem when use non-Netown type methods.
        summary->initial_step =
                GetInitialStepLength(previous_state, current_state, *summary);
        function.Init(param, current_state.search_direction);
        if (!(step_length_sercher->DoSearch(current_state, summary))) {
            summary->search_step_fail_count++;
            current_state.search_direction = -current_state.gradient;
            current_state.directional_derivative =
                    current_state.gradient.dot(current_state.search_direction);
            if (summary->search_step_fail_count > 20
                    || !(step_length_sercher->DoSearch(current_state,
                                                       summary))) {
                summary->message = "No aviliable step length!";
                summary->termination_type = NO_AVILIABLE_STEP_LENGTH;
                return false;
            }
        }

        previous_state = current_state;
        // update current state:
        current_state.step_length = summary->step;
        param = param
                + current_state.step_length * current_state.search_direction;
        problem->Evaluate(param.data(), &(current_state.cost),
                          current_state.gradient.data());
        current_state.gradient_norm = current_state.gradient.norm();
        summary->cost_function_value = current_state.cost;
        summary->gradient_norm_value = current_state.gradient_norm;
        summary->solve_iteration_count++;
    }
    summary->termination_type = ITERATION_COUNT_LIMITED;
}

}