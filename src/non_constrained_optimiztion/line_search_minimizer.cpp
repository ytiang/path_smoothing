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
//                step_length = previous_state.step_length *
//                        previous_state.directional_derivative /
//                        current_state.directional_derivative;
                step_length =
                        2.0 * (current_state.cost - previous_state.cost)
                                / current_state.directional_derivative;
//                state->step_length = std::min(10*state->step_length,
//                                              1.0/current_state.gradient_norm);
            } else {
                step_length = 1.0 / current_state.gradient_max_norm;
//                step_length =
//                        1.0 / current_state.gradient.lpNorm<Eigen::Infinity>();
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
    std::unique_ptr<LineSearchStepLength> step_length_sercher
            (LineSearchStepLength::Create(line_search_option, &function));
    std::unique_ptr<LineSearchDirection>
            direction_sercher(LineSearchDirection::Create(line_search_option));

    VectorRef param(param_ptr, problem->NumParameters());

    State current_state(problem->NumParameters());
    State previous_state(problem->NumParameters());
    problem->Evaluate(param_ptr, &(current_state.cost),
                      current_state.gradient.data());
    current_state.gradient_norm = current_state.gradient.norm();
    current_state.gradient_max_norm =
            current_state.gradient.lpNorm<Eigen::Infinity>();
    current_state.x_norm = param.norm();
    previous_state.step_length = -1;

    summary->solve_iteration_count = 0;
    summary->search_step_fail_count = 0;
    summary->initial_cost = current_state.cost;
    summary->initial_gradient_norm = current_state.gradient_norm;

#ifdef DEBUG
    summary->step_length_vec.push_back(current_state.step_length);
    summary->cost_vec.push_back(current_state.cost);
    summary->gradient_norm_vec.push_back(current_state.gradient_norm);
    summary->gradient_max_norm_vec.push_back(current_state.gradient_max_norm);
    summary->dir_norm_vec.push_back(0);
    summary->dir_vec.push_back(Eigen::Vector2d::Zero());
    summary->param_vec.push_back(param);
    summary->gradient_vec.push_back(current_state.gradient);
#endif

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
            direction_sercher->
                    NextDirection(previous_state, current_state,
                                  &(current_state.search_direction));
        }
        current_state.directional_derivative =
                current_state.gradient.dot(current_state.search_direction);
        CHECK_LT(current_state.gradient_norm, 1e10)
            << "exit unnormal gradient at iteration: "
            << summary->solve_iteration_count;

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
        // update current state:
//        if (summary->solve_iteration_count == 0) {
//            current_state.step_length = 0.0007879642166174;
//        } else {
            current_state.step_length = summary->step;
//        }
        previous_state = current_state;
        param = param
                + current_state.step_length * current_state.search_direction;
        problem->Evaluate(param.data(), &(current_state.cost),
                          current_state.gradient.data());
        current_state.gradient_norm = current_state.gradient.norm();
        current_state.gradient_max_norm =
                current_state.gradient.lpNorm<Eigen::Infinity>();
        current_state.x_norm = param.norm();
        summary->final_cost = current_state.cost;
        summary->final_gradient_norm = current_state.gradient_norm;
        summary->solve_iteration_count++;

#ifdef DEBUG
        summary->step_length_vec.push_back(current_state.step_length);
        summary->cost_vec.push_back(current_state.cost);
        summary->gradient_norm_vec.push_back(current_state.gradient_norm);
        summary->gradient_max_norm_vec.push_back(current_state.gradient_max_norm);
        summary->dir_norm_vec.push_back(current_state.search_direction.norm());
        summary->dir_vec.push_back(current_state.search_direction);
        summary->param_vec.push_back(param);
        summary->gradient_vec.push_back(current_state.gradient);
#endif
    }
    summary->termination_type = ITERATION_COUNT_LIMITED;
}

}