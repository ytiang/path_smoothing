//
// Created by yangt on 18-12-31.
//
#include "non_constrained_optimiztion/type.hpp"
namespace ncopt {

#define CASESTR(x) case x: return #x

const char *TerminationTypeToString(const TerminationType &type) {
    switch (type) {
        CASESTR(GRADIENT_NORM_CONVERGENCE);
        CASESTR(GRADIENT_NORM_DECREMENT_CONVERGENCE);
        CASESTR(COST_DECREMENT_CONVERGENCE);
        CASESTR(NO_CONVERGENCE);
        CASESTR(FAILURE);
        CASESTR(USER_SUCCESS);
        CASESTR(USER_FAILURE);
        CASESTR(ITERATION_COUNT_LIMITED);
        CASESTR(NO_AVILIABLE_STEP_LENGTH);
    }
}

const char *LineSearchDirectionTypeToString(
        const LineSearchDirectionType &type) {
    switch (type) {
        CASESTR(STEEPEST_DESCENT);
        CASESTR(NONLINEAR_CONJUGATE_GRADIENT);
        CASESTR(LBFGS);
        CASESTR(BFGS);
    }
}

const char *LIneSearchStepTypeToString(const LineSearchType &type) {
    switch (type) {
        CASESTR(ARMIJO);
        CASESTR(WOLFE);
    }
}

}
