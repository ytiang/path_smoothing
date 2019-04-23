//
// Created by yangt on 18-12-23.
//

#include "non_constrained_optimiztion/line_search_minimizer.hpp"

namespace ncopt {

Minimizer* Minimizer::Create(const GradientProblemOption &options) {
    switch(options.minimizer_type) {
        case LINE_SEARCH :
            return new LineSearchMinimizer(options);
        case TRUST_REGION :
            ;
    }
}

}
