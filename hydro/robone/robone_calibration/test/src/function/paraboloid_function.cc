#include "function/paraboloid_function.h"

#include "function/multi_value_function.h"

#include <vector>
#include "function/parameter.h"

using namespace org::robone::slam::function;

double ParaboloidFunction::getValue(std::vector<Parameter> &parameters)
{
    double result = 0.0;

    for (unsigned i = 0; i < parameters.size(); i++) {

        if (ValueType::INTEGER == parameters[i].getType()) {
            int value = parameters[i].getValue();
            result += value * value;
        }
        else {
            double value = parameters[i].getValue();
            result += value * value;

        }
    }

    return result;
}
