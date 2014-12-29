#ifndef SINGLE_VALUE_FUNCTION_H
#define SINGLE_VALUE_FUNCTION_H

#include <vector>

#include "function/multi_value_function.h"
#include "function/parameter.h"

namespace org
{
namespace robone
{
namespace slam
{
namespace function
{

template <class T>
class SingleValueFunction
{
private:
    T function;

    std::vector<Parameter> parameters;
    unsigned index;

public:
    SingleValueFunction(T &function, std::vector<Parameter> &parameters,
                        const unsigned index):function(function), parameters(parameters),index(index)
    {
    }

    double getValue(const double value)
    {
        this->parameters[index].setValue(value);
        return this->function.getValue(this->parameters);
    }

    double getValue(const int value)
    {
        this->parameters[index].setValue(value);
        return this->function.getValue(this->parameters);
    }
};

}
}
}
}

#endif // SINGLE_VALUE_FUNCTION_H
