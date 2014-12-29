#ifndef MULTI_VALUE_FUNCTION_H
#define MULTI_VALUE_FUNCTION_H

#include <vector>

#include "function/parameter.h"

namespace org
{
namespace robone
{
namespace slam
{
namespace function
{


class MultiValueFunction
{

public:

    virtual double getValue(std::vector<Parameter> &parameters) = 0;
};

}
}
}
}


#endif // MULTI_VALUE_FUNCTION_H
