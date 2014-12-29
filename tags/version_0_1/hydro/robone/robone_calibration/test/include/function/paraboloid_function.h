#include "function/multi_value_function.h"

#include <vector>
#include "function/parameter.h"

using namespace org::robone::slam::function;

class ParaboloidFunction : public MultiValueFunction
{

public:

    virtual double getValue(std::vector<Parameter> &parameters);
};
