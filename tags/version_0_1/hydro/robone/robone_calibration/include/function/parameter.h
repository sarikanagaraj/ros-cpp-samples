#ifndef PARAMETER_H
#define PARAMETER_H

#include <string>

#include "exception/application_error.h"

namespace org
{
namespace robone
{
namespace slam
{
namespace function
{


class ValueType
{
public:

    enum ParameterType
    {
        INTEGER,
        DOUBLE
    };

private:

    ParameterType type;

    union
    {
        int integerValue;
        double doubleValue;
    } value;

public:

    ValueType();
    ValueType(int integerValue);
    ValueType(double doubleValue);

    operator int();
    operator double();
    ValueType operator =(int integerValue);
    ValueType operator =(double doubleValue);

    ParameterType getType();
};

class Parameter
{

private:

    std::string name;

    ValueType value;
    ValueType minimumValue;
    ValueType maximumValue;

public:

    Parameter(std::string name, int value, int minValue, int maxValue);
    Parameter(std::string name, double value, double minValue, double maxValue);

    std::string& getName();

    ValueType getValue();
    void setValue(ValueType value);

    ValueType getMinValue();
    ValueType getMaxValue();
    ValueType::ParameterType getType();

    std::string getValueAsString();
};

}
}
}
}

#endif // PARAMETER_H
