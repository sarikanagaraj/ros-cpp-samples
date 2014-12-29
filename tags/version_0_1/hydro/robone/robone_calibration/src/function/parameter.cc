#include "function/parameter.h"

#include "util/string_utils.h"

using namespace org::robone::slam::exception;
using namespace org::robone::slam::util;

namespace org
{
namespace robone
{
namespace slam
{
namespace function
{

ValueType::ValueType()
{
    value.integerValue = 0;
    type = INTEGER;
}

ValueType::ValueType(int integerValue)
{
    (*this) = integerValue;
}

ValueType::ValueType(double doubleValue)
{
    (*this) = doubleValue;
}

ValueType::operator int()
{
    if (INTEGER == type) {
        return value.integerValue;
    }

    throw ApplicationError("ValueType is casted to int but it is of different type");
}

ValueType::operator double()
{
    if (DOUBLE == type) {
        return value.doubleValue;
    }

    throw ApplicationError("ValueType is casted to double but it is of different type");
}

ValueType ValueType::operator =(int integerValue)
{
    value.integerValue = integerValue;
    type = INTEGER;

    return (*this);
}

ValueType ValueType::operator =(double doubleValue)
{
    value.doubleValue = doubleValue;
    type = DOUBLE;

    return (*this);
}

ValueType::ParameterType ValueType::getType()
{
    return type;
}

Parameter::Parameter(std::string name, int value, int minValue, int maxValue)
{
    this->name = name;
    this->value = value;
    this->minimumValue = minValue;
    this->maximumValue = maxValue;
}

Parameter::Parameter(std::string name, double value, double minValue, double maxValue)
{
    this->name = name;
    this->value = value;
    this->minimumValue = minValue;
    this->maximumValue = maxValue;
}

std::string& Parameter::getName()
{
    return name;
}

ValueType Parameter::getValue()
{
    return value;
}

void Parameter::setValue(ValueType value)
{
    this->value = value;
}

ValueType Parameter::getMinValue()
{
    return minimumValue;
}

ValueType Parameter::getMaxValue()
{
    return maximumValue;
}

ValueType::ParameterType Parameter::getType()
{
    return value.getType();
}

std::string Parameter::getValueAsString()
{

    std::string result;

    switch (this->getType()) {

        case ValueType::INTEGER: {

            int integerValue = this->value;
            result = integerToString(integerValue);
            break;
        }

        case ValueType::DOUBLE: {

            double doubleValue = this->value;
            result = doubleToString(doubleValue);
            break;
        }

        default:
            throw ApplicationError("Unsupported type of parameter");
    }

    return result;
}

}
}
}
}
