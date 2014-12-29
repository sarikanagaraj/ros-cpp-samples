#ifndef COORDINATE_DESCENT_H
#define COORDINATE_DESCENT_H

#include <vector>
#include <math.h>
#include <stdlib.h>

#include "function/parameter.h"
#include "function/single_value_function.h"
#include "function/multi_value_function.h"
#include "exception/application_error.h"

using namespace org::robone::slam::function;
using namespace org::robone::slam::exception;

namespace org
{
namespace robone
{
namespace slam
{
namespace algorithm
{

template <class T>
class CoordinateDescent
{

private:

    double epsilon;
    unsigned integerParameterEpsilon;

    unsigned maxIterationsCount;
    unsigned iterationsCounter;


    bool stopCriteriaMet(const double &minFunctionValue, const double &previousMinFunctionValue);
    int integerDichotomyOptimization(SingleValueFunction<T> &singleValueFunction, int a, int b);
    double doubleGoldenRatioOptimization(SingleValueFunction<T> &singleValueFunction,
                                                            double a, double b);

public:

    CoordinateDescent(const double epsilon = 0.1, const unsigned integerParameterEpsilon = 2,
                      const unsigned maxIterationsCount = 30):epsilon(epsilon),
                    integerParameterEpsilon(integerParameterEpsilon),maxIterationsCount(maxIterationsCount)
    {
    }

    std::vector<Parameter>& optimize(T &qualityFunction, std::vector<Parameter> &parameters);
};


template <class T>
bool CoordinateDescent<T>::stopCriteriaMet(const double &minFunctionValue, const double &previousMinFunctionValue)
{
    return (fabs(minFunctionValue - previousMinFunctionValue) < epsilon) || (0 == this->iterationsCounter);
}

template <class T>
int CoordinateDescent<T>::integerDichotomyOptimization(SingleValueFunction<T> &singleValueFunction,
                                                        int a, int b)
{
    int x1 = (a + b - 1) / 2;
    int x2 = (a + b + 1) / 2;

    double f1 = singleValueFunction.getValue(x1);
    double f2 = singleValueFunction.getValue(x2);

    while (abs(b - a) > integerParameterEpsilon) {

        if (f1 >= f2) {
            a = x1;
        }
        else {
            b = x2;
        }

        x1 = (a + b - 1) / 2;
        if (x1 < a) {
            x1 = a;
        }

        x2 = (a + b + 1) / 2;
        if (x2 > b) {
            x2 = b;
        }

        f1 = singleValueFunction.getValue(x1);
        f2 = singleValueFunction.getValue(x2);
    }

    return (a + b) / 2;
}

template <class T>
double CoordinateDescent<T>::doubleGoldenRatioOptimization(SingleValueFunction<T> &singleValueFunction,
                                                        double a, double b)
{
    double t = (3.0 - sqrt(5.0)) / 2.0;
    double c = a + t * (b - a);
    double d = a + b - c;

    double fc = singleValueFunction.getValue(c);
    double fd = singleValueFunction.getValue(d);

    while (fabs(b - a) > epsilon) {

        if (fc < fd) {
            b = d;
            d = c;
            c = a + t * (b - a);
        }
        else {
            a = c;
            c = d;
            d = b - t * (b - a);
        }

        if (fabs(b - a) > epsilon) {

            if (fc < fd) {
                fd = fc;
                fc = singleValueFunction.getValue(c);
            }
            else {
                fc = fd;
                fd = singleValueFunction.getValue(d);
            }
        }
    }

    return (a + b) / 2.0;
}

template <class T>
std::vector<Parameter>& CoordinateDescent<T>::optimize(T &optimizedFunction,
                                     std::vector<Parameter> &parameters)
{
    this->iterationsCounter = this->maxIterationsCount;
    double minFunctionValue = optimizedFunction.getValue(parameters);
    double previousMinFunctionValue = minFunctionValue + this->epsilon * 1000;

    while (!stopCriteriaMet(minFunctionValue, previousMinFunctionValue)) {

        for (unsigned i = 0; (i < parameters.size()) && !stopCriteriaMet(minFunctionValue, previousMinFunctionValue); i++) {

            previousMinFunctionValue = minFunctionValue;
            SingleValueFunction<T> singleValueFunction(optimizedFunction, parameters, i);

            if (ValueType::INTEGER == parameters[i].getType()) {
                int value = integerDichotomyOptimization(singleValueFunction, (int)parameters[i].getMinValue(),
                                                         (int)parameters[i].getMaxValue());
                parameters[i].setValue(value);
            }
            else if (ValueType::DOUBLE == parameters[i].getType()) {
                double value = doubleGoldenRatioOptimization(singleValueFunction, (double)parameters[i].getMinValue(),
                                                                 (double)parameters[i].getMaxValue());
                parameters[i].setValue(value);
            }
            else {
                throw ApplicationError("Unsupported parameter type");
            }
            minFunctionValue = optimizedFunction.getValue(parameters);
        }

        this->iterationsCounter--;
    }

    return parameters;
}


}
}
}
}


#endif // COORDINATE_DESCENT_H
