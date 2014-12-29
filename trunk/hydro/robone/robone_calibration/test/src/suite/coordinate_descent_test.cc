#include "algorithm/coordinate_descent.h"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "function/paraboloid_function.h"

using namespace org::robone::slam::algorithm;

TEST(CoordinateDescentSuite, paraboloid)
{
    const double epsilon = 0.1;

    std::vector<Parameter> parameters;
    Parameter paramInt1("param1", 1, -10, 15);
    Parameter paramInt2("param2", -7, -100, 200);
    Parameter paramDouble1("param3", 11.6, -100.0, 20.0);
    Parameter paramDouble2("param4", 200.0, -150.0, 350.0);
    Parameter paramDouble3("param5", 100.0, -1000.0, 100.0);

    parameters.push_back(paramInt1);
    parameters.push_back(paramInt2);
    parameters.push_back(paramDouble1);
    parameters.push_back(paramDouble2);
    parameters.push_back(paramDouble3);

    ParaboloidFunction optimizedFunction;

    CoordinateDescent<ParaboloidFunction> optimizationAlgorithm(epsilon);

    std::vector<Parameter> result;

    try {
        result = optimizationAlgorithm.optimize(optimizedFunction, parameters);

        ROS_INFO("Found following minimum: ");
        for (unsigned i = 0; i < result.size(); i++) {

            if (ValueType::INTEGER == result[i].getType()) {
                ROS_INFO("  parameter %s: %d", result[i].getName().c_str(), (int)result[i].getValue());
            }
            else if (ValueType::DOUBLE == result[i].getType()) {
                ROS_INFO("  parameter %s: %f", result[i].getName().c_str(), (double)result[i].getValue());
            }
            else {
                ROS_ERROR("  Usupported type for parameter %s", result[i].getName().c_str());
            }
        }

        if (optimizedFunction.getValue(result) < epsilon) {
            ASSERT_TRUE(true);
        }
        else {
            FAIL();
        }

    }
    catch (...) {
        FAIL();
    }
}

