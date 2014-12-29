
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include "function/parameter.h"
#include "exception/application_error.h"
#include "algorithm/coordinate_descent.h"
#include "function/navigation_quality_function.h"

using namespace org::robone::slam::algorithm;
using namespace org::robone::slam::function;
using namespace org::robone::slam::exception;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Slam_Gmapping_callibration");

    //TODO: Remove after debug
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    std::vector<Parameter> parameters;
    Parameter linearUpdate("linearUpdate", 0.2, 0.01, 1.0);
    Parameter angularUpdate("angularUpdate", 0.3, 0.01, 1.5);
    Parameter deltaValue("delta", 0.01, 0.005, 0.1);
    Parameter particlesCount("particles", 10, 5, 150);
    Parameter minimumScore("minimumScore", 0.0, 0.0, 600.0);

    parameters.push_back(linearUpdate);
    parameters.push_back(angularUpdate);
    parameters.push_back(deltaValue);
    parameters.push_back(particlesCount);
    parameters.push_back(minimumScore);

    NavigationQualityFunction qualityFunction(
                "/home/john/catkin_ws/src/robone_2dnav/config/slam_gmapping_params.yaml");
    CoordinateDescent<NavigationQualityFunction> optimizationAlgorithm;

    std::vector<Parameter> result;

    try {
        result = optimizationAlgorithm.optimize(qualityFunction, parameters);

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
    }
    catch (ApplicationError error) {
        ROS_ERROR("Error occured: %s", error.getMessage().c_str());
    }

    return 0;
}
