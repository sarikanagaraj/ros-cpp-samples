#ifndef NAVIGATION_QUALITY_FUNCTION_H
#define NAVIGATION_QUALITY_FUNCTION_H

#include <vector>
#include <unistd.h>
#include <string>
#include <tf/transform_listener.h>

#include "parameter.h"
#include "multi_value_function.h"

namespace org
{
namespace robone
{
namespace slam
{
namespace function
{

class NavigationQualityFunction : public MultiValueFunction
{
private:

    std::string yamlConfigPath;
    int initializationTimeSeconds;

    pid_t forkAndExecuteSlamNode(std::string timestamp);
    void waitForTransformation(const tf::TransformListener &listener, const std::string &targetFrame,
                               const std::string &sourceFrame);
    unsigned getMeasurements(double *measurements, unsigned &measurementsCount, const unsigned maxBufferSize);

public:

    NavigationQualityFunction(std::string yamlConfigPath, int initializationTimeSeconds = 60);

    double getValue(std::vector<Parameter> &parameters);
};

}
}
}
}

#endif // NAVIGATION_QUALITY_FUNCTION_H
