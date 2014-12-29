#include "function/navigation_quality_function.h"

#include <sys/types.h>
#include <signal.h>
#include <algorithm>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

#include "yaml/yaml_config.h"
#include "util/string_utils.h"
#include "exception/application_error.h"

using namespace org::robone::slam::yaml;
using namespace org::robone::slam::util;
using namespace org::robone::slam::exception;

namespace org
{
namespace robone
{
namespace slam
{
namespace function
{

int compareDoubles(const void *x, const void *y)
{
  double xx = *(double*)x;
  double yy = *(double*)y;
  if (xx < yy) {
      return -1;
  }
  if (xx > yy) {
      return  1;
  }
  return 0;
}

NavigationQualityFunction::NavigationQualityFunction(
        std::string yamlConfigPath, int initializationTimeSeconds):yamlConfigPath(yamlConfigPath),
    initializationTimeSeconds(initializationTimeSeconds)
{
}

pid_t NavigationQualityFunction::forkAndExecuteSlamNode(std::string timestamp)
{
    pid_t result = fork();
    if (0 == result) {

        std::string command("roslaunch robone_2dnav gmapping.launch");
        command.append(" > ~/temp/slam_");
        command.append(timestamp);
        command.append(".log  2>&1");

        int errorCode = execl("/bin/sh", "sh", "-c", command.c_str(), (char *) 0);
        if (-1 == errorCode) {
            ROS_ERROR("Could not start gmapping node");
        }
    }

    return result;
}

void NavigationQualityFunction::waitForTransformation(const tf::TransformListener &listener,
        const std::string &targetFrame, const std::string &sourceFrame)
{
    ros::Time last_error = ros::Time::now();
    std::string tf_error;

    while(ros::ok() && !listener.waitForTransform(targetFrame, sourceFrame, ros::Time(),
                                             ros::Duration(0.1), ros::Duration(0.01), &tf_error)) {
      ros::spinOnce();
      if (last_error + ros::Duration(5.0) < ros::Time::now()) {
        ROS_WARN("Waiting on transform from %s to %s, tf error: %s",
            targetFrame.c_str(), sourceFrame.c_str(), tf_error.c_str());
        last_error = ros::Time::now();
      }
    }
}

unsigned NavigationQualityFunction::getMeasurements(double *measurements, unsigned &measurementsCount,
                                                    const unsigned maxBufferSize)
{
    const char* MAP_LINK = "map";
    const char* ODOM_LINK = "odom";

    tf::TransformListener listener;

    waitForTransformation(listener, MAP_LINK, ODOM_LINK);

    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    listener.lookupTransform(MAP_LINK, ODOM_LINK, ros::Time(0), start_transform);

    ros::Duration measurementIntervalDuration(0.5);
    ros::Duration measurementsCollectionTimeDuration(120);
    ros::Time endTime = ros::Time::now() + measurementsCollectionTimeDuration;

    while (ros::ok() && (endTime >= ros::Time::now()) && (measurementsCount <= maxBufferSize)) {

        ros::spinOnce();

        measurementIntervalDuration.sleep();

        try {
            listener.lookupTransform(MAP_LINK, ODOM_LINK, ros::Time(0), current_transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            break;
        }

        double rotationDifference = (current_transform.getRotation() - start_transform.getRotation()).length();
        double shiftDifference = current_transform.getOrigin().distance(start_transform.getOrigin());

        //TODO: Coefficient need to be callibrated
        double distance = shiftDifference + rotationDifference * 1000;
        measurements[measurementsCount] = distance;
        measurementsCount++;

//        ROS_DEBUG("Distance %f, shift difference %f, rotation difference %f", distance, shiftDifference, rotationDifference);
    }

    return measurementsCount;
}

double NavigationQualityFunction::getValue(std::vector<Parameter> &parameters)
{
    YamlConfig config(this->yamlConfigPath);

    std::string debugMessage = "Parameters values: ";

    for (unsigned i = 0; i < parameters.size(); i++) {
        config.saveParameter(parameters[i]);

        debugMessage.append(parameters[i].getName());
        debugMessage.append(" => ");
        debugMessage.append(parameters[i].getValueAsString());
        if (i < (parameters.size() - 1)) {
            debugMessage.append(", ");
        }
    }
    config.flushToDisk();

    ROS_DEBUG("%s", debugMessage.c_str());

    std::string timestamp = timestampString();
    pid_t gmappingNodePid = forkAndExecuteSlamNode(timestamp);

    if (0 == gmappingNodePid) {
        exit(0);
    }

    sleep(initializationTimeSeconds);

    const unsigned MAX_BUFFER_SIZE = 2000;
    double measurements[MAX_BUFFER_SIZE];
    unsigned measurementsCount = 0;
    getMeasurements(measurements, measurementsCount, MAX_BUFFER_SIZE);

    qsort(measurements, measurementsCount, sizeof(double), compareDoubles);

    if (0 == measurementsCount) {
        throw ApplicationError("No measurements were done for SLAM");
    }

    double result = measurements[measurementsCount / 2];
    //TODO: Remove this line when determine issue with ROS_DEBUG disapearence
    printf("%s: %s function value = %f\n", timestamp.c_str(),
           debugMessage.c_str(), result);
    ROS_DEBUG("%s: Function result = %f", timestamp.c_str(), result);

    kill(gmappingNodePid, SIGTERM);

    sleep(1);

    return result;
}

}
}
}
}
