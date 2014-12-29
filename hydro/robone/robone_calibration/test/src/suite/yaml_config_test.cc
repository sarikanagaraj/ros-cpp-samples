#include "yaml/yaml_config.h"

#include <gtest/gtest.h>

#include "util/temp_file.h"
#include "function/parameter.h"

using namespace org::robone::slam::yaml;
using namespace org::robone::slam::function;

TEST(YamlConfigSuite, load)
{
    const int NOT_EXISTING_PARAMETER_VALUE = 11;
    const double ACC_LIM_Y_VALUE = 20.0;

    TemporaryFile tempFile;

    tempFile.writeLine("TrajectoryPlannerROS:");
    tempFile.writeLine("    # Robot Configuration Parameters");
    tempFile.writeLine("    acc_lim_x: 1.5");
    tempFile.writeLine("    acc_lim_y: 1.5");
    tempFile.writeLine("    acc_lim_theta: 2.0");
    tempFile.writeLine("    #commented: 2");
    tempFile.save();

    YamlConfig config(tempFile.getFileName());

    Parameter notExistingParameter("notExisting", NOT_EXISTING_PARAMETER_VALUE, -1, 20);
    Parameter commentedParameter("commented", 1, -1, 2);
    Parameter accLimYParameter("acc_lim_y", ACC_LIM_Y_VALUE, -100.0, 29.0);

    bool result = config.loadParameter(notExistingParameter);
    ASSERT_FALSE(result);

    result = config.loadParameter(commentedParameter);
    ASSERT_FALSE(result);

    result = config.loadParameter(accLimYParameter);
    ASSERT_TRUE(result);
    ASSERT_NE(ACC_LIM_Y_VALUE, (double)accLimYParameter.getValue());

    accLimYParameter.setValue(ACC_LIM_Y_VALUE);
    config.saveParameter(accLimYParameter);
    config.flushToDisk();

    YamlConfig newConfig(tempFile.getFileName());

    result = newConfig.loadParameter(accLimYParameter);
    ASSERT_TRUE(result);
    ASSERT_EQ(ACC_LIM_Y_VALUE, (double)accLimYParameter.getValue());

    newConfig.saveParameter(notExistingParameter);
    newConfig.flushToDisk();

    YamlConfig newestConfig(tempFile.getFileName());

    result = newestConfig.loadParameter(notExistingParameter);
    ASSERT_TRUE(result);
    ASSERT_EQ(NOT_EXISTING_PARAMETER_VALUE, (int)notExistingParameter.getValue());
}
