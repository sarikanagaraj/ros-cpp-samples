#include "util/string_utils.h"

#include <gtest/gtest.h>

using namespace org::robone::slam::util;

TEST(StringUtilsSuite, trim)
{
    std::string string1 = " te st   \t";
    std::string string2 = "te st   ";
    std::string string3 = "\t\t\tte st";

    std::string expectedResult = "te st";

    ASSERT_EQ(trim(string1), expectedResult);
    ASSERT_EQ(trim(string2), expectedResult);
    ASSERT_EQ(trim(string3), expectedResult);
}

TEST(StringUtilsSuite, integerToString)
{
    const unsigned int VALUES_COUNT = 3;
    int values[VALUES_COUNT] = {1231, -100, 0};

    for (unsigned i = 0; i < VALUES_COUNT; i++) {
        ASSERT_EQ(atoi(integerToString(values[i]).c_str()), values[i]);
    }
}

TEST(StringUtilsSuite, doubleToString)
{
    const unsigned int VALUES_COUNT = 3;
    double values[VALUES_COUNT] = {1231, -100, 0};

    for (unsigned i = 0; i < VALUES_COUNT; i++) {
        ASSERT_EQ(atof(doubleToString(values[i]).c_str()), values[i]);
    }
}

TEST(StringUtilsSuite, longToString)
{
    const unsigned int VALUES_COUNT = 3;
    long values[VALUES_COUNT] = {1231, -100, 0};

    for (unsigned i = 0; i < VALUES_COUNT; i++) {
        ASSERT_EQ(atol(doubleToString(values[i]).c_str()), values[i]);
    }
}

TEST(StringUtilsSuite, timestampString)
{
    std::string timestamp1 = timestampString();
    std::string timestamp2 = timestampString();

    ASSERT_GE(atol(timestamp2.c_str()), atol(timestamp1.c_str()));

    std::string timestamp3 = timestampString();

    ASSERT_GE(atol(timestamp3.c_str()), atol(timestamp2.c_str()));
}

