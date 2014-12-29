#include "util/string_utils.h"

#include <sys/time.h>
#include <stdio.h>

namespace org
{
namespace robone
{
namespace slam
{
namespace util
{

std::string trim(std::string value)
{
    size_t endpos = value.find_last_not_of(" \t");
    if (std::string::npos != endpos) {
        value = value.substr(0, endpos + 1);
    }

    size_t startpos = value.find_first_not_of(" \t");
    if (std::string::npos != startpos) {
        value = value.substr(startpos);
    }

    return value;
}

template <typename T>
std::string valueToString(const char *format, T &value)
{
    const unsigned NUMBER_CONVERTION_BUFFER_SIZE = 200;

    char buffer[NUMBER_CONVERTION_BUFFER_SIZE];

    snprintf(buffer, NUMBER_CONVERTION_BUFFER_SIZE, format, value);

    std::string result(buffer);

    return result;
}

std::string integerToString(int value)
{
    return valueToString("%d", value);
}

std::string longToString(long value)
{
    return valueToString("%ld", value);
}

std::string doubleToString(double value)
{
    return valueToString("%f", value);
}

std::string timestampString()
{
    struct timeval timeStructure;
    gettimeofday(&timeStructure, NULL);
    long timestamp = timeStructure.tv_sec * 1000 + timeStructure.tv_usec / 1000;

    return longToString(timestamp);
}

}
}
}
}
