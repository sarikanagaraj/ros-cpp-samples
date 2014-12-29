#ifndef STRING_UTILS_H
#define STRING_UTILS_H

#include <string>

namespace org
{
namespace robone
{
namespace slam
{
namespace util
{

std::string trim(std::string value);

std::string integerToString(int value);

std::string longToString(long value);

std::string doubleToString(double value);

std::string timestampString();

}
}
}
}


#endif // STRING_UTILS_H
