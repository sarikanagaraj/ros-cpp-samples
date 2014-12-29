#ifndef YAML_CONFIG_H
#define YAML_CONFIG_H

#include <string>
#include <vector>

#include "function/parameter.h"

using namespace org::robone::slam::function;

namespace org
{
namespace robone
{
namespace slam
{
namespace yaml
{

class YamlConfig
{
    std::string fileName;
    std::vector<std::string> content;

    void loadFile();
    int findParameterIndex(std::string &name);

public:

    YamlConfig(std::string &fileName);

    bool loadParameter(Parameter &parameter);

    void saveParameter(Parameter &parameter);

    void flushToDisk();
};

}
}
}
}

#endif // YAML_CONFIG_H
