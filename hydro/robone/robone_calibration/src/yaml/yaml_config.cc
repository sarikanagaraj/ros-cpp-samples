#include "yaml/yaml_config.h"

#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "util/string_utils.h"
#include "exception/application_error.h"

using namespace org::robone::slam::util;
using namespace org::robone::slam::exception;

namespace org
{
namespace robone
{
namespace slam
{
namespace yaml
{

void YamlConfig::loadFile()
{
    std::ifstream file(this->fileName.c_str());
    std::string line;
    while (std::getline(file, line)) {
        content.push_back(line);
    }
}

int YamlConfig::findParameterIndex(std::string &name)
{
    for (int i = 0; i < content.size(); i++) {

        std::string line = content[i];
        size_t commentIndex = line.find('#');
        if (std::string::npos != commentIndex) {
            line = line.substr(0, commentIndex);
        }

        size_t index = line.find(name);

        if (index >= 0) {

            if ((0 == index) || (' ' == line[index - 1])) {

                size_t nextCharacterIndex = index + name.length();
                if (nextCharacterIndex == line.find_first_of(" :", nextCharacterIndex)) {

                    return i;
                }
            }
        }
    }

    return -1;
}

YamlConfig::YamlConfig(std::string &fileName)
{
    this->fileName = fileName;

    loadFile();
}

bool YamlConfig::loadParameter(Parameter &parameter)
{
    bool found = false;

    std::string name = parameter.getName();

    int contentIndex = findParameterIndex(name);

    if (contentIndex > -1) {

        found = true;

        std::string line = content[contentIndex];

        size_t indexOfColon = line.rfind(':');

        std::string value = line.substr(indexOfColon + 1);
        trim(value);

        switch (parameter.getType()) {

            case ValueType::INTEGER: {
                int integerValue = atoi(value.c_str());
                parameter.setValue(integerValue);
                break;
            }

            case ValueType::DOUBLE: {

                double doubleValue = atof(value.c_str());
                parameter.setValue(doubleValue);
                break;
            }

            default:
                throw ApplicationError("Unknown type of parameter");
        }
    }

    return found;
}

void YamlConfig::saveParameter(Parameter &parameter)
{
    std::string name = parameter.getName();

    int contentIndex = findParameterIndex(name);

    if (contentIndex > -1) {

        std::string line = content[contentIndex];

        size_t indexOfColon = line.rfind(':');
        size_t indexOfValue = line.find_first_not_of(" \t", indexOfColon + 1);

        line = line.substr(0, indexOfValue);
        line.append(parameter.getValueAsString());
        content[contentIndex] = line;
    }
    else {
        size_t indexOfParameterNameStart = 2;
        if (content.size() > 0) {
            std::string line = content[content.size() - 1];
            indexOfParameterNameStart = line.find_first_not_of(" ");
        }

        std::string blankPrefix(indexOfParameterNameStart, ' ');
        std::string serializedParameter = blankPrefix + parameter.getName();
        serializedParameter.append(" : ");
        serializedParameter.append(parameter.getValueAsString());
        content.push_back(serializedParameter);
    }
}

void YamlConfig::flushToDisk()
{
    std::ofstream file(this->fileName.c_str());

    for (int i = 0; i < content.size(); i++) {
        file << content[i];
        if (i < (content.size() - 1)) {
            file << std::endl;
        }
    }
}

}
}
}
}
