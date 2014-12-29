#ifndef APPLICATION_ERROR_H
#define APPLICATION_ERROR_H

#include <string>

namespace org
{
namespace robone
{
namespace slam
{
namespace exception
{

class ApplicationError
{
private:

    std::string message;

public:

    ApplicationError(std::string message)
    {
        this->message = message;
    }

    std::string& getMessage()
    {
        return this->message;
    }
};

}
}
}
}

#endif // APPLICATION_ERROR_H
