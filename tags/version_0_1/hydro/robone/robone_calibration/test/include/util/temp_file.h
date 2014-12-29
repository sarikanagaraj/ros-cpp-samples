#include <stdio.h>
#include <string>

class TemporaryFile
{
private:
    std::string fileName;
    FILE *fileHandler;

public:

    TemporaryFile();

    void write(std::string content);
    void writeLine(std::string content);
    void save();
    void clearFile();

    std::string& getFileName();

    ~TemporaryFile();
};

