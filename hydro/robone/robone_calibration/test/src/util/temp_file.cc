#include <stdio.h>
#include <string>

#include "util/temp_file.h"

TemporaryFile::TemporaryFile()
{
    char buffer[L_tmpnam];
    tmpnam(buffer);
    fileHandler = fopen(buffer, "wb+");
    fileName = buffer;
}

void TemporaryFile::write(std::string content)
{
    fputs(content.c_str(), fileHandler);
}

void TemporaryFile::writeLine(std::string content)
{
    this->write(content);
    this->write("\n");
}


void TemporaryFile::save()
{
    fflush(fileHandler);
}

void TemporaryFile::clearFile()
{
    freopen(fileName.c_str(), "wb+", fileHandler);
}

std::string& TemporaryFile::getFileName()
{
    return fileName;
}

TemporaryFile::~TemporaryFile()
{
    fclose(fileHandler);
    unlink(fileName.c_str());
}

