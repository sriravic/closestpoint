#ifndef __UTIL_H__
#define __UTIL_H__

#include <algorithm>
#include <common.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <malloc.h>
#include <matrix.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <vector>
#include <vector3.h>

enum class TimerUnits
{
    TIMER_NS,       // nanoseconds
    TIMER_MUS,      // microseconds
    TIMER_MS,       // milliseconds
    TIMER_S         // seconds
};

class Timer
{
public:
    using Clock     = std::chrono::high_resolution_clock;
    using TimeStamp = std::chrono::high_resolution_clock::time_point;

    Timer(const std::string& name)
        : myName(name)
    {
    }
    
    void start();
    void stop();
    void printElapsed(TimerUnits units = TimerUnits::TIMER_MS) const;
private:
    std::string     myName;
    Clock           myclock;
    TimeStamp       myStart, myEnd;
};

/// Parses a string of type [x,y,z,w,...,a] and returns numel elements
/// in return type
template<typename DataType, typename ReturnType>
static ReturnType
getElements(const std::string& input, int numel)
{
    std::string buffer = input;
    buffer.erase(std::remove(buffer.begin(), buffer.end(), '['), buffer.end());
    buffer.erase(std::remove(buffer.begin(), buffer.end(), ']'), buffer.end());

    ReturnType ret;
    std::stringstream clean(buffer);
    for (int i = 0; i < numel && !clean.eof(); i++)
    {
        std::string temp;
        std::getline(clean, temp, ',');
        if (temp.length() != 0)
            ret[i] = static_cast<DataType>(std::atof(temp.c_str()));
        else
            ret[i] = DataType(0);
        assert(isValid(ret[i]));
    }
    return ret;
}

/// Given a string in [x,y,z] format, convert it to a vector
template<
    typename DataType,
    typename VectorType = Vector3<DataType>>
static VectorType
getVector(const std::string& input)
{
    return getElements<DataType, VectorType>(input, 3);
}

/// Given a file containing vectors in [x,y,z] in a per line format
/// return the list of parsed vectors
template<
    typename DataType,
    typename VectorType = Vector3<DataType>>
static void
getVectors(const std::string& filename, std::vector<VectorType>& output)
{
    std::ifstream input;
    input.open(filename);
    if (input)
    {
        while (!input.eof())
        {
            std::string line;
            std::getline(input, line, '\n');
            output.emplace_back(getVector<DataType>(line));
        }
    }
    else
        std::cerr << "Unable to open file : " << filename;
    input.close();
}

/// Given a string in [m00,m01,m02,m03,.....,m33] format
/// parse it and provide a matrix
template<
    typename DataType,
    typename MatrixType = Matrix4<DataType>>
static void
getMatrix(const std::string& input, MatrixType& result)
{
    result = getElements<DataType, MatrixType>(input, 16);
}

// Platform specific aligned memory allocations
void* AlignedAlloc(size_t bytes, size_t alignment);
void AlignedFree(void* ptr);

#endif
