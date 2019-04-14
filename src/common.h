#ifndef __COMMON_H__
#define __COMMON_H__

#pragma once
#include <cmath>
#include <cassert>
#include <math.h>

template<typename T>
constexpr bool isPositive(T val) { return val >= T(0); }

template<typename T>
constexpr bool isNegative(T val) { return val < T(0); }

template<typename T>
constexpr int sign(T val) { return (val < T(0) ? -1 : 1); }

template<typename T>
constexpr T pi() { return T(3.1415926535897932384626433832795); }

/// Templated lerp type across scalar, vector types
/// on a component by component basis
template<typename ValueType, typename ScaleType>
constexpr ValueType
lerp(const ValueType& val1, const ValueType& val2, ScaleType factor)
{
    return (ScaleType(1) - factor) * val1 + factor * val2;
}

/// Convenient function to check if we have nans/infs
template<typename T>
bool
isValid(const T& val)
{
    return !std::isnan(val) && !std::isinf(val);
}

/// convenient class to make a non-copyable class
class NonCopyable
{
public:
    NonCopyable() = default;
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable(NonCopyable&&) = delete;
    NonCopyable& operator= (const NonCopyable&) = delete;
    NonCopyable& operator= (NonCopyable&&) = delete;
};

#endif
