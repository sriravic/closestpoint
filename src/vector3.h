#ifndef __VECTOR3_H__
#define __VECTOR3_H__

#pragma once
#include <cassert>
#include <cmath>
#include <ostream>

template<typename T>
class Vector3
{
public:

    Vector3()
        : x(T(0))
        , y(T(0))
        , z(T(0))
    {
    }

    Vector3(T val)
        : x(val)
        , y(val)
        , z(val)
    {
    }

    Vector3(T _x, T _y, T _z)
        : x(_x)
        , y(_y)
        , z(_z)
    {
    }

    Vector3(const Vector3& V)
        : x(V.x)
        , y(V.y)
        , z(V.z)
    {
    }

    T& operator[] (size_t index) noexcept
    {
        assert(index < 3);
        return (&x)[index];
    }

    T operator[] (size_t index) const noexcept
    {
        assert(index < 3);
        return (&x)[index];
    }

    // unary operators
    Vector3 operator- () const noexcept
    {
        return Vector3(-x, -y, -z);
    }

    // binary operators
    Vector3 operator+ (const Vector3& V) const noexcept
    {
        return Vector3(x + V.x, y + V.y, z + V.z);
    }

    Vector3 operator- (const Vector3& V) const noexcept
    {
        return Vector3(x - V.x, y - V.y, z - V.z);
    }

    Vector3 operator* (T scale) const noexcept
    {
        return Vector3(scale * x, scale * y, scale * z);
    }

    Vector3 operator* (const Vector3& scale) const noexcept
    {
        return Vector3(x * scale.x, y * scale.y, z * scale.z);
    }

    Vector3 operator/ (T scale) const noexcept(false)
    {
        assert(scale != T(0));
        T inv_scale = T(1) / scale;
        return *this * inv_scale;
    }

    Vector3 operator/ (const Vector3& scale) const noexcept
    {
        return Vector3(x / scale.x, y / scale.y, z / scale.z);
    }

    Vector3 friend operator* (T scale, const Vector3& V) noexcept
    {
        return Vector3(V.x * scale, V.y * scale, V.z * scale);
    }

    bool operator== (const Vector3& V) const noexcept
    {
        return x == V.x && y == V.y && z == V.z;
    }

    bool operator!= (const Vector3& V) const noexcept
    {
        return x != V.x || y != V.y || z != V.z;
    }
    
    // Returns true if vector can be normalized. If not returns 0
    bool normalize() noexcept
    {
        T len = length();
        assert(len != T(0));
        if (len == T(0))
            return false;
        *this /= len;
        return true;
    }

    // Returns zero vector for len == 0 vectors
    Vector3 normalized() const noexcept
    {
        T len = length();
        assert(len != T(0));
        if (len == T(0))
            return Vector3(T(0));
        return *this / len;
    }

    // unary assignment operators
    Vector3& operator+= (const Vector3& V) noexcept
    {
        x += V.x;
        y += V.y;
        z += V.z;
        return *this;
    }

    Vector3& operator-= (const Vector3& V) noexcept
    {
        x -= V.x;
        y -= V.y;
        z -= V.z;
        return *this;
    }

    Vector3& operator*= (T scale) noexcept
    {
        x *= scale;
        y *= scale;
        z *= scale;
        return *this;
    }

    Vector3& operator/= (T scale) noexcept(false)
    {
        assert(scale != T(0));
        T inv_scale = T(1) / scale;
        x *= inv_scale;
        y *= inv_scale;
        z *= inv_scale;
        return *this;
    }

    T dot(const Vector3& V) const noexcept
    {
        return x * V.x + y * V.y + z * V.z;
    }

    Vector3 cross(const Vector3& V) const noexcept
    {
        return Vector3(
            y * V.z - z * V.y,
            z * V.x - x * V.z,
            x * V.y - y * V.x);
    }

    T length2() const noexcept
    {
        return x * x + y * y + z * z;
    }

    T length() const noexcept
    {
        return std::sqrt(length2());
    }

    friend std::ostream& operator<< (std::ostream& out, const Vector3& V)
    {
        out << "[" << V.x << " , " << V.y << " , " << V.z << " ]";
        return out;
    }

    T minComponent() const noexcept
    {
        return std::min(x, std::min(y, z));
    }

    T maxComponent() const noexcept
    {
        return std::max(x, std::max(y, z));
    }

    int minDimension() const noexcept
    {
        return (x < y) ? ((x < z ? 0 : 2)) : ((y < z ? 1 : 2));
    }

    int maxDimension() const noexcept
    {
        return (x > y) ? ((x > z ? 0 : 2)) : ((y > z ? 1 : 2));
    }

    static Vector3 min(const Vector3& A, const Vector3& B)
    {
        Vector3 ret;
        for (int i = 0; i < 3; i++)
            ret[i] = std::min(A[i], B[i]);
        return ret;
    }

    static Vector3 max(const Vector3& A, const Vector3& B)
    {
        Vector3 ret;
        for (int i = 0; i < 3; i++)
            ret[i] = std::max(A[i], B[i]);
        return ret;
    }

private:
    T x, y, z;
};

using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;

#endif