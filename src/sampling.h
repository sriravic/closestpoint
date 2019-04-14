#ifndef __SAMPLING_H__
#define __SAMPLING_H__

#pragma once
#include <common.h>
#include <geometry.h>

/// Sample a point on a unit sphere centered at [0,0,0]
template<typename T>
Vector3<T> sampleOnSphere(T xi1, float xi2)
{
    T z = T(1) - 2 * xi1;
    T r = std::sqrt(std::max(T(0), T(1) - z * z));
    T phi = T(2) * pi<T>() * xi2;
    return Vector3(r * std::cos(phi), r * std::sin(phi), z);
}

/// Sample a point on a sphere centered at [0,0,0] with radius
template<typename T>
Vector3<T> sampleOnSphere(T radius, float xi1, float xi2)
{
    return sampleOnSphere<T>(xi1, xi2) * radius;
}

/// Sample a point on a sphere at 'center' with radius
template<typename T>
Vector3<T> sampleOnSphere(const Vector3<T>& center, T radius,
    float xi1, float xi2)
{
    return center + sampleOnSphere<T>(xi1, xi2) * radius;
}

/// Sample a point inside a sphere uniformly
template<typename T>
Vector3<T> sampleWithinSphere(const Vector3<T>& center, T radius,
    float xi1, float xi2, float xi3)
{
    Vector3<T> pt = sampleOnSphere<T>(xi1, xi2) * radius;
    return center + pt * T(std::cbrt(xi3));
}

template<typename T>
Vector3<T> sampleWithinSphere(T radius, float xi1, float xi2, float xi3)
{
    Vector3<T> center(0);
    return sampleWithinSphere(center, radius, xi1, xi2, xi3);
}

template<typename T>
Vector3<T> sampleWithinSphere(const Sphere<T>& s, float xi1, float xi2, float xi3)
{
    return sampleWithinSphere(s.center, s.radius, xi1, xi2, xi3);
}

template<typename T>
Vector3<T> sampleWithinSphere(float xi1, float xi2, float xi3)
{
    return sampleWithinSphere(T(1), xi1, xi2, xi3);
}

/// Sample a point within a unit box centered at [0,0,0]
/// from [-0.5,-0.5,-0.5] to [0.5, 0.5, 0.5]
template<typename T>
Vector3<T> sampleBox(float xi1, float xi2, float xi3)
{
    using VectorType = Vector3<T>;
    return lerp<VectorType, VectorType>(VectorType(-0.5),
                                        VectorType(0.5),
                                        VectorType(T(xi1), T(xi2), T(xi3))
                                        );
}

template<typename T>
Vector3<T> sampleBox(const Box3<T>& box, float xi1, float xi2, float xi3)
{
    using VectorType = Vector3<T>;
    return lerp<VectorType, VectorType>(box.bmin, box.bmax,
                                        VectorType(T(xi1), T(xi2), T(xi3)));
}

#endif