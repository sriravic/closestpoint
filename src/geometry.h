#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <algorithm>
#include <common.h>
#include <limits>
#include <ostream>
#include <vector3.h>

/// Axis aligned bounding box
template<typename T>
struct Box3
{
    using VectorType = Vector3<T>;

    Box3()
        : bmin(std::numeric_limits<T>::max())
        , bmax(-std::numeric_limits<T>::max())
    {
    }

    Box3(const VectorType& _min, const VectorType& _max)
        : bmin(_min)
        , bmax(_max)
    {
    }

    Box3(const Box3& box)
        : bmin(box.bmin)
        , bmax(box.bmax)
    {
    }

    bool isvalid() const noexcept
    {
        for (int i = 0; i < 3; i++)
            if (bmin[i] > bmax[i])
                return false;
        return true;
    }

    VectorType diagonal() const noexcept
    {
        assert(isvalid());
        return bmax - bmin;
    }

    VectorType center() const
    {
        return (bmax + bmin) * T(0.5);
    }

    T area() const noexcept
    {
        // 2.0 * ( dx * dy + dy * dz + dz * dx)
        auto&& diag = diagonal();
        return T(2) * (
            diag[0] * diag[1] +
            diag[1] * diag[2] +
            diag[2] * diag[0]);
    }

    T volume() const noexcept
    {
        auto&& diag = diagonal();
        return diag[0] * diag[1] * diag[2];
    }

    int maxDimension() const noexcept
    {
        assert(isvalid());
        return diagonal().maxDimension();
    }

    Box3 unionOf(const Box3& box) const noexcept
    {
        assert(isvalid());
        assert(box.isvalid());

        Box3 ret = {
            VectorType::min(bmin, box.bmin),
            VectorType::max(bmax, box.bmax)
        };
        assert(ret.isvalid());
        return ret;
    }
    
    Box3 intersectOf(const Box3& box) const noexcept
    {
        assert(isvalid());
        assert(box.isvalid());
        Box3 ret =
        {
            VectorType::max(bmin, box.bmin),
            VectorType::min(bmax, box.bmax)
        };
        assert(ret.isvalid());
        return ret;
    }

    Box3& grow(const Box3& box) noexcept
    {
        bmin = VectorType::min(bmin, box.bmin);
        bmax = VectorType::max(bmax, box.bmax);
        assert(isvalid());
        return *this;
    }

    Box3& grow(const VectorType& pt)
    {
        bmin = VectorType::min(bmin, pt);
        bmax = VectorType::max(bmax, pt);
        assert(isvalid());
        return *this;
    }

    void split(T splitPos, int dim, Box3& left, Box3& right) const noexcept
    {
        assert(isvalid());
        left = *this; 
        right = *this;
        left.bmax[dim] = splitPos;
        right.bmin[dim] = splitPos;
        assert(left.isvalid());
        assert(right.isvalid());
    }

    bool contains(const VectorType& pt) const noexcept
    {
        assert(isvalid());
        for (int i = 0; i < 3; i++)
            if (pt[i] < bmin[i] || pt[i] > bmax[i])
                return false;
        return true;
    }

    friend std::ostream& operator<< (std::ostream& out, const Box3& box)
    {
        out << "{ " << box.bmin << " , " << box.bmax << " } ";
        return out;
    }

    // min and max bounds
    VectorType bmin, bmax;
};

template<typename T>
struct Triangle
{
    using VectorType = Vector3<T>;
    using BoxType = Box3<T>;

    Triangle() = default;
    
    Triangle(const VectorType& v0, const VectorType& v1, const VectorType& v2)
    {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }
    
    Triangle(const Triangle& triangle)
    {
        v[0] = triangle.v[0];
        v[1] = triangle.v[1];
        v[2] = triangle.v[2];
    }

    T area() const noexcept
    {
        // area = 0.5 * ||(v2-v0) x (v1 - v0)||
        VectorType cp = (v[2] - v[0]).cross(v[1] - v[0]);
        return T(0.5) * cp.length();
    }

    BoxType bounds() const noexcept
    {
        BoxType ret;
        for (int i = 0; i < 3; i++)
            ret.grow(v[i]);
        assert(ret.isvalid());
        return ret;
    }

    VectorType normal() const noexcept
    {
        return ((v[2] - v[0]).cross(v[1] - v[0])).normalized();
    }

    friend std::ostream& operator<< (std::ostream& out, const Triangle& tri)
    {
        out << " { " << tri.v[0] << " , " << tri.v[1] << " , " << tri.v[2] << " } ";
        return out;
    }

    // vertex data
    VectorType v[3];
};

template<typename T>
struct Plane
{
    using VectorType = Vector3<T>;

    Plane()
        : d(T(0))
    {
    }

    Plane(const VectorType& normal, T dist)
        : normal(normal)
        , d(dist)
    {
    }

    // Construct a plane from a point on plane and normal
    Plane(const VectorType& normal, const VectorType& pos)
        : normal(normal)
    {
        // n.x + d = 0 ===> d = -n.x
        d = -normal.dot(pos);
    }

    Plane(const Plane& P)
        : normal(P.normal)
        , d(P.d)
    {
    }

    friend std::ostream& operator<< (std::ostream& out, const Plane& P)
    {
        out << " { " << P.normal << " , " << P.d << " } ";
        return out;
    }

    // Normal representation: n.x + d = 0
    VectorType  normal;
    T           d;
};

template<typename T>
struct Sphere
{
    using VectorType    = Vector3<T>;
    using BoxType       = Box3<T>;
    using SphereType    = Sphere<T>;

    Sphere()
        : radius(T(0))
    {
    }

    Sphere(const VectorType& center, T rad)
        : center(center)
        , radius(rad)
    {
    }

    Sphere(const Sphere& S)
        : center(S.center)
        , radius(S.radius)
    {
    }

    T area() const noexcept
    {
        return T(4) * pi<T>() * radius * radius;
    }

    T volume() const noexcept
    {
        return T(4) / T(3) * pi<T>() * radius * radius * radius;
    }

    BoxType bounds() const noexcept
    {
        return BoxType(
            center - VectorType(radius),
            center + VectorType(radius));
    }

    bool contains(const VectorType& pt) const noexcept
    {
        return (pt - center).length2() <= radius * radius;
    }

    friend std::ostream& operator<< (std::ostream& out, const Sphere& S)
    {
        out << " { " << S.center << " , " << S.radius << " } ";
        return out;
    }

    VectorType  center;
    T           radius;
};

template<
    typename T,
    typename SphereType = Sphere<T>,
    typename BoxType = Box3<T>>
static SphereType
getBoundingSphere(const BoxType& box)
{
    assert(box.isvalid());
    SphereType ret;
    ret.center = box.center();
    ret.radius = box.diagonal().length() * T(0.5);
    return ret;
}

using Box3F         = Box3<float>;
using Box3D         = Box3<double>;
using TriangleF     = Triangle<float>;
using TriangleD     = Triangle<double>;
using PlaneF        = Plane<float>;
using PlaneD        = Plane<double>;

#endif
