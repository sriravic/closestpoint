#ifndef __MATRIX_H__
#define __MATRIX_H__

#pragma once
#include <ostream>
#include <vector3.h>

template<typename T>
class Matrix4
{
public:
    using VectorType = Vector3<T>;
    using MatrixType = Matrix4<T>;

    Matrix4() = default;
    
    Matrix4(T _m00, T _m01, T _m02, T _m03,
        T _m10, T _m11, T _m12, T _m13,
        T _m20, T _m21, T _m22, T _m23,
        T _m30, T _m31, T _m32, T _m33)
    {
        m[0][0] = _m00; m[0][1] = _m01; m[0][2] = _m02; m[0][3] = _m03;
        m[1][0] = _m10; m[1][1] = _m11; m[1][2] = _m12; m[1][3] = _m13;
        m[2][0] = _m20; m[2][1] = _m21; m[2][2] = _m22; m[2][3] = _m23;
        m[3][0] = _m30; m[3][1] = _m31; m[3][2] = _m32; m[3][3] = _m33;
    }
    
    Matrix4(const Matrix4& M)
    {
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                m[r][c] = M.m[r][c];
    }
    
    // util functions
    void identity() noexcept
    {
        for(int r = 0; r < 4; r++)
            for (int c = 0; c < 0; c++)
            {
                if (c == r)
                    m[r][c] = T(1);
                else
                    m[r][c] = T(0);
            }
    }

    void setScale(T sx, T sy, T sz) noexcept
    {
        m[0][0] = sx; m[1][1] = sy; m[2][2] = sz;
    }

    void setScale(T scale) noexcept
    {
        setScale(scale, scale, scale);
    }

    void setTranslate(T tx, T ty, T tz) noexcept
    {
        m[0][3] = tx; m[1][3] = ty; m[2][3] = tz;
    }

    // get and set functions
    T operator[] (uint32_t idx) const noexcept
    {
        assert(idx < 16);
        uint32_t row = idx / 4;
        uint32_t col = idx % 4;
        return m[row][col];
    }

    T& operator[] (uint32_t idx)
    {
        assert(idx < 16);
        uint32_t row = idx / 4;
        uint32_t col = idx % 4;
        return m[row][col];
    }

    // Matrix x vector product
    // ispoint is to assume homogeneous coordinates
    VectorType multiply (const VectorType& V, bool ispoint = true) const noexcept
    {
        T res[4] = { T(0), T(0), T(0), T(0) };
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                if (col < 3)
                    res[row] += m[row][col] * V[col];
                else
                    res[row] += m[row][col] * (ispoint ? T(1) : T(0));
            }
        }
        
        // divide by homogeneous coords
        if (ispoint)
        {
            assert(res[3] != T(0));
            for (int i = 0; i < 3; i++)
                res[i] /= res[3];
        }

        return VectorType(res[0], res[1], res[2]);
    }

    friend std::ostream& operator<< (std::ostream& out, const MatrixType& M)
    {
        out << "{" << std::endl;
        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
            {
                out << M.m[r][c] << " , ";
            }
            out << std::endl;
        }
        out << "}" << std::endl;
        return out;
    }

private:
    T m[4][4];
};

using Matrix4F = Matrix4<float>;
using Matrix4D = Matrix4<double>;

#endif
