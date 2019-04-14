#include <catch.hpp>
#include <matrix.h>

TEMPLATE_TEST_CASE("matrix_vector_product", "[matrix][template]", float, double)
{
    using MatrixType = Matrix4<TestType>;
    using VectorType = Vector3<TestType>;

    TestType sx = 1, sy = 2, sz = 3;
    TestType tx = 6, ty = 12, tz = 18;

    // without translation
    MatrixType testm(
        sx, 0, 0, 0,
        0, sy, 0, 0,
        0, 0, sz, 0,
        0, 0, 0, 1);

    VectorType testv(5, 8, -10);
    VectorType expv(5 * sx, 8 * sy, -10 * sz);
    
    auto&& result = testm.multiply(testv, false);
    CHECK(result == expv);


    // without ranslation
    MatrixType testmt(
        sx, 0, 0, tx,
        0, sy, 0, ty,
        0, 0, sz, tz,
        0, 0, 0, 1);

    VectorType testvt(5, 8, -10);
    VectorType expvt = VectorType(5 * sx + tx, 8 * sy + ty, -10 * sz + tz);
    auto&& resultt = testmt.multiply(testvt);

    CHECK(resultt == expvt);
}