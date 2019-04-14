#include <catch.hpp>
#include <util.h>

using namespace Catch::literals;

TEMPLATE_TEST_CASE("util_parse_vector", "[util][template]", float, double)
{
    constexpr static int NUMTESTS = 3;
    static const std::string teststrs[NUMTESTS] = {
        std::string("[10.233, -02.233, 9.9923]"),
        std::string("[0, -233.23, 993.33]"),
        std::string("[5.66, , 233.3]")
    };

    using T = TestType;
    static const Vector3<TestType> expected[3] =
    {
        {T(10.233), T(-2.233),  T(9.9923)},
        {T(0),      T(-233.23), T(993.33)},
        {T(5.66),   T(0),       T(233.3)}
    };

    for (int test = 0; test < NUMTESTS; test++)
    {
        auto&& res = getVector<T, Vector3<T>>(teststrs[test]);
        for (int i = 0; i < 3; i++)
        {
            /// string parsing might not be entirely accurate
            auto&& margin_res = Catch::Detail::Approx(res[i]);
            CHECK(expected[test][i] == margin_res);
        }
    }
}

TEMPLATE_TEST_CASE("util_parse_matrix", "[util][template]", float, double)
{
    static const std::string test("[10.233, -02.233, 9.9923,  5.33,\
                                         0, -233.23, 993.33, -3.12,\
                                      5.66,      23, 233.33, 09.99,\
                                         0,       0,      0,     1]");

    using T = TestType;
    static const Matrix4<TestType> expected(
        T(10.233), T(-2.233),  T(9.9923), T(5.33),
        T(0),      T(-233.23), T(993.33), T(-3.12),
        T(5.66),   T(23),      T(233.33), T(9.99),
        T(0),      T(0),       T(0),      T(1));

    Matrix4<TestType> res;
    getMatrix<T, Matrix4<T>>(test, res);
    for (int i = 0; i < 16; i++)
    {
        /// string parsing might not be entirely accurate
        auto&& margin_res = Catch::Detail::Approx(res[i]);
        CHECK(expected[i] == margin_res);
    }
}