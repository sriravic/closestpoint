#include <catch.hpp>
#include <geometry.h>
#include <random>
#include <sampling.h>

using namespace Catch::literals;

namespace
{
    static constexpr unsigned int theDefaultSeed = 0xCAFEF002;
    static std::default_random_engine theRandom(theDefaultSeed);
}

TEMPLATE_TEST_CASE("box_surfacearea", "[box3][template]", float, double)
{
    using BoxType = Box3<TestType>;
    using VectorType = Vector3<TestType>;
    BoxType test = { VectorType(-0.5), VectorType(0.5) };
    CHECK(test.area() == 6_a);
}

TEMPLATE_TEST_CASE("box_union", "[box3][template]", float, double)
{
    using BoxType = Box3<TestType>;
    using VectorType = Vector3<TestType>;
    BoxType b1 = { VectorType(-2), VectorType(0) };
    BoxType b2 = { VectorType(0), VectorType(2) };
    BoxType res = b1.unionOf(b2);
    CHECK(res.bmin == b1.bmin);
    CHECK(res.bmax == b2.bmax);
}

