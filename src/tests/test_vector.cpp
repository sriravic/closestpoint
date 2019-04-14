#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <random>
#include <vector3.h>

using namespace Catch::literals;

namespace
{
    static constexpr unsigned int theDefaultSeed = 0xCAFEF00D;
    static constexpr int numTestVectors = 20;
    static std::default_random_engine theRandom(theDefaultSeed);
}

// check if normalization of vectors is correct
TEMPLATE_TEST_CASE("vector_normalization", "[vector][template]", float, double)
{
    std::uniform_real_distribution<TestType> f01(TestType(-1), TestType(1));
    for (int i = 0; i < numTestVectors; i++)
    {
        Vector3<TestType> t(f01(theRandom), f01(theRandom), f01(theRandom));
        Vector3<TestType> tn = t.normalized();
        CHECK(tn.length() == 1_a);
    }
}

// checks if dot product of unit vector with itself is 1
TEMPLATE_TEST_CASE("vector_parallel", "[vector][template]", float, double)
{
    std::uniform_real_distribution<TestType> f01(TestType(-1), TestType(1));
    for (int i = 0; i < numTestVectors; i++)
    {
        Vector3<TestType> t(f01(theRandom), f01(theRandom), f01(theRandom));
        Vector3<TestType> tperp = t.cross(t);
        t.normalize();
        CHECK(t.length() == 1_a);
        CHECK(t.dot(t) == 1_a);
        CHECK(tperp.length() == 0_a);
    }
}

// checks access operator
TEMPLATE_TEST_CASE("vector_access_operator", "[vector][template]", float, double)
{
    std::uniform_real_distribution<TestType> f01(TestType(-1), TestType(1));
    TestType data[3] = { f01(theRandom), f01(theRandom), f01(theRandom) };
    Vector3<TestType> test(data[0], data[1], data[2]);
    for (int i = 0; i < 3; i++)
        CHECK(test[i] == data[i]);
}

// check min and max operators of vectors
TEMPLATE_TEST_CASE("vector_min_max", "[vector][template]", float, double)
{
    std::uniform_real_distribution<TestType> f01(TestType(-1), TestType(1));
    for (int t = 0; t < numTestVectors; t++)
    {
        Vector3<TestType> t1(f01(theRandom), f01(theRandom), f01(theRandom));
        Vector3<TestType> t2(f01(theRandom), f01(theRandom), f01(theRandom));
        Vector3<TestType> minT = Vector3<TestType>::min(t1, t2);
        Vector3<TestType> maxT = Vector3<TestType>::max(t1, t2);

        for (int i = 0; i < 3; i++)
        {
            CHECK(minT[i] <= t1[i]);
            CHECK(minT[i] <= t2[i]);
            CHECK(maxT[i] >= t1[i]);
            CHECK(maxT[i] >= t2[i]);
        }
    }
}

// Check min and max components
TEMPLATE_TEST_CASE("vector_min_comps", "[vector][template]", float, double)
{
    std::uniform_real_distribution<TestType> f01(TestType(-1), TestType(1));
    for (int i = 0; i < numTestVectors; i++)
    {
        auto findMin = [](const TestType (&val)[3], TestType& minval, int& minidx)
        {
            minidx = 0; minval = val[0];
            if (val[1] < minval) { minidx = 1; minval = val[1]; }
            if (val[2] < minval) { minidx = 2; minval = val[2]; }
        };

        TestType data[3] = { f01(theRandom), f01(theRandom), f01(theRandom) };
        TestType expVal;
        int expIdx;

        findMin(data, expVal, expIdx);
        Vector3 testVector(data[0], data[1], data[2]);
        CHECK(testVector.minDimension() == expIdx);
        CHECK(testVector.minComponent() == expVal);
    }
}

TEMPLATE_TEST_CASE("vector_max_comps", "[vector][template]", float, double)
{
    std::uniform_real_distribution<TestType> f01(TestType(-1), TestType(1));
    for (int i = 0; i < numTestVectors; i++)
    {
        auto findMax = [](const TestType(&val)[3], TestType& minval, int& minidx)
        {
            minidx = 0; minval = val[0];
            if (val[1] > minval) { minidx = 1; minval = val[1]; }
            if (val[2] > minval) { minidx = 2; minval = val[2]; }
        };

        TestType data[3] = { f01(theRandom), f01(theRandom), f01(theRandom) };
        TestType expVal;
        int expIdx;

        findMax(data, expVal, expIdx);
        Vector3 testVector(data[0], data[1], data[2]);
        CHECK(testVector.maxDimension() == expIdx);
        CHECK(testVector.maxComponent() == expVal);
    }
}