#include <catch.hpp>
#include <sampling.h>
#include <random>

namespace
{
    static constexpr unsigned int theSeed = 0xCAFEF00B;
    static std::default_random_engine theRand(theSeed);
    static const int theNumTests = 20;
}

TEMPLATE_TEST_CASE("sampling_unitbox", "[sampling][template]", float, double)
{
    using DataType = TestType;
    using VectorType = Vector3<DataType>;
    using BoxType = Box3<DataType>;

    std::uniform_real_distribution<float> f01;
    BoxType box = { VectorType(-0.5), VectorType(0.5) };
    
    for (int t = 0; t < theNumTests; t++)
    {
        auto&& res = sampleBox<DataType>(f01(theRand),
                                         f01(theRand),
                                         f01(theRand));
        CHECK(box.contains(res));
    }
}

TEMPLATE_TEST_CASE("sampling_box", "[sampling][box][template]", float, double)
{
    using DataType = TestType;
    using VectorType = Vector3<DataType>;
    using BoxType = Box3<DataType>;

    std::uniform_real_distribution<float> f01;
    BoxType box = { VectorType(1, 1, 1), VectorType(5, 4, 20) };

    for (int t = 0; t < theNumTests; t++)
    {
        auto&& res = sampleBox<DataType>(box, f01(theRand),
                                              f01(theRand),
                                              f01(theRand));
        CHECK(box.contains(res));
    }
}

TEMPLATE_TEST_CASE("sampling_sphere", "[sampling][sphere][template]", float, double)
{
    using DataType = TestType;
    using VectorType = Vector3<DataType>;
    using SphereType = Sphere<DataType>;

    std::uniform_real_distribution<float> f01;
    for (int t = 0; t < theNumTests; t++)
    {
        // generate different radii
        auto&& radius = f01(theRand);
        SphereType testSphere(VectorType(0), DataType(radius));

        auto&& samplePt = sampleWithinSphere(testSphere, f01(theRand),
                                                         f01(theRand),
                                                         f01(theRand));

        CHECK(testSphere.contains(samplePt));
    }
}