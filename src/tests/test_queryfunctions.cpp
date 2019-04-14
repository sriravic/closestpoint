#include <catch.hpp>
#include <queryfunctions.h>
using namespace Catch::literals;

TEMPLATE_TEST_CASE("triangle_point_query", "[query][template]", float, double)
{
    using DataType          = TestType;
    using VectorType        = Vector3<DataType>;
    using TriangleType      = Triangle<DataType>;
    using QueryResultType   = QueryResult<DataType, uint32_t>;

    // A canonical triangle
    VectorType v0(-1, -1, 0);
    VectorType v1(1, -1, 0);
    VectorType v2(0, 1, 0);
    TriangleType tri(v0, v1, v2);

    // pt along z-axis at 5 units
    VectorType testpt(0, 0, -5);
    VectorType expectedpt(0);   // origin is what we should project to

    QueryResultType result;
    computeTriangleClosestPoint<DataType, uint32_t>(tri, testpt, 0, result);
    CHECK(result.myClosestDistance == 25_a);
    CHECK(result.myClosestPoints[0] == expectedpt);
    result.clear();

    // test along edge
    testpt = VectorType(-0.5, -2, 0);
    expectedpt = VectorType(-0.5, -1, 0);
    computeTriangleClosestPoint<DataType, uint32_t>(tri, testpt, 0, result);
    INFO("Result : " << result.myClosestPoints[0] << " ; Exp : " << expectedpt);
    CHECK(result.myClosestPoints[0] == expectedpt);
}

TEMPLATE_TEST_CASE("manytriangle_query", "[query][template]", float, double)
{
    using T                 = TestType;
    using VectorType        = Vector3<T>;
    using TriangleType      = Triangle<T>;
    using QueryResultType   = QueryResult<T, uint32_t>;

    const int NUM_TRIS = 7;
    TriangleType test[NUM_TRIS] = {
                           {{T(-0.104629), T(-0.046658),  T(-0.0284528)},
                            {T(-0.103447), T(-0.0485516), T(-0.0297152)},
                            {T(-0.103688), T(-0.0491829), T(-0.0284528)}},
                           {{T(-0.103688), T(-0.0491829), T(-0.0284528)},
                            {T(-0.103447), T(-0.0485516), T(-0.0297152)},
                            {T(-0.10288),  T(-0.0504454), T(-0.0297152)}},
                           {{T(-0.103447), T(-0.0485516), T(-0.0297152)},
                            {T(-0.103447), T(-0.0479204), T(-0.0307727)},
                            {T(-0.102926), T(-0.0491829), T(-0.0309777)}},
                           {{T(-0.103447), T(-0.0485516), T(-0.0297152)},
                            {T(-0.102926), T(-0.0491829), T(-0.0309777)},
                            {T(-0.10288),  T(-0.0504454), T(-0.0297152)}},
                           {{T(-0.104629), T(-0.046658),  T(-0.0284528)},
                            {T(-0.1042),   T(-0.046658),  T(-0.0297152)},
                            {T(-0.103447), T(-0.0485516), T(-0.0297152)}},
                           {{T(-0.1042),   T(-0.046658),  T(-0.0297152)},
                            {T(-0.103943), T(-0.046658),  T(-0.0309777)},
                            {T(-0.103447), T(-0.0485516), T(-0.0297152)}},
                           {{T(-0.103943), T(-0.046658),  T(-0.0309777)},
                            {T(-0.103447), T(-0.0479204), T(-0.0307727)},
                            {T(-0.103447), T(-0.0485516), T(-0.0297152)}} };

    VectorType queryPt(T(-0.0570646), T(-0.0301497), T(-0.0156564));
    QueryResultType results[NUM_TRIS];

    for (int i = 0; i < NUM_TRIS; i++)
    {
        computeTriangleClosestPoint<T, uint32_t>
            (test[i], queryPt, i, results[i]);
    }

    // all these points should map to the same result
    for (int i = 0; i < NUM_TRIS - 1; i++)
        CHECK(results[i] == results[i + 1]);
}

TEMPLATE_TEST_CASE("cube_center", "[query][template]", float, double)
{
    using T = TestType;
    using VectorType = Vector3<T>;
    using TriangleType = Triangle<T>;
    using QueryResultType = QueryResult<T, uint32_t>;

    const int NUM_TRIS = 12;
    const int NUM_VERTS = 8;
    VectorType vertices[NUM_VERTS] = {
        {T(0), T(0), T(0)},
        {T(0), T(0), T(1)},
        {T(0), T(1), T(0)},
        {T(0), T(1), T(1)},
        {T(1), T(0), T(0)},
        {T(1), T(0), T(1)},
        {T(1), T(1), T(0)},
        {T(1), T(1), T(1)} };

    TriangleType tris[NUM_TRIS] = {    
        { vertices[0] , vertices[6] , vertices[4] },
        { vertices[0] , vertices[2] , vertices[6] }, 
        { vertices[0] , vertices[3] , vertices[2] }, 
        { vertices[0] , vertices[1] , vertices[3] }, 
        { vertices[2] , vertices[7] , vertices[6] }, 
        { vertices[2] , vertices[3] , vertices[7] }, 
        { vertices[4] , vertices[6] , vertices[7] }, 
        { vertices[4] , vertices[7] , vertices[5] }, 
        { vertices[0] , vertices[4] , vertices[5] }, 
        { vertices[0] , vertices[5] , vertices[1] }, 
        { vertices[1] , vertices[5] , vertices[7] }, 
        { vertices[1] , vertices[7] , vertices[3] }};

    const VectorType queryPt(T(0.5));
    QueryResultType result;

    for (int i = 0; i < NUM_TRIS; i++)
    {
        QueryResultType temp;
        computeTriangleClosestPoint<T, uint32_t>(tris[i], queryPt, i, temp);
        if (temp <= result)
        {
            if (temp < result)
                result.clear();
            result.appendResults(temp);
        }
    }

    /// 12 needed points
    CHECK(result.myClosestPoints.size() == 12);
    CHECK(result.myClosestDistance == 0.25_a); // (0.5)^2
}