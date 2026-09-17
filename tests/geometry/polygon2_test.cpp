#include <cassert>
#include <cmath>

#include "geometry/polygon2.h"

using namespace zonai;

bool NearlyEqual(
    float a,
    float b,
    float epsilon = 1e-5f )
{
    return std::fabs( a - b ) <= epsilon;
}

bool NearlyEqual(
    const vec2& a,
    const vec2& b,
    float epsilon = 1e-5f )
{
    return NearlyEqual( a.x, b.x, epsilon ) &&
        NearlyEqual( a.y, b.y, epsilon );
}

float SignedArea2( const polygon2& polygon )
{
    float area2 = 0.0f;

    for( std::size_t i = 0; i < polygon.vertexCount; ++i )
    {
        const std::size_t next = ( i + 1 ) % polygon.vertexCount;
        area2 += Cross( polygon.vertices[i], polygon.vertices[next] );
    }

    return area2;
}

int main()
{
    {
        // MakeBox는 완성된 CCW polygon을 반환해야 한다.
        const polygon2 box = MakeBox( { 2.0f, 1.0f } );

        assert( box.vertexCount == 4 );
        assert( NearlyEqual( box.centroid, { 0.0f, 0.0f } ) );

        assert( NearlyEqual( box.normals[0], { 0.0f, -1.0f } ) );
        assert( NearlyEqual( box.normals[1], { 1.0f, 0.0f } ) );
        assert( NearlyEqual( box.normals[2], { 0.0f, 1.0f } ) );
        assert( NearlyEqual( box.normals[3], { -1.0f, 0.0f } ) );
    }

    {
        // capsule은 2개의 core vertex와 서로 반대인 normal을 가져야 한다.
        const polygon2 capsule = MakeCapsule(
            { -1.0f, 0.0f },
            { 1.0f, 0.0f },
            0.5f
        );

        assert( capsule.vertexCount == 2 );
        assert( NearlyEqual( capsule.vertices[0], { -1.0f, 0.0f } ) );
        assert( NearlyEqual( capsule.vertices[1], { 1.0f, 0.0f } ) );
        assert( NearlyEqual( capsule.centroid, { 0.0f, 0.0f } ) );
        assert( NearlyEqual( capsule.normals[0], { 0.0f, -1.0f } ) );
        assert( NearlyEqual( capsule.normals[1], { 0.0f, 1.0f } ) );
        assert( NearlyEqual( capsule.radius, 0.5f ) );
    }

    {
        // CCW triangle의 centroid와 outward normal을 계산해야 한다.
        const vec2 vertices[] =
        {
            { 0.0f, 0.0f },
            { 2.0f, 0.0f },
            { 0.0f, 2.0f }
        };

        const polygon2 polygon = MakePolygon( vertices );

        assert( polygon.vertexCount == 3 );
        assert( SignedArea2( polygon ) > 0.0f );
        assert( NearlyEqual( polygon.centroid, { 2.0f / 3.0f, 2.0f / 3.0f } ) );

        assert( NearlyEqual( polygon.normals[0], { 0.0f, -1.0f } ) );
        assert( NearlyEqual( polygon.normals[2], { -1.0f, 0.0f } ) );
    }

    {
        // CW 입력은 CCW winding으로 교정해야 한다.
        const vec2 vertices[] =
        {
            { 0.0f, 0.0f },
            { 0.0f, 2.0f },
            { 2.0f, 0.0f }
        };

        const polygon2 polygon = MakePolygon( vertices );

        assert( polygon.vertexCount == 3 );
        assert( SignedArea2( polygon ) > 0.0f );
        assert( NearlyEqual( polygon.centroid, { 2.0f / 3.0f, 2.0f / 3.0f } ) );
    }

    {
        // polygon radius는 AABB에 포함되어야 한다.
        polygon2 box = MakeBox( { 2.0f, 1.0f } );
        box.radius = 0.5f;

        const aabb2 bounds = ComputeAABB( box );

        assert( NearlyEqual( bounds.min, { -2.5f, -1.5f } ) );
        assert( NearlyEqual( bounds.max, { 2.5f, 1.5f } ) );
    }

    return 0;
}
