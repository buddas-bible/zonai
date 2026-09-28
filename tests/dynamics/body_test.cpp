#include <cassert>

#include "dynamics/body.h"

using namespace zonai;

int main()
{
    {
        Body body{};

        // 기본 Body는 정적이고 아직 Shape가 연결되지 않은 상태로 시작함.
        assert( body.bodyId == Body::NULL_INDEX );
        assert( body.generation == 0 );
        assert( body.nextFreeId == Body::NULL_INDEX );
        assert( body.type == BodyType::Static );
        assert( body.headContactKey == Body::NULL_INDEX );
        assert( body.contactCount == 0 );
        assert( body.headShapeId == Body::NULL_INDEX );
        assert( body.shapeCount == 0 );

        // 기본 transform은 원점 + identity rotation임.
        assert( body.transform.position.x == 0.0f );
        assert( body.transform.position.y == 0.0f );
        assert( body.transform.rotation.c == 1.0f );
        assert( body.transform.rotation.s == 0.0f );
    }

    {
        Body body{};

        body.type = BodyType::Dynamic;
        body.transform.position = { 3.0f, -2.0f };
        body.transform.rotation = rot2::FromRadians( 0.5f );
        body.headShapeId = 7;
        body.shapeCount = 3;

        assert( body.type == BodyType::Dynamic );
        assert( body.transform.position.x == 3.0f );
        assert( body.transform.position.y == -2.0f );
        assert( body.headShapeId == 7 );
        assert( body.shapeCount == 3 );
    }

    return 0;
}
