#include <cassert>

#include "dynamics/body.h"
#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"

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
        assert( body.mass == 0.0f );
        assert( body.inertia == 0.0f );
        assert( body.headContactKey == Body::NULL_INDEX );
        assert( body.contactCount == 0 );
        assert( body.headShapeId == Body::NULL_INDEX );
        assert( body.shapeCount == 0 );

    }

    {
        Body body{};

        body.type = BodyType::Dynamic;
        body.headShapeId = 7;
        body.shapeCount = 3;

        assert( body.type == BodyType::Dynamic );
        assert( body.headShapeId == 7 );
        assert( body.shapeCount == 3 );
    }

    {
        BodySim bodySim{};

        // 기본 simulation slot은 비어 있고 transform은 identity임.
        assert( bodySim.bodyId == BodySim::NULL_INDEX );
        assert( bodySim.transform.position.x == 0.0f );
        assert( bodySim.transform.position.y == 0.0f );
        assert( bodySim.transform.rotation.c == 1.0f );
        assert( bodySim.transform.rotation.s == 0.0f );
        assert( bodySim.localCenter.x == 0.0f );
        assert( bodySim.localCenter.y == 0.0f );
        assert( bodySim.center.x == 0.0f );
        assert( bodySim.center.y == 0.0f );
        assert( bodySim.invMass == 0.0f );
        assert( bodySim.invInertia == 0.0f );

        bodySim.bodyId = 7;
        bodySim.transform.position = { 3.0f, -2.0f };
        bodySim.transform.rotation = rot2::FromRadians( 0.5f );

        assert( bodySim.bodyId == 7 );
        assert( bodySim.transform.position.x == 3.0f );
        assert( bodySim.transform.position.y == -2.0f );
    }

    {
        BodyState state{};

        assert( state.linearVelocity.x == 0.0f );
        assert( state.linearVelocity.y == 0.0f );
        assert( state.angularVelocity == 0.0f );

        state.linearVelocity = { 4.0f, -3.0f };
        state.angularVelocity = 2.5f;

        assert( state.linearVelocity.x == 4.0f );
        assert( state.linearVelocity.y == -3.0f );
        assert( state.angularVelocity == 2.5f );
    }

    return 0;
}
