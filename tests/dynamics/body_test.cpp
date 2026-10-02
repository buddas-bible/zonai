#include <cassert>
#include <limits>

#include "dynamics/body.h"
#include "dynamics/bodyDef.h"
#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"

using namespace zonai;

int main()
{

    {
        bodyDef definition{};

        assert( definition.type == bodyType::Static );
        assert( definition.transform.position.x == 0.0f );
        assert( definition.transform.position.y == 0.0f );
        assert( definition.linearVelocity.x == 0.0f );
        assert( definition.linearVelocity.y == 0.0f );
        assert( definition.angularVelocity == 0.0f );
        assert( definition.linearDamping == 0.0f );
        assert( definition.angularDamping == 0.0f );
        assert( definition.gravityScale == 1.0f );
        assert( definition.enableSleep );
        assert( definition.isAwake );
        assert( definition.sleepThreshold == 0.05f );
        assert( definition.safetyFactor == 0.5f );
        assert( !definition.isBullet );
        assert( !definition.allowFastRotation );
    }

    {
        body body{};

        // 기본 body는 정적이고 아직 shape가 연결되지 않은 상태로 시작함.
        assert( body.bodyId == body::NULL_INDEX );
        assert( body.generation == 0 );
        assert( body.nextFreeId == body::NULL_INDEX );
        assert( body.type == bodyType::Static );
        assert( body.mass == 0.0f );
        assert( body.inertia == 0.0f );
        assert( body.headContactKey == body::NULL_INDEX );
        assert( body.contactCount == 0 );
        assert( body.headShapeId == body::NULL_INDEX );
        assert( body.shapeCount == 0 );
        assert( body.safetyFactor == 0.5f );

    }

    {
        body body{};

        body.type = bodyType::Dynamic;
        body.headShapeId = 7;
        body.shapeCount = 3;

        assert( body.type == bodyType::Dynamic );
        assert( body.headShapeId == 7 );
        assert( body.shapeCount == 3 );
    }

    {
        bodySim bodySim{};

        // 기본 simulation slot은 비어 있고 transform은 identity임.
        assert( bodySim.bodyId == bodySim::NULL_INDEX );
        assert( bodySim.transform.position.x == 0.0f );
        assert( bodySim.transform.position.y == 0.0f );
        assert( bodySim.transform.rotation.c == 1.0f );
        assert( bodySim.transform.rotation.s == 0.0f );
        assert( bodySim.localCenter.x == 0.0f );
        assert( bodySim.localCenter.y == 0.0f );
        assert( bodySim.center.x == 0.0f );
        assert( bodySim.center.y == 0.0f );
        assert( bodySim.force.x == 0.0f );
        assert( bodySim.force.y == 0.0f );
        assert( bodySim.torque == 0.0f );
        assert( bodySim.invMass == 0.0f );
        assert( bodySim.invInertia == 0.0f );
        assert( bodySim.linearDamping == 0.0f );
        assert( bodySim.angularDamping == 0.0f );
        assert( bodySim.gravityScale == 1.0f );
        assert( !bodySim.isBullet );
        assert( !bodySim.isFast );
        assert( !bodySim.hadTimeOfImpact );
        assert( !bodySim.allowFastRotation );
        assert( bodySim.minExtent == std::numeric_limits<float>::max() );
        assert( bodySim.maxExtent == 0.0f );

        bodySim.bodyId = 7;
        bodySim.transform.position = { 3.0f, -2.0f };
        bodySim.transform.rotation = rot2::FromRadians( 0.5f );

        assert( bodySim.bodyId == 7 );
        assert( bodySim.transform.position.x == 3.0f );
        assert( bodySim.transform.position.y == -2.0f );
    }

    {
        bodyState state{};

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
