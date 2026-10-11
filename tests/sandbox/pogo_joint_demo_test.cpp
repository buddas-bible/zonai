#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string_view>

#include "demo.h"
#include "rigidBodyDemo.h"

using namespace zonai;
using namespace zonai::sandbox;

namespace
{

void check( bool ok, const char* message )
{
    if( !ok )
    {
        std::fprintf( stderr, "%s\n", message );
        std::exit( EXIT_FAILURE );
    }
}

bool near( float a, float b )
{
    return std::abs( a - b ) < 0.0001f;
}

}

int main()
{
    // Scene registration부터 preset/runtime response까지 하나의 학습용 Pogo playground 계약으로 확인함.
    check( getDemoEntries().size() == 13, "Pogo Joint demo is independently selectable" );
    check( std::string_view{ getDemoEntry( demoKind::pogoJointPlayground ).category } == "조인트", "Pogo Joint demo stays in joint category" );

    rigidBodyDemo pogo{ demoKind::pogoJointPlayground };
    check( pogo.getWorld().GetBodyCount() == 2 && pogo.getWorld().getJointCount() == 1 && pogo.getShapes().size() == 2, "Pogo scene has ground and character bodies" );
    check( pogo.getWorld().IsValid( pogo.getPogoJoint() ), "Pogo scene owns a valid Pogo Joint" );

    pogo.applyPogoJointPreset( pogoJointDemoPreset::soft );
    auto data = pogo.getWorld().getPogoJointData( pogo.getPogoJoint() );
    check( data.hertz > 0.0f && data.hertz < 4.0f, "Pogo soft preset uses a low spring frequency" );
    check( data.maxTensionForce > 0.0f && data.maxCompressionForce > data.maxTensionForce, "Pogo soft preset supports both tension and compression" );

    pogo.applyPogoJointPreset( pogoJointDemoPreset::stiff );
    data = pogo.getWorld().getPogoJointData( pogo.getPogoJoint() );
    check( data.hertz >= 6.0f && data.maxCompressionForce >= 300.0f, "Pogo stiff preset raises spring response and force budget" );

    pogo.applyPogoJointPreset( pogoJointDemoPreset::compressionOnly );
    data = pogo.getWorld().getPogoJointData( pogo.getPogoJoint() );
    check( near( data.maxTensionForce, 0.0f ) && data.maxCompressionForce > 0.0f, "Pogo compression-only preset cannot pull the character down" );

    pogo.applyPogoJointPreset( pogoJointDemoPreset::asymmetric );
    data = pogo.getWorld().getPogoJointData( pogo.getPogoJoint() );
    check( data.maxCompressionForce > data.maxTensionForce && data.maxTensionForce > 0.0f, "Pogo asymmetric preset demonstrates separate tension/compression budgets" );

    pogo.setPogoJointSettings( 0.9f, 5.0f, 0.8f, 25.0f, 220.0f );
    data = pogo.getWorld().getPogoJointData( pogo.getPogoJoint() );
    check( near( data.restLength, 0.9f ) && near( data.hertz, 5.0f ) && near( data.dampingRatio, 0.8f ), "Pogo inspector applies spring settings" );
    check( near( data.maxTensionForce, 25.0f ) && near( data.maxCompressionForce, 220.0f ), "Pogo inspector applies asymmetric force limits" );

    const bodyId driven = data.bodyB;
    pogo.applyPogoJointPreset( pogoJointDemoPreset::compressionOnly );
    pogo.step( 1.0f / 60.0f, 1 );
    check( IsFinite( pogo.getWorld().GetBodyLinearVelocity( driven ) ), "Pogo demo produces a finite driven-body response" );
    check( IsFinite( pogo.getWorld().getPogoJointData( pogo.getPogoJoint() ).force ), "Pogo demo exposes a finite reaction force" );

    return EXIT_SUCCESS;
}
