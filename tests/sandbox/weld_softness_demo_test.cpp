#include <cstdio>
#include <cstdlib>

#include "demo.h"
#include "rigidBodyDemo.h"

using namespace zonai;
using namespace zonai::sandbox;

namespace
{

void check( bool condition, const char* message )
{
    if( !condition )
    {
        std::fprintf( stderr, "%s\n", message );
        std::exit( EXIT_FAILURE );
    }
}

}

int main()
{
    rigidBodyDemo weld{ demoKind::weldPair };
    const jointId joint = weld.getWeldJoint();
    check( weld.getWorld().IsValid( joint ), "Weld softness demo owns a valid joint" );

    auto data = weld.getWorld().getWeldJointData( joint );
    check( data.linearHertz == 0.0f && data.angularHertz == 0.0f, "Weld demo starts as a rigid Weld" );

    weld.setWeldLinearSettings( 5.0f, 1.2f );
    weld.setWeldAngularSettings( 6.0f, 0.9f );
    data = weld.getWorld().getWeldJointData( joint );
    check( data.linearHertz == 5.0f && data.linearDampingRatio == 1.2f, "Weld inspector applies arbitrary linear tuning" );
    check( data.angularHertz == 6.0f && data.angularDampingRatio == 0.9f, "Weld inspector applies arbitrary angular tuning" );

    weld.applyWeldPreset( weldDemoPreset::softLinear );
    data = weld.getWorld().getWeldJointData( joint );
    check( data.linearHertz > 0.0f && data.angularHertz == 0.0f, "Soft Linear preset isolates translation softness" );

    weld.applyWeldPreset( weldDemoPreset::softAngular );
    data = weld.getWorld().getWeldJointData( joint );
    check( data.linearHertz == 0.0f && data.angularHertz > 0.0f, "Soft Angular preset isolates rotation softness" );

    weld.applyWeldPreset( weldDemoPreset::softBoth );
    data = weld.getWorld().getWeldJointData( joint );
    check( data.linearHertz > 0.0f && data.angularHertz > 0.0f, "Soft Both preset enables both Weld spring channels" );

    weld.applyWeldPreset( weldDemoPreset::rigid );
    data = weld.getWorld().getWeldJointData( joint );
    check( data.linearHertz == 0.0f && data.angularHertz == 0.0f, "Rigid preset restores hard Weld behavior" );

    return EXIT_SUCCESS;
}
