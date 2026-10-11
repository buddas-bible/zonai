#include "rigidBodyDemo.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string_view>

using namespace zonai;
using namespace zonai::sandbox;

int main()
{
    int failures = 0;
    const auto check = [&]( bool condition, const char* message )
    {
        if( condition ) return;

        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    };

    const auto entries = getDemoEntries();
    const auto terrainEntry = std::find_if( entries.begin(), entries.end(),
        []( const demoEntry& entry )
        {
            return std::string_view{ entry.name } == "체인 지형";
        } );

    check( terrainEntry != entries.end(), "terrain chain demo is registered" );
    if( terrainEntry == entries.end() ) return EXIT_FAILURE;

    check( std::string_view{ terrainEntry->category } == "지형", "terrain chain demo uses the terrain category" );

    rigidBodyDemo terrain{ terrainEntry->kind };
    world& simulation = terrain.getWorld();

    check( simulation.GetChainCount() == 1, "terrain scene owns one parent chain" );

    chainId parentChain{};
    std::size_t chainChildCount = 0;

    for( const visualShape& visual : terrain.getShapes() )
    {
        if( !simulation.IsValid( visual.shapeHandle ) ) continue;

        const chainId parent = simulation.GetShapeParentChain( visual.shapeHandle );
        if( !simulation.IsValid( parent ) ) continue;

        if( !simulation.IsValid( parentChain ) )
        {
            parentChain = parent;
        }
        else
        {
            check( parent.index1 == parentChain.index1 && parent.generation == parentChain.generation,
                "all terrain child shapes belong to one parent chain" );
        }

        ++chainChildCount;
    }

    check( simulation.IsValid( parentChain ), "terrain child shapes expose their parent chain" );
    if( simulation.IsValid( parentChain ) )
    {
        check( simulation.GetChainSegmentCount( parentChain ) == 8, "terrain parent chain owns eight segments" );
    }
    check( chainChildCount == 8, "terrain demo exposes all eight chain segments for debug drawing" );

    bool sawContact = false;
    for( int i = 0; i < 240; ++i )
    {
        terrain.step( 1.0f / 60.0f, 4 );
        sawContact = sawContact || !terrain.getContacts().empty();
    }

    check( sawContact, "dynamic probes make real contact with the chain terrain" );

    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
