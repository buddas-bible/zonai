#include <cassert>

#include "collision/broadphase/broadPhase.h"
#include "dynamics/bodyType.h"

using namespace zonai;

int main()
{
    BroadPhase broadPhase{};

    DynamicTree& staticTree = broadPhase.GetTree( BodyType::Static );
    DynamicTree& kinematicTree = broadPhase.GetTree( BodyType::Kinematic );
    DynamicTree& dynamicTree = broadPhase.GetTree( BodyType::Dynamic );

    assert( &staticTree != &kinematicTree );
    assert( &staticTree != &dynamicTree );
    assert( &kinematicTree != &dynamicTree );

    const aabb2 box{
        { 0.0f, 0.0f },
        { 1.0f, 1.0f }
    };

    staticTree.CreateProxy( box, 1 );

    assert( staticTree.GetProxyCount() == 1 );
    assert( kinematicTree.GetProxyCount() == 0 );
    assert( dynamicTree.GetProxyCount() == 0 );

    kinematicTree.CreateProxy( box, 2 );

    assert( staticTree.GetProxyCount() == 1 );
    assert( kinematicTree.GetProxyCount() == 1 );
    assert( dynamicTree.GetProxyCount() == 0 );

    dynamicTree.CreateProxy( box, 3 );

    assert( staticTree.GetProxyCount() == 1 );
    assert( kinematicTree.GetProxyCount() == 1 );
    assert( dynamicTree.GetProxyCount() == 1 );

    return 0;
}
