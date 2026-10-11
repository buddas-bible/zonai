#include "collision/massData2.h"

#include <cassert>
#include <cmath>

namespace zonai
{

massData2 ComputeMass( const chainSegment2&, float density )
{
    assert( std::isfinite( density ) );
    assert( density >= 0.0f );

    // Chain edge도 ordinary segment와 동일하게 2D 면적이 없고 ghost vertex는 adjacency metadata뿐임.
    return {};
}

} // namespace zonai
