#include "collision/narrowphase/collide.h"

namespace zonai
{

localManifold2 CollidePolygonCapsule(
    const polygon2& polygon,
    const capsule2& capsule, const transform2& capsuleTransform )
{
    const polygon2 capsulePolygon = MakeCapsule(
        capsule.center1,
        capsule.center2,
        capsule.radius
    );

    return CollidePolygons(
        polygon,
        capsulePolygon,
        capsuleTransform
    );
}

} // namespace zonai
