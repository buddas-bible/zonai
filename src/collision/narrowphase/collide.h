#pragma once

#include "geometry/segment2.h"
#include "geometry/circle2.h"
#include "geometry/capsule2.h"
#include "geometry/polygon2.h"

#include "math/transform2.h"
#include "collision/narrowphase/localManifold2.h"

namespace zonai
{

/*
segment-circle
segment-capsule
segment-polygon
*/
localManifold2 CollideSegmentCircle(
    const segment2& segment,
    const circle2& circle, const transform2& circleTransform);

localManifold2 CollideCapsuleSegment(
    const capsule2& capsule, 
    const segment2& segment, const transform2& segmentTransform);

localManifold2 CollidePolygonSegment(
    const polygon2& polygon,
    const segment2& segment, const transform2& segmentTransform);


/*
circle-circle
circle-capsule
circle-polygon
*/
localManifold2 CollideCircles(
    const circle2& a,
    const circle2& b, const transform2& transformB);

localManifold2 CollideCapsuleCircle(
    const capsule2& capsule,
    const circle2& circle, const transform2& circleTransform);

localManifold2 CollidePolygonCircle(
    const polygon2& polygon,
    const circle2& circle, const transform2& circleTransform);

/*
capsule-capsule
capsule-polygon
*/
localManifold2 CollideCapsules(
    const capsule2& a,
    const capsule2& b, const transform2& transformB);

localManifold2 CollidePolygonCapsule(
    const polygon2& polygon, 
    const capsule2& capsule, const transform2& capsuleTransform);


/*
polygon-polygon
*/
localManifold2 CollidePolygons(
    const polygon2& a,
    const polygon2& b, const transform2& transformB);

} // namespace zonai