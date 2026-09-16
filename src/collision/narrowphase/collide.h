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
* circle-circle
* segment-circle
* capsule-circle
* polygon-circle
*/
localManifold2 CollideCircles(
    const circle2& a,
    const circle2& b, const transform2& transformB);

localManifold2 CollideSegmentCircle(
    const segment2& segment,
    const circle2& circle, const transform2& circleTransform );

localManifold2 CollideCapsuleCircle(
    const capsule2& capsule,
    const circle2& circle, const transform2& circleTransform );

localManifold2 CollidePolygonCircle(
    const polygon2& polygon,
    const circle2& circle, const transform2& circleTransform );

/*
* capsule-capsule
* segment-capsule
* polygon-capsule
*/
localManifold2 CollideCapsules(
    const capsule2& a,
    const capsule2& b, const transform2& transformB );

localManifold2 CollideSegmentCapsule(
    const segment2& segment,
    const capsule2& capsuleB, const transform2& capsuleBTransform );

localManifold2 CollidePolygonCapsule(
    const polygon2& polygon,
    const capsule2& capsule, const transform2& capsuleTransform );

/*
* polygon-polygon
* polygon-segment
*/
localManifold2 CollidePolygons(
    const polygon2& a,
    const polygon2& b, const transform2& transformB );

localManifold2 CollidePolygonSegment(
    const polygon2& polygon,
    const segment2& segment, const transform2& segmentTransform );

} // namespace zonai