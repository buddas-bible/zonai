#pragma once

#include "geometry/segment2.h"
#include "geometry/circle2.h"
#include "geometry/capsule2.h"
#include "geometry/polygon2.h"

#include "math/transform2.h"
#include "collision/narrowphase/manifold2.h"

namespace zonai
{

/*
segment-circle
segment-capsule
segment-polygon
*/
bool CollideSegmentCircle(
    const segment2& segment,
    const circle2& circle, const transform2& circleTransform,
    manifold2& manifold );

bool CollideCapsuleSegment(
    const capsule2& capsule, 
    const segment2& segment, const transform2& segmentTransform,
    manifold2& manifold );

bool CollidePolygonSegment(
    const polygon2& polygon,
    const segment2& segment, const transform2& segmentTransform,
    manifold2& manifold );


/*
circle-circle
circle-capsule
circle-polygon
*/
bool CollideCircles(
    const circle2& a,
    const circle2& b, const transform2& transformB,
    manifold2& manifold );

bool CollideCapsuleCircle(
    const capsule2& capsule,
    const circle2& circle, const transform2& circleTransform,
    manifold2& manifold );

bool CollidePolygonCircle(
    const polygon2& polygon,
    const circle2& circle, const transform2& circleTransform,
    manifold2& manifold );

/*
capsule-capsule
capsule-polygon
*/
bool CollideCapsules(
    const capsule2& a,
    const capsule2& b, const transform2& transformB,
    manifold2& manifold );

bool CollidePolygonCapsule(
    const polygon2& polygon, 
    const capsule2& capsule, const transform2& capsuleTransform,
    manifold2& manifold );


/*
polygon-polygon
*/
bool CollidePolygons(
    const polygon2& a,
    const polygon2& b, const transform2& transformB,
    manifold2& manifold );

} // namespace zonai