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
segment-box
segment-capsule
segment-polygon
*/
bool CollideSegmentCircle(
    const segment2& segment,
    const circle2& circle, const transform2& circleTransform,
    manifold2& manifold );

bool CollideSegmentCapsule(
    const segment2& segment,
    const capsule2& capsule, const transform2& capsuleTransform,
    manifold2& manifold );

bool CollideSegmentPolygon(
    const segment2& segment,
    const polygon2& polygon, const transform2& polygonTransform,
    manifold2& manifold );


/*
circle-circle
circle-box
circle-capsule
circle-polygon
*/
bool CollideCircles(
    const circle2& a,
    const circle2& b, const transform2& transformB,
    manifold2& manifold );

bool CollideCircleCapsule(
    const circle2& circle,
    const capsule2& capsule, const transform2& capsuleTransform,
    manifold2& manifold );

bool CollideCirclePolygon(
    const circle2& circle,
    const polygon2& polygon, const transform2& polygonTransform,
    manifold2& manifold );

/*
capsule-capsule
capsule-polygon
*/
bool CollideCapsules(
    const capsule2& a,
    const capsule2& b, const transform2& transformB,
    manifold2& manifold );

bool CollideCapsulePolygon(
    const capsule2& capsule,
    const polygon2& polygon, const transform2& polygonTransform,
    manifold2& manifold );


/*
polygon-polygon
*/
bool CollidePolygons(
    const polygon2& a,
    const polygon2& b, const transform2& transformB,
    manifold2& manifold );

} // namespace zonai