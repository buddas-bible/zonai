#pragma once

#include "geometry/segment2.h"
#include "geometry/circle2.h"
#include "geometry/capsule2.h"
#include "geometry/box2.h"
#include "geometry/polygon2.h"

#include "math/transform2.h"
#include "collision/narrowphase/manifold2.h"

namespace zonai
{


bool (*CollideFunctions[]) (const segment2&, const circle2&, const transform2&, manifold2&) = {
    Collide,    //,         // ,        // ,
	NULL,       //,         // ,        // ,
	NULL,      NULL,        // ,        // ,
	NULL,      NULL,        NULL,       //
    // ... other collision functions
};

/*
segment-circle
segment-box
segment-capsule
segment-polygon
*/
bool Collide(
    const segment2& segment,
    const circle2& circle, const transform2& circleTransform,
    manifold2& manifold );

bool Collide(
    const segment2& segment,
    const box2& box, const transform2& boxTransform,
    manifold2& manifold );

bool Collide(
    const segment2& segment,
    const capsule2& capsule, const transform2& capsuleTransform,
    manifold2& manifold );

bool Collide(
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

bool CollideCircleBox(
    const circle2& circle,
    const box2& box, const transform2& boxTransform,
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
box-box
box-capsule
box-polygon
*/
bool Collide(
    const box2& a,
    const box2& b, const transform2& transformB,
    manifold2& manifold );

bool Collide(
    const box2& box,
    const capsule2& capsule, const transform2& capsuleTransform,
    manifold2& manifold );

bool Collide(
    const box2& box,
    const polygon2& polygon, const transform2& polygonTransform,
    manifold2& manifold );


/*
capsule-capsule
capsule-polygon
*/
bool Collide(
    const capsule2& a,
    const capsule2& b, const transform2& transformB,
    manifold2& manifold );

bool Collide(
    const capsule2& capsule,
    const polygon2& polygon, const transform2& polygonTransform,
    manifold2& manifold );


/*
polygon-polygon
*/
bool Collide(
    const polygon2& a,
    const polygon2& b, const transform2& transformB,
    manifold2& manifold );

} // namespace zonai