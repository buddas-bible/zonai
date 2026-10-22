#include "dynamics/joints/splineRailJointConstraint2.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include "dynamics/constraintSoftness2.h"

namespace zonai
{
namespace
{

vec2 pointAt( const preparedSplineSegment2& segment, double t )
{
    if( t == 0.0 ) return segment.start;
    if( t == 1.0 ) return segment.end;
    return { static_cast<float>( segment.start.x + t * ( segment.tangent1.x + t * ( segment.quadratic.x + t * segment.cubic.x ) ) ), static_cast<float>( segment.start.y + t * ( segment.tangent1.y + t * ( segment.quadratic.y + t * segment.cubic.y ) ) ) };
}

double speedAt( const preparedSplineSegment2& segment, double t )
{
    return std::hypot( segment.tangent1.x + t * ( 2.0 * segment.quadratic.x + 3.0 * t * segment.cubic.x ), segment.tangent1.y + t * ( 2.0 * segment.quadratic.y + 3.0 * t * segment.cubic.y ) );
}

double arcLength( const preparedSplineSegment2& segment, double end )
{
    const auto integrate = [&]( auto&& self, double a, double b, double fa, double fm, double fb, double estimate, double error, int depth ) -> double
    {
        const double middle = 0.5 * ( a + b ), leftMiddle = 0.5 * ( a + middle ), rightMiddle = 0.5 * ( middle + b );
        const double fl = speedAt( segment, leftMiddle ), fr = speedAt( segment, rightMiddle );
        const double left = ( middle - a ) * ( fa + 4.0 * fl + fm ) / 6.0, right = ( b - middle ) * ( fm + 4.0 * fr + fb ) / 6.0;
        const double correction = ( left + right - estimate ) / 15.0;
        if( std::abs( correction ) <= error ) return left + right + correction;
        if( depth == 16 ) return std::numeric_limits<double>::quiet_NaN();
        return self( self, a, middle, fa, fl, fm, left, error * 0.5, depth + 1 ) + self( self, middle, b, fm, fr, fb, right, error * 0.5, depth + 1 );
    };
    if( end == 0.0 ) return 0.0;
    const double fa = speedAt( segment, 0.0 ), fm = speedAt( segment, end * 0.5 ), fb = speedAt( segment, end );
    return integrate( integrate, 0.0, end, fa, fm, fb, end * ( fa + 4.0 * fm + fb ) / 6.0, 1.0e-8 * std::max( { fa, fm, fb } ), 0 );
}

bool regular( const preparedSplineSegment2& segment )
{
    const double ax = segment.tangent1.x, ay = segment.tangent1.y, bx = 2.0 * segment.quadratic.x, by = 2.0 * segment.quadratic.y, cx = 3.0 * segment.cubic.x, cy = 3.0 * segment.cubic.y;
    const double c0 = ax * bx + ay * by, c1 = bx * bx + by * by + 2.0 * ( ax * cx + ay * cy ), c2 = 3.0 * ( bx * cx + by * cy ), c3 = 2.0 * ( cx * cx + cy * cy );
    const auto polynomial = [&]( double t )
    {
        return c0 + t * ( c1 + t * ( c2 + t * c3 ) );
    };
    std::array<double, 4> intervals{ 0.0, 1.0, 1.0, 1.0 };
    std::size_t count = 2;
    const auto add = [&]( double t )
    {
        if( t > 0.0 && t < 1.0 ) intervals[count++] = t;
    };
    if( c3 == 0.0 )
    {
        if( c2 != 0.0 ) add( -c1 / ( 2.0 * c2 ) );
    }
    else
    {
        const double discriminant = 4.0 * c2 * c2 - 12.0 * c3 * c1;
        if( discriminant >= 0.0 )
        {
            const double q = -c2 - std::copysign( std::sqrt( discriminant ) * 0.5, c2 );
            add( q / ( 3.0 * c3 ) );
            if( q != 0.0 ) add( c1 / q );
        }
    }
    std::sort( intervals.begin(), intervals.begin() + count );
    double minimum = std::min( speedAt( segment, 0.0 ), speedAt( segment, 1.0 ) );
    for( std::size_t i = 0; i < count; ++i )
        minimum = std::min( minimum, speedAt( segment, intervals[i] ) );
    for( std::size_t i = 1; i < count; ++i )
    {
        double a = intervals[i - 1], b = intervals[i];
        if( ( polynomial( a ) < 0.0 ) == ( polynomial( b ) < 0.0 ) ) continue;
        for( int iteration = 0; iteration < 50; ++iteration )
        {
            const double middle = 0.5 * ( a + b );
            if( ( polynomial( a ) < 0.0 ) == ( polynomial( middle ) < 0.0 ) )
                a = middle;
            else
                b = middle;
        }
        minimum = std::min( minimum, speedAt( segment, 0.5 * ( a + b ) ) );
    }
    const double scale = std::hypot( ax, ay ) + std::hypot( bx, by ) + std::hypot( cx, cy );
    return std::isfinite( minimum ) && minimum > 64.0 * std::numeric_limits<float>::epsilon() * scale;
}

// 이전 구간에서 연결된 투영을 추적해 교차점의 먼 분기로 바뀌지 않게 함.
double project( const preparedSplineSegment2& segment, vec2 point, double seed )
{
    const auto squaredDistance = [&]( double t )
    {
        const double x = segment.start.x + t * ( segment.tangent1.x + t * ( segment.quadratic.x + t * segment.cubic.x ) ) - point.x;
        const double y = segment.start.y + t * ( segment.tangent1.y + t * ( segment.quadratic.y + t * segment.cubic.y ) ) - point.y;
        return x * x + y * y;
    };
    double t = std::clamp( seed, 0.0, 1.0 );
    for( int iteration = 0; iteration < 64; ++iteration )
    {
        const double x = segment.start.x + t * ( segment.tangent1.x + t * ( segment.quadratic.x + t * segment.cubic.x ) ) - point.x;
        const double y = segment.start.y + t * ( segment.tangent1.y + t * ( segment.quadratic.y + t * segment.cubic.y ) ) - point.y;
        const double dx = segment.tangent1.x + t * ( 2.0 * segment.quadratic.x + 3.0 * t * segment.cubic.x ), dy = segment.tangent1.y + t * ( 2.0 * segment.quadratic.y + 3.0 * t * segment.cubic.y );
        const double gradient = x * dx + y * dy;
        const double speedSquared = dx * dx + dy * dy;
        const double hessian = speedSquared + x * ( 2.0 * segment.quadratic.x + 6.0 * t * segment.cubic.x ) + y * ( 2.0 * segment.quadratic.y + 6.0 * t * segment.cubic.y );
        if( !std::isfinite( gradient ) || !std::isfinite( hessian ) || speedSquared <= 0.0 ) break;
        if( ( t == 0.0 && gradient >= 0.0 ) || ( t == 1.0 && gradient <= 0.0 ) || std::abs( gradient ) <= 1.0e-11 * speedSquared ) return t;
        // 큰 이동으로 Newton 곡률이 음수가 되어도 같은 구간의 감소 방향으로 진행함.
        double step = std::clamp( gradient / ( hessian > 0.0 ? hessian : speedSquared ), -0.25, 0.25 );
        const double before = squaredDistance( t );
        double next = t;
        for( int backtrack = 0; backtrack < 32; ++backtrack )
        {
            next = std::clamp( t - step, 0.0, 1.0 );
            if( squaredDistance( next ) <= before ) break;
            step *= 0.5;
        }
        if( std::abs( next - t ) <= 1.0e-12 )
        {
            if( std::abs( gradient ) <= 1.0e-7 * speedSquared ) return next;
            break;
        }
        t = next;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

transform2 currentTransform( const bodySim& body, const bodyState& state )
{
    const rot2 rotation = state.deltaRotation * body.transform.rotation;
    return { body.center + state.deltaPosition - Rotate( rotation, body.localCenter ), rotation };
}

struct row2
{
    vec2 axis{};
    float a = 0.0f, b = 0.0f;
};

float velocity( const row2& row, const bodyState& a, const bodyState& b )
{
    return Dot( row.axis, b.linearVelocity - a.linearVelocity ) + row.b * b.angularVelocity - row.a * a.angularVelocity;
}

float mass( const row2& x, const row2& y, const splineRailJointConstraint2& c )
{
    return ( c.bodyA.invMass + c.bodyB.invMass ) * Dot( x.axis, y.axis ) + c.bodyA.invInertia * x.a * y.a + c.bodyB.invInertia * x.b * y.b;
}

void apply( const row2& row, float impulse, const splineRailJointConstraint2& c, bodyState& a, bodyState& b )
{
    a.linearVelocity -= c.bodyA.invMass * impulse * row.axis;
    a.angularVelocity -= c.bodyA.invInertia * impulse * row.a;
    b.linearVelocity += c.bodyB.invMass * impulse * row.axis;
    b.angularVelocity += c.bodyB.invInertia * impulse * row.b;
}

void clear( splineRailJointConstraint2& c )
{
    c.impulse = {};
    c.springImpulse = c.motorImpulse = c.lowerImpulse = c.upperImpulse = 0.0f;
    c.force = {};
    c.torque = 0.0f;
    c.trackingValid = false;
}

struct frameRows2
{
    splineRailFrame2 frame{};
    row2 normal{}, angular{}, arc{};
};

frameRows2 getRows( splineRailJointConstraint2& c, const bodyState& a, const bodyState& b )
{
    frameRows2 result{};
    const auto poseA = currentTransform( c.bodyA, a ), poseB = currentTransform( c.bodyB, b );
    const vec2 point = InverseTransformPoint( poseA, TransformPoint( poseB, c.joint->localAnchorB ) );
    auto seed = c.parameter;
    if( c.trackingValid && seed.segmentIndex < c.joint->segments.size() )
    {
        // 속도 적분으로 이동한 anchor를 이전 접선에 투영해 다음 parameter를 예측함.
        const auto& segment = c.joint->geometry.getSegments()[seed.segmentIndex];
        const vec2 derivative = segment.tangent1 + seed.t * ( 2.0f * segment.quadratic + 3.0f * seed.t * segment.cubic );
        const double squared = static_cast<double>( derivative.x ) * derivative.x + static_cast<double>( derivative.y ) * derivative.y;
        const vec2 displacement = point - c.trackedPoint;
        if( squared > 0.0 ) seed.t = static_cast<float>( std::clamp( seed.t + ( static_cast<double>( displacement.x ) * derivative.x + static_cast<double>( displacement.y ) * derivative.y ) / squared, 0.0, 1.0 ) );
    }
    c.trackedPoint = point;
    // 계산 실패는 impulse만 폐기하고 분기는 유지함. 전역 재투영은 생성·편집에서 수행함.
    result.frame = getSplineRailFrame( *c.joint, poseA, poseB, seed, true, c.distanceOffset );
    if( !result.frame.valid )
    {
        clear( c );
        return result;
    }
    c.parameter = result.frame.parameter;
    c.distanceOffset = result.frame.distanceOffset;
    c.trackingValid = true;
    const vec2 rA = result.frame.anchorB - ( c.bodyA.center + a.deltaPosition ), rB = result.frame.anchorB - ( c.bodyB.center + b.deltaPosition );
    const auto& f = result.frame;
    result.normal = { f.normal, Cross( rA, f.normal ), Cross( rB, f.normal ) };
    result.arc = { f.arcScale * f.tangent, f.arcScale * Cross( rA, f.tangent ), f.arcScale * Cross( rB, f.tangent ) };
    result.angular = { {}, 1.0f, 1.0f };
    if( c.joint->rotationMode == splineRailRotationMode::followTangent )
    {
        const float curvature = static_cast<float>( f.curvature );
        result.angular = { -curvature * result.arc.axis, 1.0f - curvature * result.arc.a, 1.0f - curvature * result.arc.b };
    }
    return result;
}

}

bool setSplineRailGeometry( splineRailJointSim2& joint, std::span<const vec2> points )
{
    auto geometry = prepareSpline( { points, joint.isLoop } );
    if( !geometry ) return false;

    std::vector<splineRailSegment2> segments;
    double length = 0.0;
    for( const auto& curve : geometry->getSegments() )
    {
        if( !regular( curve ) ) return false;
        splineRailSegment2 segment{};
        segment.prefix = length;
        segment.length = arcLength( curve, 1.0 );
        if( !std::isfinite( segment.length ) || segment.length <= 0.0 ) return false;
        length += segment.length;
        segments.push_back( segment );
    }
    if( !std::isfinite( length ) || ( !joint.isLoop && joint.enableLimit && ( joint.upperTranslation < -joint.referenceDistance || joint.lowerTranslation > length - joint.referenceDistance ) ) ) return false;

    joint.points.assign( points.begin(), points.end() );
    joint.geometry = std::move( *geometry );
    joint.segments = std::move( segments );
    joint.length = length;
    joint.impulse = {};
    joint.springImpulse = joint.motorImpulse = joint.lowerImpulse = joint.upperImpulse = 0.0f;
    joint.trackingValid = false;
    joint.force = {};
    joint.torque = 0.0f;
    return true;
}

splineRailFrame2 getSplineRailFrame( const splineRailJointSim2& joint, const transform2& bodyA, const transform2& bodyB, splineParameter2 seed, bool tracked, double distanceOffset )
{
    splineRailFrame2 frame{};
    frame.distanceOffset = distanceOffset;
    const auto curves = joint.geometry.getSegments();
    if( curves.empty() ) return frame;
    frame.anchorB = TransformPoint( bodyB, joint.localAnchorB );
    const vec2 point = InverseTransformPoint( bodyA, frame.anchorB );
    if( !IsFinite( point ) ) return frame;
    std::size_t index = std::min( seed.segmentIndex, curves.size() - 1 );
    double t = project( curves[index], point, seed.t );
    if( !tracked )
    {
        double best = std::numeric_limits<double>::infinity();
        for( std::size_t i = 0; i < curves.size(); ++i )
            for( int sample = 0; sample <= 8; ++sample )
            {
                const double candidate = project( curves[i], point, sample / 8.0 );
                const vec2 delta = pointAt( curves[i], candidate ) - point;
                const double squared = static_cast<double>( delta.x ) * delta.x + static_cast<double>( delta.y ) * delta.y;
                if( squared < best )
                {
                    best = squared;
                    index = i;
                    t = candidate;
                }
            }
    }
    else
    {
        for( std::size_t traversed = 0; traversed < curves.size(); ++traversed )
        {
            if( t <= 1.0e-9 && ( index > 0 || joint.isLoop ) )
            {
                const auto& segment = curves[index];
                if( Dot( point - segment.start, segment.tangent1 ) < 0.0f )
                {
                    if( index == 0 )
                    {
                        index = curves.size() - 1;
                        frame.distanceOffset -= joint.length;
                    }
                    else
                        --index;
                    t = project( curves[index], point, 1.0 );
                    continue;
                }
            }
            if( t >= 1.0 - 1.0e-9 && ( index + 1 < curves.size() || joint.isLoop ) )
            {
                const auto& segment = curves[index];
                const vec2 derivative = segment.tangent1 + 2.0f * segment.quadratic + 3.0f * segment.cubic;
                if( Dot( point - segment.end, derivative ) > 0.0f )
                {
                    if( index + 1 == curves.size() )
                    {
                        index = 0;
                        frame.distanceOffset += joint.length;
                    }
                    else
                        ++index;
                    t = project( curves[index], point, 0.0 );
                    continue;
                }
            }
            break;
        }
    }
    if( !std::isfinite( t ) ) return frame;
    const auto& segment = curves[index];
    const double dx = segment.tangent1.x + t * ( 2.0 * segment.quadratic.x + 3.0 * t * segment.cubic.x ), dy = segment.tangent1.y + t * ( 2.0 * segment.quadratic.y + 3.0 * t * segment.cubic.y );
    const double speed = std::hypot( dx, dy );
    if( !std::isfinite( speed ) || speed <= 0.0 ) return frame;
    frame.localPoint = pointAt( segment, t );
    frame.anchorA = TransformPoint( bodyA, frame.localPoint );
    const vec2 tangent{ static_cast<float>( dx / speed ), static_cast<float>( dy / speed ) };
    frame.tangent = Rotate( bodyA.rotation, tangent );
    frame.normal = Cross( 1.0f, frame.tangent );
    frame.curvature = ( dx * ( 2.0 * segment.quadratic.y + 6.0 * t * segment.cubic.y ) - dy * ( 2.0 * segment.quadratic.x + 6.0 * t * segment.cubic.x ) ) / ( speed * speed * speed );
    const vec2 separation = frame.anchorB - frame.anchorA;
    frame.lateralError = Dot( frame.normal, separation );
    const bool endpoint = !joint.isLoop && ( ( index == 0 && t == 0.0 ) || ( index + 1 == curves.size() && t == 1.0 ) );
    // 끝점 밖에서는 고정된 끝점 접선을 사용하므로 접선각의 거리 미분도 0임.
    if( endpoint && ( ( index == 0 && Dot( frame.tangent, separation ) < 0.0f ) || ( index + 1 == curves.size() && Dot( frame.tangent, separation ) > 0.0f ) ) ) frame.curvature = 0.0;
    const double denominator = endpoint ? 1.0 : 1.0 - frame.curvature * frame.lateralError;
    if( !std::isfinite( denominator ) || denominator <= 0.05 ) return frame;
    frame.arcScale = static_cast<float>( 1.0 / denominator );
    const auto& arc = joint.segments[index];
    frame.distance = arc.prefix + ( t == 1.0 ? arc.length : arcLength( segment, t ) );
    if( endpoint ) frame.distance += Dot( frame.tangent, separation );
    if( joint.isLoop ) frame.distance += frame.distanceOffset;
    frame.translation = frame.distance - joint.referenceDistance;
    frame.parameter = { index, static_cast<float>( t ) };
    const rot2 rotation = Inverse( bodyA.rotation ) * bodyB.rotation;
    const float target = joint.rotationMode == splineRailRotationMode::fixed ? joint.referenceAngle : std::atan2( tangent.y, tangent.x ) + joint.tangentOffset;
    frame.angleError = std::remainder( std::atan2( rotation.s, rotation.c ) - target, 2.0f * std::numbers::pi_v<float> );
    frame.valid = IsFinite( frame.anchorA ) && std::isfinite( frame.translation ) && std::isfinite( frame.curvature ) && std::isfinite( frame.angleError );
    return frame;
}

splineRailJointConstraint2 prepareSplineRailJointConstraint( const splineRailJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime )
{
    splineRailJointConstraint2 c{};
    c.jointId = joint.jointId;
    c.bodyIdA = joint.bodyIdA;
    c.bodyIdB = joint.bodyIdB;
    c.joint = &joint;
    c.bodyA = bodySimA;
    c.bodyB = bodySimB;
    c.subStepTime = subStepTime;
    c.parameter = joint.parameter;
    c.distanceOffset = joint.distanceOffset;
    c.trackingValid = joint.trackingValid;
    c.trackedPoint = InverseTransformPoint( bodySimA.transform, TransformPoint( bodySimB.transform, joint.localAnchorB ) );
    if( joint.subStepTime == subStepTime && bodySimA.invMass + bodySimA.invInertia + bodySimB.invMass + bodySimB.invInertia > 0.0f )
    {
        c.impulse = joint.impulse;
        c.springImpulse = joint.enableSpring && joint.hertz > 0.0f ? joint.springImpulse : 0.0f;
        c.lowerImpulse = joint.lowerImpulse;
        c.upperImpulse = joint.upperImpulse;
        c.motorImpulse = joint.enableMotor ? std::clamp( joint.motorImpulse, -joint.maxMotorForce * subStepTime, joint.maxMotorForce * subStepTime ) : 0.0f;
    }
    return c;
}

void warmStartSplineRailJointConstraint( splineRailJointConstraint2& c, bodyState& stateA, bodyState& stateB )
{
    bodyState a = stateA, b = stateB;
    const auto rows = getRows( c, a, b );
    if( !rows.frame.valid ) return;
    apply( rows.normal, c.impulse.x, c, a, b );
    apply( rows.angular, c.impulse.y, c, a, b );
    apply( rows.arc, c.springImpulse + c.motorImpulse + c.lowerImpulse - c.upperImpulse, c, a, b );
    if( !IsFinite( a.linearVelocity ) || !IsFinite( b.linearVelocity ) || !std::isfinite( a.angularVelocity ) || !std::isfinite( b.angularVelocity ) )
    {
        clear( c );
        return;
    }
    stateA = a;
    stateB = b;
}

void solveSplineRailJointConstraint( splineRailJointConstraint2& c, bodyState& stateA, bodyState& stateB, bool useBias )
{
    // 계산 실패 시 부분 impulse도 Body에 남기지 않음.
    bodyState a = stateA, b = stateB;
    const auto rows = getRows( c, a, b );
    if( !rows.frame.valid || c.subStepTime <= 0.0f ) return;
    const auto& joint = *c.joint;
    const auto softness = makeConstraintSoftness( std::min( 60.0f, 0.25f / c.subStepTime ), 2.0f, c.subStepTime );
    const float axialK = mass( rows.arc, rows.arc, c ), axialMass = axialK > 0.0f ? 1.0f / axialK : 0.0f;
    const float translation = static_cast<float>( rows.frame.translation );
    if( joint.enableSpring && joint.hertz > 0.0f )
    {
        const auto spring = makeConstraintSoftness( joint.hertz, joint.dampingRatio, c.subStepTime );
        const float bias = spring.biasRate * ( translation - joint.targetTranslation );
        const float impulse = -spring.massScale * axialMass * ( velocity( rows.arc, a, b ) + bias ) - spring.impulseScale * c.springImpulse;
        c.springImpulse += impulse;
        apply( rows.arc, impulse, c, a, b );
    }
    if( joint.enableMotor )
    {
        const float impulse = axialMass * ( joint.motorSpeed - velocity( rows.arc, a, b ) ), old = c.motorImpulse;
        c.motorImpulse = std::clamp( old + impulse, -joint.maxMotorForce * c.subStepTime, joint.maxMotorForce * c.subStepTime );
        apply( rows.arc, c.motorImpulse - old, c, a, b );
    }
    const float lower = joint.isLoop ? joint.lowerTranslation : static_cast<float>( std::max( -joint.referenceDistance, joint.enableLimit ? static_cast<double>( joint.lowerTranslation ) : -joint.referenceDistance ) );
    const float upper = joint.isLoop ? joint.upperTranslation : static_cast<float>( std::min( joint.length - joint.referenceDistance, joint.enableLimit ? static_cast<double>( joint.upperTranslation ) : joint.length - joint.referenceDistance ) );
    const auto limit = [&]( float separation, float direction, float& accumulated )
    {
        float bias = 0.0f, massScale = 1.0f, impulseScale = 0.0f;
        if( separation > 0.0f )
            bias = separation / c.subStepTime;
        else if( useBias )
        {
            bias = softness.biasRate * separation;
            massScale = softness.massScale;
            impulseScale = softness.impulseScale;
        }
        const float impulse = -massScale * axialMass * ( direction * velocity( rows.arc, a, b ) + bias ) - impulseScale * accumulated, old = accumulated;
        accumulated = std::max( 0.0f, old + impulse );
        apply( rows.arc, direction * ( accumulated - old ), c, a, b );
    };
    if( !joint.isLoop || joint.enableLimit )
    {
        limit( translation - lower, 1.0f, c.lowerImpulse );
        limit( upper - translation, -1.0f, c.upperImpulse );
    }
    const float k11 = mass( rows.normal, rows.normal, c ), k12 = mass( rows.normal, rows.angular, c ), k22 = mass( rows.angular, rows.angular, c );
    vec2 rhs{ velocity( rows.normal, a, b ), velocity( rows.angular, a, b ) }, delta{};
    if( joint.rotationMode == splineRailRotationMode::fixed )
    {
        float massScale = 1.0f, impulseScale = 0.0f;
        if( useBias )
        {
            rhs += softness.biasRate * vec2{ rows.frame.lateralError, rows.frame.angleError };
            massScale = softness.massScale;
            impulseScale = softness.impulseScale;
        }
        const float determinant = k11 * k22 - k12 * k12;
        const vec2 correction = determinant > 0.0f ? vec2{ ( k22 * rhs.x - k12 * rhs.y ) / determinant, ( k11 * rhs.y - k12 * rhs.x ) / determinant } : vec2{ k11 > 0.0f ? rhs.x / k11 : 0.0f, k22 > 0.0f ? rhs.y / k22 : 0.0f };
        delta = -massScale * correction - impulseScale * c.impulse;
    }
    else
    {
        const bool angularSpring = joint.enableAngularSpring && joint.angularHertz > 0.0f;
        const auto angular = angularSpring ? makeConstraintSoftness( joint.angularHertz, joint.angularDampingRatio, c.subStepTime ) : ( useBias ? softness : constraintSoftness2{} );
        const auto normal = useBias ? softness : constraintSoftness2{};
        const float gamma1 = normal.massScale > 0.0f ? k11 * normal.impulseScale / normal.massScale : 0.0f;
        const float gamma2 = angular.massScale > 0.0f ? k22 * angular.impulseScale / angular.massScale : 0.0f;
        rhs.x += ( useBias ? normal.biasRate * rows.frame.lateralError : 0.0f ) + gamma1 * c.impulse.x;
        rhs.y += ( angularSpring || useBias ? angular.biasRate * rows.frame.angleError : 0.0f ) + gamma2 * c.impulse.y;
        const float x = k11 + gamma1, y = k22 + gamma2, determinant = x * y - k12 * k12;
        delta = determinant > 0.0f ? vec2{ -( y * rhs.x - k12 * rhs.y ) / determinant, -( x * rhs.y - k12 * rhs.x ) / determinant } : vec2{ x > 0.0f ? -rhs.x / x : 0.0f, y > 0.0f ? -rhs.y / y : 0.0f };
    }
    if( !IsFinite( delta ) )
    {
        clear( c );
        return;
    }
    c.impulse += delta;
    apply( rows.normal, delta.x, c, a, b );
    apply( rows.angular, delta.y, c, a, b );
    const float axial = c.springImpulse + c.motorImpulse + c.lowerImpulse - c.upperImpulse;
    c.force = ( c.impulse.x * rows.normal.axis + c.impulse.y * rows.angular.axis + axial * rows.arc.axis ) / c.subStepTime;
    c.torque = ( c.impulse.x * rows.normal.b + c.impulse.y * rows.angular.b + axial * rows.arc.b ) / c.subStepTime;
    if( !IsFinite( a.linearVelocity ) || !IsFinite( b.linearVelocity ) || !std::isfinite( a.angularVelocity ) || !std::isfinite( b.angularVelocity ) || !IsFinite( c.force ) || !std::isfinite( c.torque ) )
    {
        clear( c );
        return;
    }
    stateA = a;
    stateB = b;
}

} // namespace zonai
