#include "fluid/fluidField2.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <numbers>

namespace zonai
{
namespace
{

bool ContainsFluidObstacle( const fluidShapeObstacle2& obstacle, vec2 point )
{
    const vec2 local = InverseTransformPoint( obstacle.transform, point );
    if( const auto* circle = std::get_if<circle2>( &obstacle.geometry ) )
        return circle->radius > 0.0f && Contains( *circle, local );

    if( const auto* capsule = std::get_if<capsule2>( &obstacle.geometry ) )
        return capsule->radius > 0.0f && Contains( *capsule, local );

    if( const auto* polygon = std::get_if<polygon2>( &obstacle.geometry ) )
    {
        if( polygon->vertexCount < 2 || polygon->vertexCount > static_cast<int>( MAX_POLYGON_VERTICES ) ||
            !std::isfinite( polygon->radius ) || polygon->radius < 0.0f ) return false;
        if( polygon->vertexCount == 2 )
            return polygon->radius > 0.0f &&
                DistanceSquared( segment2{ polygon->vertices[0], polygon->vertices[1] }, local ) <=
                    polygon->radius * polygon->radius;

        bool inside = true;
        float closestSquared = std::numeric_limits<float>::max();
        for( int i = 0; i < polygon->vertexCount; ++i )
        {
            if( Dot( polygon->normals[i], local - polygon->vertices[i] ) > 0.0f ) inside = false;
            const int next = ( i + 1 ) % polygon->vertexCount;
            closestSquared = std::min( closestSquared,
                DistanceSquared( segment2{ polygon->vertices[i], polygon->vertices[next] }, local ) );
        }
        return inside || ( polygon->radius > 0.0f && closestSquared <= polygon->radius * polygon->radius );
    }
    return false;
}

bool HasSameFluidGeometry( const shapeGeometry& a, const shapeGeometry& b )
{
    if( a.index() != b.index() ) return false;
    if( const auto* lhs = std::get_if<circle2>( &a ) )
    {
        const auto& rhs = std::get<circle2>( b );
        return lhs->radius == rhs.radius && lhs->center.x == rhs.center.x && lhs->center.y == rhs.center.y;
    }
    if( const auto* lhs = std::get_if<capsule2>( &a ) )
    {
        const auto& rhs = std::get<capsule2>( b );
        return lhs->radius == rhs.radius &&
            lhs->center1.x == rhs.center1.x && lhs->center1.y == rhs.center1.y &&
            lhs->center2.x == rhs.center2.x && lhs->center2.y == rhs.center2.y;
    }
    if( const auto* lhs = std::get_if<polygon2>( &a ) )
    {
        const auto& rhs = std::get<polygon2>( b );
        if( lhs->vertexCount != rhs.vertexCount || lhs->radius != rhs.radius ||
            lhs->vertexCount < 2 || lhs->vertexCount > static_cast<int>( MAX_POLYGON_VERTICES ) ) return false;
        for( int i = 0; i < lhs->vertexCount; ++i )
            if( lhs->vertices[i].x != rhs.vertices[i].x || lhs->vertices[i].y != rhs.vertices[i].y ||
                lhs->normals[i].x != rhs.normals[i].x || lhs->normals[i].y != rhs.normals[i].y )
                return false;
        return true;
    }
    return false;
}

float FluidGeometryRadius( const shapeGeometry& geometry )
{
    if( const auto* circle = std::get_if<circle2>( &geometry ) )
        return Length( circle->center ) + circle->radius;
    if( const auto* capsule = std::get_if<capsule2>( &geometry ) )
        return std::max( Length( capsule->center1 ), Length( capsule->center2 ) ) + capsule->radius;
    if( const auto* polygon = std::get_if<polygon2>( &geometry ) )
    {
        if( polygon->vertexCount < 2 || polygon->vertexCount > static_cast<int>( MAX_POLYGON_VERTICES ) )
            return std::numeric_limits<float>::quiet_NaN();
        float radius = polygon->radius;
        for( int i = 0; i < polygon->vertexCount; ++i )
            radius = std::max( radius, Length( polygon->vertices[i] ) + polygon->radius );
        return radius;
    }
    return 0.0f;
}

bool IsValidFluidObstacle( const fluidShapeObstacle2& obstacle )
{
    return IsFinite( obstacle.center ) && IsFinite( obstacle.transform.position ) &&
        std::isfinite( obstacle.transform.rotation.c ) && std::isfinite( obstacle.transform.rotation.s ) &&
        IsFinite( obstacle.linearVelocity ) && std::isfinite( obstacle.angularVelocity );
}

// 도형의 local 원점에서 모든 점을 감싸는 보수적인 world 반경.
// 회전이 비정규화된 입력이어도 false-negative culling을 피하도록 확대한다.
// 비정상 기하일 때는 성능 필터만 끄고 기존 Contains 판정을 유지한다.
double FluidObstacleBoundRadius( const fluidShapeObstacle2& obstacle )
{
    const double localRadius = FluidGeometryRadius( obstacle.geometry );
    const double scale = std::max( 1.0, std::hypot(
        static_cast<double>( obstacle.transform.rotation.c ),
        static_cast<double>( obstacle.transform.rotation.s ) ) );
    const double radius = localRadius * scale;
    return std::isfinite( radius ) && radius >= 0.0
        ? radius : std::numeric_limits<double>::infinity();
}

} // namespace

fluidField2::fluidField2( int width, int height, float cellSize, vec2 origin )
    : width_( width ), height_( height ), cellSize_( cellSize ), origin_( origin )
{
    if( width < 2 || width > 256 || height < 2 || height > 256 ||
        !std::isfinite( cellSize ) || cellSize <= 0.0f || !IsFinite( origin ) ||
        !std::isfinite( origin.x + width * cellSize ) || !std::isfinite( origin.y + height * cellSize ) )
        throw std::invalid_argument( "fluidField2 requires a finite 2..256 grid, positive cell size and finite bounds" );

    u_.resize( static_cast<std::size_t>( width + 1 ) * height );
    v_.resize( static_cast<std::size_t>( width ) * ( height + 1 ) );
    nextU_.resize( u_.size() );
    nextV_.resize( v_.size() );
    tracerMass_.assign( static_cast<std::size_t>( width ) * height, 0.0 );
    nextTracerMass_.resize( tracerMass_.size() );
    tracerOutgoing_.resize( tracerMass_.size() );
    tracerUFlux_.resize( u_.size() );
    tracerVFlux_.resize( v_.size() );
    pressure_.resize( static_cast<std::size_t>( width ) * height );
    nextPressure_.resize( pressure_.size() );
    divergence_.resize( pressure_.size() );
    solidOwner_.assign( pressure_.size(), -1 );
    previousSolidOwner_.assign( pressure_.size(), -1 );
    cellFluidFraction_.assign( pressure_.size(), 1.0f );
    previousCellFluidFraction_.assign( pressure_.size(), 1.0f );
    uOpenFraction_.assign( u_.size(), 1.0f );
    vOpenFraction_.assign( v_.size(), 1.0f );
    uSolidVelocity_.assign( u_.size(), 0.0f );
    vSolidVelocity_.assign( v_.size(), 0.0f );
    pressureUSolidFlux_.assign( u_.size(), 0.0f );
    pressureVSolidFlux_.assign( v_.size(), 0.0f );
    cutCellVolumeRate_.assign( pressure_.size(), 0.0f );
    sweptUVolume_.assign( u_.size(), 0.0f );
    sweptVVolume_.assign( v_.size(), 0.0f );
    cutCellComponent_.assign( pressure_.size(), -1 );
    cutCellNearestFluid_.assign( pressure_.size(), -1 );
    cutCellQueue_.reserve( pressure_.size() );
    for( int y = 0; y < height_; ++y )
    {
        uOpenFraction_[uIndex( 0, y )] = 0.0f;
        uOpenFraction_[uIndex( width_, y )] = 0.0f;
    }
    for( int x = 0; x < width_; ++x )
    {
        vOpenFraction_[vIndex( x, 0 )] = 0.0f;
        vOpenFraction_[vIndex( x, height_ )] = 0.0f;
    }
}

void fluidField2::setCutCellSampleCount( int count ) noexcept
{
    const int samples = count >= 8 ? 8 : 4;
    if( cutCellSampleCount_ == samples ) return;
    cutCellSampleCount_ = samples;

    // 서로 다른 이산 면적의 연속 snapshot을 차분하면 정지 물체의
    // 가짜 swept flux/체적 변화가 발생한다. 다음 step은 새로운 첫 snapshot.
    hasCutCellHistory_ = false;
    volumeSourceValid_ = false;
    previousObstacles_.clear();
    sweptFaceFlux_ = {};
    cutCellPressure_ = {};
}

void fluidField2::clear() noexcept
{
    std::fill( u_.begin(), u_.end(), 0.0f );
    std::fill( v_.begin(), v_.end(), 0.0f );
    std::fill( nextU_.begin(), nextU_.end(), 0.0f );
    std::fill( nextV_.begin(), nextV_.end(), 0.0f );
    std::fill( tracerMass_.begin(), tracerMass_.end(), 0.0 );
    std::fill( nextTracerMass_.begin(), nextTracerMass_.end(), 0.0 );
    std::fill( tracerOutgoing_.begin(), tracerOutgoing_.end(), 0.0 );
    std::fill( tracerUFlux_.begin(), tracerUFlux_.end(), 0.0 );
    std::fill( tracerVFlux_.begin(), tracerVFlux_.end(), 0.0 );
    tracerEnabled_ = false;
    tracerStats_ = {};
    std::fill( pressure_.begin(), pressure_.end(), 0.0f );
    std::fill( nextPressure_.begin(), nextPressure_.end(), 0.0f );
    std::fill( divergence_.begin(), divergence_.end(), 0.0f );
    std::fill( solidOwner_.begin(), solidOwner_.end(), -1 );
    std::fill( previousSolidOwner_.begin(), previousSolidOwner_.end(), -1 );
    std::fill( cellFluidFraction_.begin(), cellFluidFraction_.end(), 1.0f );
    std::fill( previousCellFluidFraction_.begin(), previousCellFluidFraction_.end(), 1.0f );
    std::fill( uSolidVelocity_.begin(), uSolidVelocity_.end(), 0.0f );
    std::fill( vSolidVelocity_.begin(), vSolidVelocity_.end(), 0.0f );
    std::fill( pressureUSolidFlux_.begin(), pressureUSolidFlux_.end(), 0.0f );
    std::fill( pressureVSolidFlux_.begin(), pressureVSolidFlux_.end(), 0.0f );
    std::fill( cutCellVolumeRate_.begin(), cutCellVolumeRate_.end(), 0.0f );
    std::fill( sweptUVolume_.begin(), sweptUVolume_.end(), 0.0f );
    std::fill( sweptVVolume_.begin(), sweptVVolume_.end(), 0.0f );
    previousObstacles_.clear();
    sweptFaceFlux_ = {};
    std::fill( cutCellComponent_.begin(), cutCellComponent_.end(), -1 );
    std::fill( cutCellNearestFluid_.begin(), cutCellNearestFluid_.end(), -1 );
    cutCellQueue_.clear();
    hasCutCellHistory_ = false;
    volumeSourceValid_ = false;
    std::fill( uOpenFraction_.begin(), uOpenFraction_.end(), 1.0f );
    std::fill( vOpenFraction_.begin(), vOpenFraction_.end(), 1.0f );
    for( int y = 0; y < height_; ++y )
    {
        uOpenFraction_[uIndex( 0, y )] = 0.0f;
        uOpenFraction_[uIndex( width_, y )] = 0.0f;
    }
    for( int x = 0; x < width_; ++x )
    {
        vOpenFraction_[vIndex( x, 0 )] = 0.0f;
        vOpenFraction_[vIndex( x, height_ )] = 0.0f;
    }
    solidCellCount_ = 0;
    solidTransition_ = {};
    cutCellTransition_ = {};
    cutCellPressure_ = {};
    advectionStats_ = {};
    pressureMomentum_ = {};
    stepMomentum_ = {};
}

float fluidField2::tracerCellFraction( int x, int y ) const noexcept
{
    if( x < 0 || x >= width_ || y < 0 || y >= height_ ) return 0.0f;
    return cutCellProjectionEnabled_ ? cellFluidFraction_[cellIndex( x, y )] :
        ( isSolidCell( x, y ) ? 0.0f : 1.0f );
}

void fluidField2::setTracerConcentration( int x, int y, float concentration ) noexcept
{
    if( !std::isfinite( concentration ) || concentration < 0.0f ||
        x < 0 || x >= width_ || y < 0 || y >= height_ ) return;
    const float fraction = tracerCellFraction( x, y );
    if( fraction <= 0.0f && concentration > 0.0f ) return;
    const double mass = static_cast<double>( concentration ) * cellSize_ * cellSize_ * fraction;
    if( !std::isfinite( mass ) ) return;
    tracerMass_[cellIndex( x, y )] = mass;
    tracerEnabled_ = true;
}

float fluidField2::getTracerConcentration( int x, int y ) const noexcept
{
    const float fraction = tracerCellFraction( x, y );
    if( fraction <= 0.0f ) return 0.0f;
    return static_cast<float>( tracerMass_[cellIndex( x, y )] /
        ( static_cast<double>( cellSize_ ) * cellSize_ * fraction ) );
}

float fluidField2::getTracerMass() const noexcept
{
    double mass = 0.0;
    for( const double value : tracerMass_ ) mass += value;
    return static_cast<float>( mass );
}

vec2 fluidField2::getCellCenter( int x, int y ) const noexcept
{
    return origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) + 0.5f };
}

float fluidField2::sampleComponent( const std::vector<float>& values, int width, int height, float x, float y ) noexcept
{
    x = std::clamp( x, 0.0f, static_cast<float>( width - 1 ) );
    y = std::clamp( y, 0.0f, static_cast<float>( height - 1 ) );
    const int i = static_cast<int>( x );
    const int j = static_cast<int>( y );
    const int i1 = std::min( i + 1, width - 1 );
    const int j1 = std::min( j + 1, height - 1 );
    const float tx = x - i;
    const float ty = y - j;
    const float bottom = std::lerp( values[j * width + i], values[j * width + i1], tx );
    const float top = std::lerp( values[j1 * width + i], values[j1 * width + i1], tx );
    return std::lerp( bottom, top, ty );
}

vec2 fluidField2::sampleFrom( const std::vector<float>& u, const std::vector<float>& v, vec2 position ) const noexcept
{
    if( !IsFinite( position ) ) return {};
    const vec2 grid = ( position - origin_ ) / cellSize_;
    return {
        sampleComponent( u, width_ + 1, height_, grid.x, grid.y - 0.5f ),
        sampleComponent( v, width_, height_ + 1, grid.x - 0.5f, grid.y )
    };
}

vec2 fluidField2::sampleVelocity( vec2 position ) const noexcept
{
    // 바깥 영역에 유체가 있다고 가정하지 않음.
    if( !IsFinite( position ) || position.x < origin_.x || position.y < origin_.y ||
        position.x > origin_.x + width_ * cellSize_ || position.y > origin_.y + height_ * cellSize_ ) return {};
    return sampleFrom( u_, v_, position );
}

void fluidField2::addVelocity( vec2 position, vec2 velocityDelta, float radius )
{
    if( !IsFinite( position ) || !IsFinite( velocityDelta ) || !std::isfinite( radius ) || radius <= 0.0f ) return;

    // 중심에서 선형 감쇠하는 국소 속도 source. 경계 face는 no-through이므로 건드리지 않음.
    const float radiusSquared = radius * radius;
    if( !std::isfinite( radiusSquared ) ) return;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
            const vec2 offset = face - position;
            const float distanceSquared = Dot( offset, offset );
            if( distanceSquared < radiusSquared )
                u_[uIndex( x, y )] += velocityDelta.x * ( 1.0f - distanceSquared / radiusSquared );
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
            const vec2 offset = face - position;
            const float distanceSquared = Dot( offset, offset );
            if( distanceSquared < radiusSquared )
                v_[vIndex( x, y )] += velocityDelta.y * ( 1.0f - distanceSquared / radiusSquared );
        }
}

vec2 fluidField2::getFluidMomentum() const noexcept
{
    // 각 내부 fluid-fluid face의 dual volume은 h² * unitDepth.
    // Solid 경계 속도는 지정된 운동학 경계값이므로 유체 운동량 적분에서 제외.
    const double faceMass = static_cast<double>( cellSize_ ) * cellSize_;
    double x = 0.0;
    double y = 0.0;
    for( int j = 0; j < height_; ++j )
        for( int i = 1; i < width_; ++i )
            if( !isSolidCell( i - 1, j ) && !isSolidCell( i, j ) )
                x += u_[uIndex( i, j )];

    for( int j = 1; j < height_; ++j )
        for( int i = 0; i < width_; ++i )
            if( !isSolidCell( i, j - 1 ) && !isSolidCell( i, j ) )
                y += v_[vIndex( i, j )];

    return { static_cast<float>( x * faceMass ), static_cast<float>( y * faceMass ) };
}

double fluidField2::getFluidKineticEnergy() const noexcept
{
    // fluid-fluid face에만 정의한 MAC dual kinetic energy. 전체 유체 에너지는 아님.
    double squared = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( !isSolidCell( x - 1, y ) && !isSolidCell( x, y ) )
            {
                const double speed = u_[uIndex( x, y )];
                squared += speed * speed;
            }
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( !isSolidCell( x, y - 1 ) && !isSolidCell( x, y ) )
            {
                const double speed = v_[vIndex( x, y )];
                squared += speed * speed;
            }
    return 0.5 * static_cast<double>( cellSize_ ) * cellSize_ * squared;
}

vec2 fluidField2::applyMomentumImpulse( vec2 position, vec2 impulse, float radius )
{
    if( !IsFinite( position ) || !IsFinite( impulse ) || !std::isfinite( radius ) || radius <= 0.0f )
        return {};

    const float radiusSquared = radius * radius;
    const float faceMass = cellSize_ * cellSize_; // 1 kg/m³ * h² * 1 m
    if( !std::isfinite( radiusSquared ) || !std::isfinite( faceMass ) || faceMass <= 0.0f )
        return {};

    // X/Y 성분은 서로 다른 위치의 MAC face에 존재하므로 독립 정규화한다.
    // 압력 보정/벽 제약은 이 교환 다음 Step에 따로 작용한다.
    const auto weight = [&]( vec2 face )
    {
        const vec2 delta = face - position;
        return std::max( 0.0f, 1.0f - Dot( delta, delta ) / radiusSquared );
    };

    double sumU = 0.0;
    double sumV = 0.0;
    for( int j = 0; j < height_; ++j )
        for( int i = 1; i < width_; ++i )
        {
            if( isSolidCell( i - 1, j ) || isSolidCell( i, j ) ) continue;
            const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( i ), static_cast<float>( j ) + 0.5f };
            sumU += weight( face );
        }

    for( int j = 1; j < height_; ++j )
        for( int i = 0; i < width_; ++i )
        {
            if( isSolidCell( i, j - 1 ) || isSolidCell( i, j ) ) continue;
            const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( i ) + 0.5f, static_cast<float>( j ) };
            sumV += weight( face );
        }

    // 너무 큰 impulse/너무 작은 cell을 다룰 때 float face 속도 overflow 방지.
    const double xScale = sumU > 0.0 ? static_cast<double>( impulse.x ) / ( faceMass * sumU ) : 0.0;
    const double yScale = sumV > 0.0 ? static_cast<double>( impulse.y ) / ( faceMass * sumV ) : 0.0;
    if( !std::isfinite( xScale ) || !std::isfinite( yScale ) ||
        std::abs( xScale ) > std::numeric_limits<float>::max() ||
        std::abs( yScale ) > std::numeric_limits<float>::max() ) return {};

    double actualX = 0.0;
    double actualY = 0.0;
    if( impulse.x != 0.0f && sumU > 0.0 )
    {
        for( int j = 0; j < height_; ++j )
            for( int i = 1; i < width_; ++i )
            {
                if( isSolidCell( i - 1, j ) || isSolidCell( i, j ) ) continue;
                const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( i ), static_cast<float>( j ) + 0.5f };
                const float w = weight( face );
                if( w <= 0.0f ) continue;
                const int index = uIndex( i, j );
                const float old = u_[index];
                const float next = old + static_cast<float>( w * xScale );
                if( !std::isfinite( next ) ) continue;
                u_[index] = next;
                actualX += static_cast<double>( next - old ) * faceMass;
            }
    }

    if( impulse.y != 0.0f && sumV > 0.0 )
    {
        for( int j = 1; j < height_; ++j )
            for( int i = 0; i < width_; ++i )
            {
                if( isSolidCell( i, j - 1 ) || isSolidCell( i, j ) ) continue;
                const vec2 face = origin_ + cellSize_ * vec2{ static_cast<float>( i ) + 0.5f, static_cast<float>( j ) };
                const float w = weight( face );
                if( w <= 0.0f ) continue;
                const int index = vIndex( i, j );
                const float old = v_[index];
                const float next = old + static_cast<float>( w * yScale );
                if( !std::isfinite( next ) ) continue;
                v_[index] = next;
                actualY += static_cast<double>( next - old ) * faceMass;
            }
    }
    return { static_cast<float>( actualX ), static_cast<float>( actualY ) };
}

void fluidField2::step( float timeStep, int pressureIterations )
{
    step( timeStep, std::span<const fluidShapeObstacle2>{}, std::span<fluidObstacleForce2>{}, pressureIterations );
}

void fluidField2::updateObstacles( std::span<const fluidShapeObstacle2> obstacles )
{
    std::copy( solidOwner_.begin(), solidOwner_.end(), previousSolidOwner_.begin() );
    solidTransition_ = {};
    std::fill( solidOwner_.begin(), solidOwner_.end(), -1 );
    solidCellCount_ = 0;
    for( std::size_t index = 0; index < obstacles.size(); ++index )
    {
        const fluidShapeObstacle2& obstacle = obstacles[index];
        if( !IsValidFluidObstacle( obstacle ) ) continue;

        // Cell-center occupancy는 도형의 보수적 AABB 범위에서만 검사한다.
        // 멀리 떨어진 Static 장애물이 늘어도 격자 전체를 도형마다 순회하지 않는다.
        const double radius = FluidObstacleBoundRadius( obstacle );
        const auto boundIndex = [&]( double coordinate, float origin, int limit, bool end )
        {
            const double cell = ( coordinate - origin ) / cellSize_ - 0.5;
            const double inclusive = end ? std::ceil( cell ) + 2.0 : std::floor( cell ) - 1.0;
            return static_cast<int>( std::clamp( inclusive, 0.0, static_cast<double>( limit ) ) );
        };
        const double x = obstacle.transform.position.x;
        const double y = obstacle.transform.position.y;
        const int minX = boundIndex( x - radius, origin_.x, width_, false );
        const int maxX = boundIndex( x + radius, origin_.x, width_, true );
        const int minY = boundIndex( y - radius, origin_.y, height_, false );
        const int maxY = boundIndex( y + radius, origin_.y, height_, true );

        // 최초의 원이 cell을 소유함. 겹치는 도형의 우선순위는 유지한다.
        for( int cellY = minY; cellY < maxY; ++cellY )
            for( int cellX = minX; cellX < maxX; ++cellX )
            {
                const int cell = cellIndex( cellX, cellY );
                if( solidOwner_[cell] >= 0 ) continue;
                if( ContainsFluidObstacle( obstacle, getCellCenter( cellX, cellY ) ) )
                {
                    solidOwner_[cell] = static_cast<int>( index );
                    ++solidCellCount_;
                }
            }
    }

    for( std::size_t cell = 0; cell < solidOwner_.size(); ++cell )
    {
        if( previousSolidOwner_[cell] >= 0 && solidOwner_[cell] < 0 ) ++solidTransition_.exposedCells;
        if( previousSolidOwner_[cell] < 0 && solidOwner_[cell] >= 0 ) ++solidTransition_.coveredCells;
    }
    solidTransition_.fluidVolumeDelta =
        static_cast<float>( solidTransition_.exposedCells - solidTransition_.coveredCells ) * cellSize_ * cellSize_;
}

float fluidField2::getCellFluidFraction( int x, int y ) const noexcept
{
    return x >= 0 && x < width_ && y >= 0 && y < height_ ? cellFluidFraction_[cellIndex( x, y )] : 0.0f;
}

float fluidField2::getUFaceOpenFraction( int x, int y ) const noexcept
{
    return x > 0 && x < width_ && y >= 0 && y < height_ ? uOpenFraction_[uIndex( x, y )] : 0.0f;
}

float fluidField2::getVFaceOpenFraction( int x, int y ) const noexcept
{
    return x >= 0 && x < width_ && y > 0 && y < height_ ? vOpenFraction_[vIndex( x, y )] : 0.0f;
}

float fluidField2::getUFaceSolidVelocity( int x, int y ) const noexcept
{
    return x > 0 && x < width_ && y >= 0 && y < height_
        ? uSolidVelocity_[uIndex( x, y )] : 0.0f;
}

float fluidField2::getVFaceSolidVelocity( int x, int y ) const noexcept
{
    return x >= 0 && x < width_ && y > 0 && y < height_
        ? vSolidVelocity_[vIndex( x, y )] : 0.0f;
}

void fluidField2::updateCutCellGeometry( std::span<const fluidShapeObstacle2> obstacles )
{
    // 이진 cell-center mask와 별도로 4x4 sub-cell / 4개 face 표본을 계산.
    // 선택적 가중 압력 경로에서 면 개방율과 강체 속도를 직접 사용한다.
    const int sampleCount = cutCellSampleCount_;
    const float inverseCount = 1.0f / sampleCount;
    const float inverseCellSamples = inverseCount * inverseCount;
    std::copy( cellFluidFraction_.begin(), cellFluidFraction_.end(), previousCellFluidFraction_.begin() );
    volumeSourceValid_ = hasCutCellHistory_;
    hasCutCellHistory_ = true;
    cutCellTransition_ = {};
    std::fill( uSolidVelocity_.begin(), uSolidVelocity_.end(), 0.0f );
    std::fill( vSolidVelocity_.begin(), vSolidVelocity_.end(), 0.0f );

    double previousVolume = 0.0;
    for( const float fraction : previousCellFluidFraction_ ) previousVolume += fraction;
    cutCellTransition_.previousFluidVolume = static_cast<float>(
        previousVolume * static_cast<double>( cellSize_ ) * cellSize_ );

    if( obstacles.empty() )
    {
        std::fill( cellFluidFraction_.begin(), cellFluidFraction_.end(), 1.0f );
        std::fill( uOpenFraction_.begin(), uOpenFraction_.end(), 1.0f );
        std::fill( vOpenFraction_.begin(), vOpenFraction_.end(), 1.0f );
    }
    else
    {
        // 서브셀 16표본과 MAC face 4표본 각각에 대해 Shape Contains를
        // 호출하기 전에 도형의 보수적 world 반경으로 대부분을 걸러낸다.
        std::vector<double> bounds( obstacles.size(), -1.0 );
        for( std::size_t i = 0; i < obstacles.size(); ++i )
            if( IsValidFluidObstacle( obstacles[i] ) )
                bounds[i] = FluidObstacleBoundRadius( obstacles[i] );
        const auto firstOwner = [&]( vec2 point )
        {
            for( std::size_t i = 0; i < obstacles.size(); ++i )
            {
                const double radius = bounds[i];
                if( radius < 0.0 ) continue;
                const vec2 center = obstacles[i].transform.position;
                if( std::abs( static_cast<double>( point.x ) - center.x ) > radius ||
                    std::abs( static_cast<double>( point.y ) - center.y ) > radius ) continue;
                if( ContainsFluidObstacle( obstacles[i], point ) ) return static_cast<int>( i );
            }
            return -1;
        };
        const auto isOccupied = [&]( vec2 point ) { return firstOwner( point ) >= 0; };

        // 대부분의 셀/face는 장애물 바깥이다. 보수적 원형 경계의 AABB와
        // 대상 셀/면 구간이 만나지 않으면 공간 N×N 표본 검사를 생략한다.
        // 반경 여유는 경계 부동소수점 오차로 인한 누락을 막는다.
        const auto mayOverlap = [&]( vec2 center, vec2 halfSize )
        {
            constexpr double tolerance = 1.0e-5;
            for( std::size_t i = 0; i < obstacles.size(); ++i )
            {
                if( bounds[i] < 0.0 ) continue;
                const vec2 body = obstacles[i].transform.position;
                if( std::abs( static_cast<double>( center.x ) - body.x ) >
                        bounds[i] + halfSize.x + tolerance ||
                    std::abs( static_cast<double>( center.y ) - body.y ) >
                        bounds[i] + halfSize.y + tolerance ) continue;
                return true;
            }
            return false;
        };

        for( int y = 0; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const vec2 center = origin_ + cellSize_ * vec2{
                    static_cast<float>( x ) + 0.5f, static_cast<float>( y ) + 0.5f };
                if( !mayOverlap( center, { 0.5f * cellSize_, 0.5f * cellSize_ } ) )
                {
                    cellFluidFraction_[cellIndex( x, y )] = 1.0f;
                    continue;
                }
                int covered = 0;
                for( int sy = 0; sy < sampleCount; ++sy )
                    for( int sx = 0; sx < sampleCount; ++sx )
                    {
                        const vec2 point = origin_ + cellSize_ * vec2{
                            static_cast<float>( x ) + ( sx + 0.5f ) * inverseCount,
                            static_cast<float>( y ) + ( sy + 0.5f ) * inverseCount };
                        if( isOccupied( point ) ) ++covered;
                    }
                cellFluidFraction_[cellIndex( x, y )] = 1.0f - covered * inverseCellSamples;
            }

        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const vec2 center = origin_ + cellSize_ * vec2{
                    static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
                if( !mayOverlap( center, { 0.0f, 0.5f * cellSize_ } ) )
                {
                    uOpenFraction_[uIndex( x, y )] = 1.0f;
                    continue;
                }
                int blocked = 0;
                for( int sample = 0; sample < sampleCount; ++sample )
                {
                    const vec2 point = origin_ + cellSize_ * vec2{
                        static_cast<float>( x ), static_cast<float>( y ) + ( sample + 0.5f ) * inverseCount };
                    const int owner = firstOwner( point );
                    if( owner >= 0 )
                    {
                        ++blocked;
                        const fluidShapeObstacle2& shape = obstacles[static_cast<std::size_t>( owner )];
                        uSolidVelocity_[uIndex( x, y )] +=
                            ( shape.linearVelocity + Cross( shape.angularVelocity, point - shape.center ) ).x;
                    }
                }
                uOpenFraction_[uIndex( x, y )] = 1.0f - blocked * inverseCount;
                if( blocked > 0 ) uSolidVelocity_[uIndex( x, y )] /= blocked;
            }

        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const vec2 center = origin_ + cellSize_ * vec2{
                    static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
                if( !mayOverlap( center, { 0.5f * cellSize_, 0.0f } ) )
                {
                    vOpenFraction_[vIndex( x, y )] = 1.0f;
                    continue;
                }
                int blocked = 0;
                for( int sample = 0; sample < sampleCount; ++sample )
                {
                    const vec2 point = origin_ + cellSize_ * vec2{
                        static_cast<float>( x ) + ( sample + 0.5f ) * inverseCount, static_cast<float>( y ) };
                    const int owner = firstOwner( point );
                    if( owner >= 0 )
                    {
                        ++blocked;
                        const fluidShapeObstacle2& shape = obstacles[static_cast<std::size_t>( owner )];
                        vSolidVelocity_[vIndex( x, y )] +=
                            ( shape.linearVelocity + Cross( shape.angularVelocity, point - shape.center ) ).y;
                    }
                }
                vOpenFraction_[vIndex( x, y )] = 1.0f - blocked * inverseCount;
                if( blocked > 0 ) vSolidVelocity_[vIndex( x, y )] /= blocked;
            }
    }

    // Exterior faces always remain closed even in a geometrically empty domain.
    for( int y = 0; y < height_; ++y )
    {
        uOpenFraction_[uIndex( 0, y )] = 0.0f;
        uOpenFraction_[uIndex( width_, y )] = 0.0f;
    }
    for( int x = 0; x < width_; ++x )
    {
        vOpenFraction_[vIndex( x, 0 )] = 0.0f;
        vOpenFraction_[vIndex( x, height_ )] = 0.0f;
    }

    double current = 0.0;
    double covered = 0.0;
    double released = 0.0;
    for( std::size_t cell = 0; cell < cellFluidFraction_.size(); ++cell )
    {
        const float old = previousCellFluidFraction_[cell];
        const float now = cellFluidFraction_[cell];
        current += now;
        covered += std::max( 0.0f, old - now );
        released += std::max( 0.0f, now - old );
        if( now > 0.0f && now < 1.0f ) ++cutCellTransition_.partialCells;
    }
    for( const float open : uOpenFraction_ )
        if( open > 0.0f && open < 1.0f ) ++cutCellTransition_.partialUFaces;
    for( const float open : vOpenFraction_ )
        if( open > 0.0f && open < 1.0f ) ++cutCellTransition_.partialVFaces;

    const double volumeScale = static_cast<double>( cellSize_ ) * cellSize_; // unit depth = 1m
    cutCellTransition_.fluidVolume = static_cast<float>( current * volumeScale );
    cutCellTransition_.coveredVolume = static_cast<float>( covered * volumeScale );
    cutCellTransition_.releasedVolume = static_cast<float>( released * volumeScale );
}

void fluidField2::remapExposedFaces()
{
    // 새로 열린 fluid-fluid face가 이전 rigid boundary의 속도를 그대로 상속하지 않도록
    // 직전 Step에서도 fluid이고 이번 Step에도 fluid인 face만 source로 사용한다.
    // nextU_/nextV_는 이류 전에 비어있는 scratch이므로 원래 속도를 복사해둔다.
    if( solidTransition_.exposedCells > 0 )
    {
        std::copy( u_.begin(), u_.end(), nextU_.begin() );
        std::copy( v_.begin(), v_.end(), nextV_.begin() );
    }

    const auto oldFluid = [&]( int x, int y ) { return previousSolidOwner_[cellIndex( x, y )] < 0; };
    const auto newFluid = [&]( int x, int y ) { return solidOwner_[cellIndex( x, y )] < 0; };
    const auto validU = [&]( int x, int y )
    {
        return x > 0 && x < width_ && y >= 0 && y < height_ &&
            oldFluid( x - 1, y ) && oldFluid( x, y ) &&
            newFluid( x - 1, y ) && newFluid( x, y );
    };
    const auto validV = [&]( int x, int y )
    {
        return x >= 0 && x < width_ && y > 0 && y < height_ &&
            oldFluid( x, y - 1 ) && oldFluid( x, y ) &&
            newFluid( x, y - 1 ) && newFluid( x, y );
    };

    // 가까운 유효 유체 면의 직전 속도를 복사한다. 이전 solid 경계 속도나
    // 이번 frame에 함께 새로 열린 면은 참조하지 않으므로 순회 순서와 무관하다.
    const auto extrapolate = [&]( int x, int y, bool horizontal )
    {
        for( int distance = 1; distance <= 4; ++distance )
        {
            double sum = 0.0;
            int count = 0;
            for( int dx = -distance; dx <= distance; ++dx )
            {
                const int dy = distance - std::abs( dx );
                const auto sample = [&]( int sx, int sy )
                {
                    if( horizontal ? validU( sx, sy ) : validV( sx, sy ) )
                    {
                        sum += horizontal ? nextU_[uIndex( sx, sy )] : nextV_[vIndex( sx, sy )];
                        ++count;
                    }
                };
                sample( x + dx, y + dy );
                if( dy != 0 ) sample( x + dx, y - dy );
            }
            if( count > 0 )
            {
                ++solidTransition_.extrapolatedFaces;
                return static_cast<float>( sum / count );
            }
        }
        ++solidTransition_.fallbackFaces;
        return 0.0f;
    };

    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const bool wasFluid = oldFluid( x - 1, y ) && oldFluid( x, y );
            const bool nowFluid = newFluid( x - 1, y ) && newFluid( x, y );
            if( wasFluid && !nowFluid ) ++solidTransition_.closedUFaces;
            if( !wasFluid && nowFluid )
            {
                ++solidTransition_.openedUFaces;
                u_[uIndex( x, y )] = extrapolate( x, y, true );
            }
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const bool wasFluid = oldFluid( x, y - 1 ) && oldFluid( x, y );
            const bool nowFluid = newFluid( x, y - 1 ) && newFluid( x, y );
            if( wasFluid && !nowFluid ) ++solidTransition_.closedVFaces;
            if( !wasFluid && nowFluid )
            {
                ++solidTransition_.openedVFaces;
                v_[vIndex( x, y )] = extrapolate( x, y, false );
            }
        }
}

bool fluidField2::isSolidCell( int x, int y ) const noexcept
{
    return x >= 0 && x < width_ && y >= 0 && y < height_ && solidOwner_[cellIndex( x, y )] >= 0;
}

void fluidField2::applySolidBoundaries( std::span<const fluidShapeObstacle2> obstacles,
    std::span<const fluidObstacleForce2> pressureFeedback, float timeStep )
{
    // 압력 투영 전에는 전달된 Body 속도로 no-through 경계를 설정한다.
    // 수렴한 Schur 압력 이후에는 같은 pressure traction으로 계산한
    // Δv=dt*M^-1*F, Δomega=dt*I^-1*tau를 같은 MAC face에도 반영한다.
    const bool applyPressure = timeStep > 0.0f && pressureFeedback.size() == obstacles.size();
    double changedSquared = 0.0;
    int boundaryFaces = 0;
    const auto predictedVelocity = [&]( int owner, vec2 point )
    {
        const fluidShapeObstacle2& solid = obstacles[static_cast<std::size_t>( owner )];
        vec2 linear = solid.linearVelocity;
        float angular = solid.angularVelocity;
        if( applyPressure )
        {
            const fluidObstacleForce2& pressure = pressureFeedback[static_cast<std::size_t>( owner )];
            linear += ( timeStep * solid.inverseMass ) * pressure.force;
            angular += timeStep * solid.inverseInertia * pressure.torque;
        }
        return linear + Cross( angular, point - solid.center );
    };

    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const int left = solidOwner_[cellIndex( x - 1, y )];
            const int right = solidOwner_[cellIndex( x, y )];
            const int owner = left >= 0 ? left : right;
            if( owner < 0 ) continue;
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
            const int face = uIndex( x, y );
            const float next = predictedVelocity( owner, point ).x;
            if( applyPressure && ( left < 0 || right < 0 ) )
            {
                const double difference = static_cast<double>( next ) - u_[face];
                changedSquared += difference * difference;
                ++boundaryFaces;
            }
            u_[face] = next;
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int bottom = solidOwner_[cellIndex( x, y - 1 )];
            const int top = solidOwner_[cellIndex( x, y )];
            const int owner = bottom >= 0 ? bottom : top;
            if( owner < 0 ) continue;
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
            const int face = vIndex( x, y );
            const float next = predictedVelocity( owner, point ).y;
            if( applyPressure && ( bottom < 0 || top < 0 ) )
            {
                const double difference = static_cast<double>( next ) - v_[face];
                changedSquared += difference * difference;
                ++boundaryFaces;
            }
            v_[face] = next;
        }

    if( applyPressure )
    {
        pressureMomentum_.synchronizedBoundaryFaces = boundaryFaces;
        pressureMomentum_.boundaryNormalChangeRms = boundaryFaces > 0
            ? static_cast<float>( std::sqrt( changedSquared / boundaryFaces ) ) : 0.0f;
    }
}

float fluidField2::advectionUOpen( int x, int y ) const noexcept
{
    if( x <= 0 || x >= width_ || y < 0 || y >= height_ ) return 0.0f;
    if( cutCellProjectionEnabled_ ) return cutCellUOpen( x, y );
    return isSolidCell( x - 1, y ) || isSolidCell( x, y ) ? 0.0f : 1.0f;
}

float fluidField2::advectionVOpen( int x, int y ) const noexcept
{
    if( x < 0 || x >= width_ || y <= 0 || y >= height_ ) return 0.0f;
    if( cutCellProjectionEnabled_ ) return cutCellVOpen( x, y );
    return isSolidCell( x, y - 1 ) || isSolidCell( x, y ) ? 0.0f : 1.0f;
}

vec2 fluidField2::getAdvectionFaceMomentum() const noexcept
{
    double sumU = 0.0;
    double sumV = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            sumU += static_cast<double>( advectionUOpen( x, y ) ) * u_[uIndex( x, y )];
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            sumV += static_cast<double>( advectionVOpen( x, y ) ) * v_[vIndex( x, y )];
    const double mass = static_cast<double>( cellSize_ ) * cellSize_; // rho=1, depth=1m
    return { static_cast<float>( sumU * mass ), static_cast<float>( sumV * mass ) };
}

void fluidField2::advectConservative( float timeStep )
{
    // 고정된 현재 face 개방율에 대해 U/V dual control volume의 flux-form upwind 이류.
    // 공유 dual edge는 양쪽 셀에서 동일한 flux를 사용하며, dual 외곽은 닫힘.
    float speedMax = 0.0f;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( advectionUOpen( x, y ) > 0.0f )
                speedMax = std::max( speedMax, std::abs( u_[uIndex( x, y )] ) );
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( advectionVOpen( x, y ) > 0.0f )
                speedMax = std::max( speedMax, std::abs( v_[vIndex( x, y )] ) );

    const double cfl = static_cast<double>( speedMax ) * timeStep / cellSize_;
    constexpr int maxSubsteps = 64;
    const int substeps = static_cast<int>( std::clamp( std::ceil( cfl / 0.45 ), 1.0,
        static_cast<double>( maxSubsteps ) ) );
    advectionStats_.maxCfl = static_cast<float>( cfl );
    advectionStats_.substeps = substeps;
    advectionStats_.cflLimited = cfl > maxSubsteps * 0.45;
    const float scale = ( timeStep / substeps ) / cellSize_;
    const auto flux = []( float speed, float lower, float upper, float open )
    {
        return open * speed * ( speed >= 0.0f ? lower : upper );
    };

    for( int step = 0; step < substeps; ++step )
    {
        std::copy( u_.begin(), u_.end(), nextU_.begin() );
        std::copy( v_.begin(), v_.end(), nextV_.begin() );
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const float open = advectionUOpen( x, y );
                if( open == 0.0f ) continue;
                const int index = uIndex( x, y );
                const float u = u_[index];
                float east = 0.0f, west = 0.0f, north = 0.0f, south = 0.0f;
                if( x + 1 < width_ )
                {
                    const float next = u_[uIndex( x + 1, y )];
                    east = flux( 0.5f * ( u + next ), u, next,
                        std::min( open, advectionUOpen( x + 1, y ) ) );
                }
                if( x > 1 )
                {
                    const float prev = u_[uIndex( x - 1, y )];
                    west = flux( 0.5f * ( prev + u ), prev, u,
                        std::min( open, advectionUOpen( x - 1, y ) ) );
                }
                if( y + 1 < height_ )
                {
                    const float next = u_[uIndex( x, y + 1 )];
                    north = flux( 0.5f * ( v_[vIndex( x - 1, y + 1 )] + v_[vIndex( x, y + 1 )] ),
                        u, next, std::min( open, advectionUOpen( x, y + 1 ) ) );
                }
                if( y > 0 )
                {
                    const float prev = u_[uIndex( x, y - 1 )];
                    south = flux( 0.5f * ( v_[vIndex( x - 1, y )] + v_[vIndex( x, y )] ),
                        prev, u, std::min( open, advectionUOpen( x, y - 1 ) ) );
                }
                nextU_[index] = u - scale * ( east - west + north - south ) / open;
            }

        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const float open = advectionVOpen( x, y );
                if( open == 0.0f ) continue;
                const int index = vIndex( x, y );
                const float v = v_[index];
                float east = 0.0f, west = 0.0f, north = 0.0f, south = 0.0f;
                if( x + 1 < width_ )
                {
                    const float next = v_[vIndex( x + 1, y )];
                    east = flux( 0.5f * ( u_[uIndex( x + 1, y - 1 )] + u_[uIndex( x + 1, y )] ),
                        v, next, std::min( open, advectionVOpen( x + 1, y ) ) );
                }
                if( x > 0 )
                {
                    const float prev = v_[vIndex( x - 1, y )];
                    west = flux( 0.5f * ( u_[uIndex( x, y - 1 )] + u_[uIndex( x, y )] ),
                        prev, v, std::min( open, advectionVOpen( x - 1, y ) ) );
                }
                if( y + 1 < height_ )
                {
                    const float next = v_[vIndex( x, y + 1 )];
                    north = flux( 0.5f * ( v + next ), v, next,
                        std::min( open, advectionVOpen( x, y + 1 ) ) );
                }
                if( y > 1 )
                {
                    const float prev = v_[vIndex( x, y - 1 )];
                    south = flux( 0.5f * ( prev + v ), prev, v,
                        std::min( open, advectionVOpen( x, y - 1 ) ) );
                }
                nextV_[index] = v - scale * ( east - west + north - south ) / open;
            }
        u_.swap( nextU_ );
        v_.swap( nextV_ );
    }
}


void fluidField2::remapTracerSwept()
{
    // 끝점의 cut-cell 체적 감소를 이송 상한으로, 실제 sampled swept face
    // 통과 체적에 출발 셀의 이전 농도를 곱한 값을 면별 이송 제안량으로 사용.
    // 정확한 공간-시간 계면 재구성은 아니므로 Part 12a 방식은 기본값으로 유지.
    if( !sweptTracerRemapEnabled_ || !cutCellProjectionEnabled_ ||
        !volumeSourceValid_ || !sweptFaceFlux_.usedSweptFlux ) return;

    tracerStats_.usedSweptRemap = true;
    std::copy( tracerMass_.begin(), tracerMass_.end(), nextTracerMass_.begin() );
    const double cellVolume = static_cast<double>( cellSize_ ) * cellSize_; // unit depth, 1m

    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int source = cellIndex( x, y );
            const double previous = previousCellFluidFraction_[source];
            const double current = cellFluidFraction_[source];
            const double availableMass = tracerMass_[source];
            if( previous <= current || previous <= 0.0 || availableMass <= 0.0 ) continue;

            // 사라진 유체 부피 안에 있던 수동 tracer 질량의 snapshot 추정.
            const double concentration = availableMass / ( previous * cellVolume );
            const double displaced = std::min( availableMass,
                concentration * ( previous - current ) * cellVolume );
            if( displaced <= 0.0 ) continue;

            const std::array<int, 4> neighbors{{
                x > 0 ? cellIndex( x - 1, y ) : -1,
                x + 1 < width_ ? cellIndex( x + 1, y ) : -1,
                y > 0 ? cellIndex( x, y - 1 ) : -1,
                y + 1 < height_ ? cellIndex( x, y + 1 ) : -1
            }};
            std::array<double, 4> outward{{
                -static_cast<double>( sweptUVolume_[uIndex( x, y )] ),
                static_cast<double>( sweptUVolume_[uIndex( x + 1, y )] ),
                -static_cast<double>( sweptVVolume_[vIndex( x, y )] ),
                static_cast<double>( sweptVVolume_[vIndex( x, y + 1 )] )
            }};

            // 각 U/V face의 부호를 셀 바깥쪽으로 통일. 목적지가 현재
            // solid라면 현재 유체의 shared-face 수송 경로로 인정하지 않는다.
            double totalOutwardVolume = 0.0;
            for( int direction = 0; direction < 4; ++direction )
            {
                const int neighbor = neighbors[direction];
                if( neighbor < 0 || cellFluidFraction_[neighbor] <= 0.0f ||
                    !std::isfinite( outward[direction] ) )
                    outward[direction] = 0.0;
                else
                    outward[direction] = std::max( 0.0, outward[direction] );
                totalOutwardVolume += outward[direction];
            }

            const double requested = concentration * totalOutwardVolume;
            const double sweptTransfer = std::min( displaced, requested );
            tracerStats_.sweptRequestedMass += static_cast<float>( requested );
            tracerStats_.sweptLimitedMass += static_cast<float>( std::max( 0.0, requested - displaced ) );
            if( sweptTransfer > 0.0 )
            {
                // 공유 swept face가 기록한 체적 비율대로 각 recipient에게
                // 양을 분배한다. 요청량이 축소 체적을 초과하면 함께 축소한다.
                // 모든 원천 계산은 이 Step 시작 질량만 참조하므로 순서 독립.
                double remaining = sweptTransfer;
                int lastDestination = -1;
                for( int direction = 0; direction < 4; ++direction )
                {
                    if( outward[direction] <= 0.0 ) continue;
                    const double proposed = sweptTransfer * outward[direction] / totalOutwardVolume;
                    const double transfer = std::min( remaining, proposed );
                    nextTracerMass_[source] -= transfer;
                    nextTracerMass_[neighbors[direction]] += transfer;
                    tracerStats_.sweptRemappedMass += static_cast<float>( transfer );
                    remaining -= transfer;
                    lastDestination = neighbors[direction];
                }
                if( remaining > 0.0 && lastDestination >= 0 )
                {
                    nextTracerMass_[source] -= remaining;
                    nextTracerMass_[lastDestination] += remaining;
                    tracerStats_.sweptRemappedMass += static_cast<float>( remaining );
                }
            }

            const double unmatched = displaced - sweptTransfer;
            if( unmatched <= 1.0e-12 * std::max( 1.0, displaced ) ) continue;

            // 적분된 swept 체적이 감소한 cut-cell 체적보다 작을 수 있다.
            // 완전 차폐 셀에서는 기존의 가장 가까운 활성 유체로 이동하고,
            // 부분 개방 셀에서는 새로 열린 이웃이 있을 때에만 이동한다.
            // 그 밖에는 질량을 그대로 두고 미해결량을 공개한다.
            int destination = -1;
            if( current <= 0.0 )
                destination = cutCellNearestFluid_[source];
            else
            {
                float largestGain = 0.0f;
                for( const int neighbor : neighbors )
                {
                    if( neighbor < 0 || cellFluidFraction_[neighbor] <= 0.0f ) continue;
                    const float gain = cellFluidFraction_[neighbor] - previousCellFluidFraction_[neighbor];
                    if( gain > largestGain )
                    {
                        largestGain = gain;
                        destination = neighbor;
                    }
                }
            }

            if( destination >= 0 && destination != source )
            {
                nextTracerMass_[source] -= unmatched;
                nextTracerMass_[destination] += unmatched;
                tracerStats_.fallbackRemappedMass += static_cast<float>( unmatched );
            }
            else
                tracerStats_.unresolvedDisplacementMass += static_cast<float>( unmatched );
        }

    tracerMass_.swap( nextTracerMass_ );
}

void fluidField2::transportTracer( float timeStep )
{
    tracerStats_ = {};
    if( !tracerEnabled_ ) return;

    tracerStats_.beforeMass = getTracerMass();
    remapTracerSwept();
    const double area = static_cast<double>( cellSize_ ) * cellSize_;
    double maxSpeed = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( advectionUOpen( x, y ) > 0.0f )
                maxSpeed = std::max( maxSpeed, static_cast<double>( std::abs( u_[uIndex( x, y )] ) ) );
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( advectionVOpen( x, y ) > 0.0f )
                maxSpeed = std::max( maxSpeed, static_cast<double>( std::abs( v_[vIndex( x, y )] ) ) );

    const double cfl = maxSpeed * timeStep / cellSize_;
    constexpr int maxSubsteps = 64;
    const int substeps = static_cast<int>( std::clamp( std::ceil( cfl / 0.45 ), 1.0,
        static_cast<double>( maxSubsteps ) ) );
    tracerStats_.maxCfl = static_cast<float>( cfl );
    tracerStats_.substeps = substeps;
    tracerStats_.cflLimited = cfl > maxSubsteps * 0.45;
    const double lengthTime = static_cast<double>( timeStep ) * cellSize_ / substeps;

    // 셀 tracer 질량 = 농도 * 현재 유체 비율 * h². 각 공유 MAC 면의
    // signed upwind 유량은 딱 한 번 계산하고, donor에서만 질량을 뺀다.
    // 출구가 여럿일 때 동시 유출량을 donor 질량으로 제한해 음수를 방지한다.
    for( int substep = 0; substep < substeps; ++substep )
    {
        std::fill( tracerOutgoing_.begin(), tracerOutgoing_.end(), 0.0 );
        std::fill( tracerUFlux_.begin(), tracerUFlux_.end(), 0.0 );
        std::fill( tracerVFlux_.begin(), tracerVFlux_.end(), 0.0 );
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const float open = advectionUOpen( x, y );
                if( open <= 0.0f ) continue;
                const int left = cellIndex( x - 1, y );
                const int right = cellIndex( x, y );
                const double velocity = u_[uIndex( x, y )];
                const int donor = velocity >= 0.0 ? left : right;
                const float fraction = tracerCellFraction( velocity >= 0.0 ? x - 1 : x, y );
                if( fraction <= 0.0f ) continue;
                const double concentration = tracerMass_[donor] / ( area * fraction );
                const double flux = velocity * open * lengthTime * concentration;
                tracerUFlux_[uIndex( x, y )] = flux;
                tracerOutgoing_[donor] += std::abs( flux );
            }
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const float open = advectionVOpen( x, y );
                if( open <= 0.0f ) continue;
                const int bottom = cellIndex( x, y - 1 );
                const int top = cellIndex( x, y );
                const double velocity = v_[vIndex( x, y )];
                const int donor = velocity >= 0.0 ? bottom : top;
                const float fraction = tracerCellFraction( x, velocity >= 0.0 ? y - 1 : y );
                if( fraction <= 0.0f ) continue;
                const double concentration = tracerMass_[donor] / ( area * fraction );
                const double flux = velocity * open * lengthTime * concentration;
                tracerVFlux_[vIndex( x, y )] = flux;
                tracerOutgoing_[donor] += std::abs( flux );
            }

        std::copy( tracerMass_.begin(), tracerMass_.end(), nextTracerMass_.begin() );
        const auto limitedFlux = [&]( double flux, int donor )
        {
            const double outgoing = tracerOutgoing_[donor];
            return outgoing > tracerMass_[donor] && outgoing > 0.0
                ? flux * ( tracerMass_[donor] / outgoing ) : flux;
        };
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const int left = cellIndex( x - 1, y );
                const int right = cellIndex( x, y );
                const double flux = tracerUFlux_[uIndex( x, y )];
                const double transfer = limitedFlux( flux, flux >= 0.0 ? left : right );
                nextTracerMass_[left] -= transfer;
                nextTracerMass_[right] += transfer;
            }
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int bottom = cellIndex( x, y - 1 );
                const int top = cellIndex( x, y );
                const double flux = tracerVFlux_[vIndex( x, y )];
                const double transfer = limitedFlux( flux, flux >= 0.0 ? bottom : top );
                nextTracerMass_[bottom] -= transfer;
                nextTracerMass_[top] += transfer;
            }
        tracerMass_.swap( nextTracerMass_ );
    }

    double trapped = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( tracerCellFraction( x, y ) <= 0.0f )
                trapped += tracerMass_[cellIndex( x, y )];
    tracerStats_.trappedMass = static_cast<float>( trapped );
    tracerStats_.afterMass = getTracerMass();
}

void fluidField2::step( float timeStep, std::span<const fluidCircleObstacle2> obstacles,
    std::span<fluidObstacleForce2> feedback, int pressureIterations )
{
    if( feedback.size() != obstacles.size() )
        throw std::invalid_argument( "fluidField2: obstacle and feedback sizes must match" );

    // 기존 원 전용 API는 호환 유지. 단일 solver는 공통 Shape API만 수행.
    std::vector<fluidShapeObstacle2> shapes;
    shapes.reserve( obstacles.size() );
    for( const fluidCircleObstacle2& obstacle : obstacles )
        shapes.push_back( { circle2{ {}, obstacle.radius }, { obstacle.center, {} }, obstacle.center,
            obstacle.linearVelocity, obstacle.angularVelocity } );
    step( timeStep, std::span<const fluidShapeObstacle2>( shapes ), feedback, pressureIterations );
}

void fluidField2::step( float timeStep, std::span<const fluidShapeObstacle2> obstacles,
    std::span<fluidObstacleForce2> feedback, int pressureIterations )
{
    if( feedback.size() != obstacles.size() )
        throw std::invalid_argument( "fluidField2: obstacle and feedback sizes must match" );
    std::fill( feedback.begin(), feedback.end(), fluidObstacleForce2{} );
    if( !std::isfinite( timeStep ) || timeStep <= 0.0f ) return;

    stepMomentum_.before = getFluidMomentum();
    updateObstacles( obstacles );
    updateCutCellGeometry( obstacles );
    if( cutCellProjectionEnabled_ )
    {
        updateSweptSolidFlux( timeStep, obstacles );
        reconcileSweptSolidFlux();
    }
    else sweptFaceFlux_ = {};
    remapExposedFaces();
    stepMomentum_.afterObstacles = getFluidMomentum();
    if( cutCellProjectionEnabled_ ) applyCutCellBoundaries();
    else applySolidBoundaries( obstacles );

    advectionStats_ = {};
    advectionStats_.conservative = conservativeAdvectionEnabled_;
    advectionStats_.before = getAdvectionFaceMomentum();
    if( conservativeAdvectionEnabled_ ) advectConservative( timeStep );
    else
    {
        // 기존 반 라그랑주 이류: 읽기 전용 source로부터 face별 역추적.
        std::fill( nextU_.begin(), nextU_.end(), 0.0f );
    std::fill( nextV_.begin(), nextV_.end(), 0.0f );
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
            const vec2 previous = point - timeStep * sampleFrom( u_, v_, point );
            nextU_[uIndex( x, y )] = sampleFrom( u_, v_, previous ).x;
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
            const vec2 previous = point - timeStep * sampleFrom( u_, v_, point );
            nextV_[vIndex( x, y )] = sampleFrom( u_, v_, previous ).y;
        }

        u_.swap( nextU_ );
        v_.swap( nextV_ );
        advectionStats_.substeps = 1;
    }
    if( cutCellProjectionEnabled_ ) applyCutCellBoundaries();
    else applySolidBoundaries( obstacles );
    advectionStats_.after = getAdvectionFaceMomentum();
    stepMomentum_.afterAdvection = getFluidMomentum();
    if( cutCellProjectionEnabled_ )
    {
        pressureMomentum_ = {}; // 이전 이진 MAC face impulse ledger와 단위가 다르므로 혼합하지 않음.
        projectCutCell( timeStep, std::clamp( pressureIterations, 1, 200 ), obstacles, feedback );
    }
    else
    {
        cutCellPressure_ = {};
        project( timeStep, std::clamp( pressureIterations, 1, 200 ), obstacles, feedback );
    }
    stepMomentum_.afterPressure = getFluidMomentum();
    transportTracer( timeStep );
    previousObstacles_.assign( obstacles.begin(), obstacles.end() );
}

float fluidField2::cutCellUOpen( int x, int y ) const noexcept
{
    if( x <= 0 || x >= width_ || y < 0 || y >= height_ ) return 0.0f;
    if( cellFluidFraction_[cellIndex( x - 1, y )] == 0.0f ||
        cellFluidFraction_[cellIndex( x, y )] == 0.0f ) return 0.0f;
    return uOpenFraction_[uIndex( x, y )];
}

float fluidField2::cutCellVOpen( int x, int y ) const noexcept
{
    if( x < 0 || x >= width_ || y <= 0 || y >= height_ ) return 0.0f;
    if( cellFluidFraction_[cellIndex( x, y - 1 )] == 0.0f ||
        cellFluidFraction_[cellIndex( x, y )] == 0.0f ) return 0.0f;
    return vOpenFraction_[vIndex( x, y )];
}

void fluidField2::applyCutCellBoundaries() noexcept
{
    // 완전히 막힌 face에는 강체 법선 속도를 둔다. 일부 열린 face의 u/v는
    // 유체 성분 속도이며, 막힌 구간의 강체 속도는 별도 flux 항에만 사용한다.
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( cutCellUOpen( x, y ) == 0.0f ) u_[uIndex( x, y )] = uSolidVelocity_[uIndex( x, y )];
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( cutCellVOpen( x, y ) == 0.0f ) v_[vIndex( x, y )] = vSolidVelocity_[vIndex( x, y )];
}

void fluidField2::synchronizeCutCellBoundaries( float timeStep,
    std::span<const fluidShapeObstacle2> obstacles, std::span<const fluidObstacleForce2> feedback )
{
    // updateCutCellGeometry와 동일한 sub-face 표본 / 첫 점유 Shape 규칙으로 재계산한다.
    // fluid 면의 속도 u_/v_는 보존하고 막힌 비율의 solid 속도만 예측값으로 갱신.
    const int sampleCount = cutCellSampleCount_;
    const float inverseCount = 1.0f / sampleCount;
    struct predictedBody
    {
        vec2 linear{};
        float angular = 0.0f;
    };
    std::fill( pressureUSolidFlux_.begin(), pressureUSolidFlux_.end(), 0.0f );
    std::fill( pressureVSolidFlux_.begin(), pressureVSolidFlux_.end(), 0.0f );
    std::vector<predictedBody> predicted( obstacles.size() );
    std::vector<double> bounds( obstacles.size(), -1.0 );
    for( std::size_t i = 0; i < obstacles.size(); ++i )
    {
        const fluidShapeObstacle2& body = obstacles[i];
        predicted[i].linear = body.linearVelocity + ( timeStep * body.inverseMass ) * feedback[i].force;
        predicted[i].angular = body.angularVelocity + timeStep * body.inverseInertia * feedback[i].torque;
        if( IsValidFluidObstacle( body ) ) bounds[i] = FluidObstacleBoundRadius( body );
    }
    const auto firstOwner = [&]( vec2 point )
    {
        for( std::size_t i = 0; i < obstacles.size(); ++i )
        {
            if( bounds[i] < 0.0 ) continue;
            const vec2 center = obstacles[i].transform.position;
            if( std::abs( static_cast<double>( point.x ) - center.x ) > bounds[i] ||
                std::abs( static_cast<double>( point.y ) - center.y ) > bounds[i] ) continue;
            if( ContainsFluidObstacle( obstacles[i], point ) ) return static_cast<int>( i );
        }
        return -1;
    };

    double changedSquared = 0.0;
    int boundaryFaces = 0;
    int partialFaces = 0;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const int face = uIndex( x, y );
            if( uOpenFraction_[face] >= 1.0f ) continue;
            float sum = 0.0f;
            int blocked = 0;
            for( int sample = 0; sample < sampleCount; ++sample )
            {
                const vec2 point = origin_ + cellSize_ * vec2{
                    static_cast<float>( x ), static_cast<float>( y ) + ( sample + 0.5f ) * inverseCount };
                const int owner = firstOwner( point );
                if( owner < 0 ) continue;
                const predictedBody& body = predicted[static_cast<std::size_t>( owner )];
                sum += ( body.linear + Cross( body.angular, point - obstacles[owner].center ) ).x;
                ++blocked;
            }
            if( blocked == 0 ) continue;
            const float next = sum / blocked;
            const double change = static_cast<double>( next ) - uSolidVelocity_[face];
            changedSquared += change * change;
            pressureUSolidFlux_[face] = ( 1.0f - uOpenFraction_[face] ) * static_cast<float>( change );
            uSolidVelocity_[face] = next;
            ++boundaryFaces;
            if( uOpenFraction_[face] > 0.0f ) ++partialFaces;
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int face = vIndex( x, y );
            if( vOpenFraction_[face] >= 1.0f ) continue;
            float sum = 0.0f;
            int blocked = 0;
            for( int sample = 0; sample < sampleCount; ++sample )
            {
                const vec2 point = origin_ + cellSize_ * vec2{
                    static_cast<float>( x ) + ( sample + 0.5f ) * inverseCount, static_cast<float>( y ) };
                const int owner = firstOwner( point );
                if( owner < 0 ) continue;
                const predictedBody& body = predicted[static_cast<std::size_t>( owner )];
                sum += ( body.linear + Cross( body.angular, point - obstacles[owner].center ) ).y;
                ++blocked;
            }
            if( blocked == 0 ) continue;
            const float next = sum / blocked;
            const double change = static_cast<double>( next ) - vSolidVelocity_[face];
            changedSquared += change * change;
            pressureVSolidFlux_[face] = ( 1.0f - vOpenFraction_[face] ) * static_cast<float>( change );
            vSolidVelocity_[face] = next;
            ++boundaryFaces;
            if( vOpenFraction_[face] > 0.0f ) ++partialFaces;
        }

    // 완전히 막힌 MAC 면만 u_/v_로 복사한다. 부분 개방 면의 유체 성분은 그대로 둔다.
    applyCutCellBoundaries();
    cutCellPressure_.synchronizedBoundaryFaces = boundaryFaces;
    cutCellPressure_.synchronizedPartialFaces = partialFaces;
    cutCellPressure_.boundaryNormalChangeRms = boundaryFaces > 0
        ? static_cast<float>( std::sqrt( changedSquared / boundaryFaces ) ) : 0.0f;
}

float fluidField2::cutCellFluxDivergenceRms( float inverseTimeStep ) const noexcept
{
    double sum = 0.0;
    int active = 0;
    const float inverseCell = 1.0f / cellSize_;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            if( cellFluidFraction_[cell] <= 0.0f ) continue;
            const float flux = (
                cutCellUOpen( x + 1, y ) * u_[uIndex( x + 1, y )] -
                cutCellUOpen( x, y ) * u_[uIndex( x, y )] +
                cutCellVOpen( x, y + 1 ) * v_[vIndex( x, y + 1 )] -
                cutCellVOpen( x, y ) * v_[vIndex( x, y )] ) * inverseCell;
            const double residual = static_cast<double>( flux ) +
                static_cast<double>( cutCellVolumeRate_[cell] ) * inverseTimeStep;
            sum += residual * residual;
            ++active;
        }
    return active > 0 ? static_cast<float>( std::sqrt( sum / active ) ) : 0.0f;
}

void fluidField2::updateSweptSolidFlux( float timeStep, std::span<const fluidShapeObstacle2> obstacles )
{
    sweptFaceFlux_ = {};
    std::fill( sweptUVolume_.begin(), sweptUVolume_.end(), 0.0f );
    std::fill( sweptVVolume_.begin(), sweptVVolume_.end(), 0.0f );

    // 인덱스가 유지되고 기하가 같으며 도메인 바깥을 가로지르지 않은 shape만 추적.
    if( !volumeSourceValid_ || obstacles.empty() || previousObstacles_.size() != obstacles.size() ) return;

    const float limitX = origin_.x + width_ * cellSize_;
    const float limitY = origin_.y + height_ * cellSize_;
    std::vector<vec2> linearSpeeds( obstacles.size() );
    std::vector<float> startAngles( obstacles.size() );
    std::vector<float> angularSpeeds( obstacles.size() );
    std::vector<double> bounds( obstacles.size() );
    float maximumTravel = 0.0f;
    for( std::size_t i = 0; i < obstacles.size(); ++i )
    {
        const auto& start = previousObstacles_[i];
        const auto& finish = obstacles[i];
        if( !IsValidFluidObstacle( start ) || !IsValidFluidObstacle( finish ) ||
            !HasSameFluidGeometry( start.geometry, finish.geometry ) ) return;
        const float radius = FluidGeometryRadius( finish.geometry );
        if( !std::isfinite( radius ) || radius < 0.0f ) return;
        for( const fluidShapeObstacle2* shape : { &start, &finish } )
        {
            const vec2 p = shape->transform.position;
            if( p.x - radius <= origin_.x || p.x + radius >= limitX ||
                p.y - radius <= origin_.y || p.y + radius >= limitY ) return;
        }

        const vec2 displacement = finish.transform.position - start.transform.position;
        linearSpeeds[i] = displacement / timeStep;
        const float startAngle = std::atan2( start.transform.rotation.s, start.transform.rotation.c );
        const float endAngle = std::atan2( finish.transform.rotation.s, finish.transform.rotation.c );
        const float deltaAngle = std::remainder( endAngle - startAngle, 2.0f * std::numbers::pi_v<float> );
        startAngles[i] = startAngle;
        angularSpeeds[i] = deltaAngle / timeStep;
        bounds[i] = FluidObstacleBoundRadius( finish );
        maximumTravel = std::max( maximumTravel, Length( displacement ) + std::abs( deltaAngle ) * radius );
    }

    sweptFaceFlux_.usedSweptFlux = true;
    // 정적 장애물이 많은 장면에서 매 step 전 격자·4 sub-face를 검사할 필요가 없다.
    // 유효한 동일 geometry가 정지했다면 swept flux는 정확히 0이다.
    if( maximumTravel == 0.0f )
    {
        sweptFaceFlux_.timeSamples = 1;
        return;
    }

    const int faceSamples = cutCellSampleCount_;
    constexpr int maxTimeSamples = 64;
    int timeSamples = std::clamp(
        static_cast<int>( std::ceil( maximumTravel / ( 0.25f * cellSize_ ) ) ), 1, 32 );
    std::vector<fluidShapeObstacle2> intermediate( obstacles.begin(), obstacles.end() );

    const auto integrate = [&]( int samples, std::vector<float>& sweptU, std::vector<float>& sweptV )
    {
        std::fill( sweptU.begin(), sweptU.end(), 0.0f );
        std::fill( sweptV.begin(), sweptV.end(), 0.0f );
        const float weight = timeStep * cellSize_ / ( samples * faceSamples );
        for( int step = 0; step < samples; ++step )
        {
            const float alpha = ( step + 0.5f ) / samples;
            for( std::size_t i = 0; i < obstacles.size(); ++i )
            {
                const auto& start = previousObstacles_[i];
                const auto& finish = obstacles[i];
                intermediate[i].transform.position =
                    start.transform.position + alpha * ( finish.transform.position - start.transform.position );
                intermediate[i].transform.rotation =
                    rot2::FromRadians( startAngles[i] + angularSpeeds[i] * timeStep * alpha );
            }

            const auto normalSpeed = [&]( vec2 point, bool horizontal )
            {
                for( std::size_t i = 0; i < intermediate.size(); ++i )
                {
                    const vec2 center = intermediate[i].transform.position;
                    // 미리 계산한 보수적 geometry radius: 원거리 static obstacle Contains 검사 생략.
                    if( std::abs( static_cast<double>( point.x ) - center.x ) > bounds[i] ||
                        std::abs( static_cast<double>( point.y ) - center.y ) > bounds[i] ) continue;
                    if( !ContainsFluidObstacle( intermediate[i], point ) ) continue;
                    const vec2 velocity = linearSpeeds[i] +
                        Cross( angularSpeeds[i], point - center );
                    return horizontal ? velocity.x : velocity.y;
                }
                return 0.0f;
            };
            for( int y = 0; y < height_; ++y )
                for( int x = 1; x < width_; ++x )
                    for( int sample = 0; sample < faceSamples; ++sample )
                    {
                        const vec2 point = origin_ + cellSize_ * vec2{
                            static_cast<float>( x ), static_cast<float>( y ) + ( sample + 0.5f ) / faceSamples };
                        sweptU[uIndex( x, y )] += weight * normalSpeed( point, true );
                    }
            for( int y = 1; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                    for( int sample = 0; sample < faceSamples; ++sample )
                    {
                        const vec2 point = origin_ + cellSize_ * vec2{
                            static_cast<float>( x ) + ( sample + 0.5f ) / faceSamples, static_cast<float>( y ) };
                        sweptV[vIndex( x, y )] += weight * normalSpeed( point, false );
                    }
        }
    };

    integrate( timeSamples, sweptUVolume_, sweptVVolume_ );
    if( adaptiveSweptSamplingEnabled_ )
    {
        // 스냅샷의 sub-cell ΔV와 swept face divergence가 최대한 일치하는
        // 표본만 채택한다. face마다 임의 보정하지 않고 공유 face 유량을 유지.
        const double area = static_cast<double>( cellSize_ ) * cellSize_;
        const auto mismatch = [&]( const std::vector<float>& sweptU, const std::vector<float>& sweptV )
        {
            double sum = 0.0;
            for( int y = 0; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const int cell = cellIndex( x, y );
                    const double snapshot =
                        static_cast<double>( cellFluidFraction_[cell] - previousCellFluidFraction_[cell] ) * area;
                    const double flux =
                        static_cast<double>( sweptU[uIndex( x + 1, y )] ) - sweptU[uIndex( x, y )] +
                        static_cast<double>( sweptV[vIndex( x, y + 1 )] ) - sweptV[vIndex( x, y )];
                    sum += std::abs( snapshot - flux );
                }
            return sum;
        };

        double bestMismatch = mismatch( sweptUVolume_, sweptVVolume_ );
        sweptFaceFlux_.initialTimeSamples = timeSamples;
        sweptFaceFlux_.initialGeometryMismatch = static_cast<float>( bestMismatch );
        if( bestMismatch > 1.0e-7 )
        {
            std::vector<float> trialU( sweptUVolume_.size() );
            std::vector<float> trialV( sweptVVolume_.size() );
            // 적응 검사는 opt-in에서만 수행. 개선이 멈추면 더 비싼 샘플을 실행하지 않는다.
            for( int refine = 0; refine < 3 && timeSamples < maxTimeSamples; ++refine )
            {
                const int candidateSamples = std::min( 2 * timeSamples, maxTimeSamples );
                integrate( candidateSamples, trialU, trialV );
                const double candidateMismatch = mismatch( trialU, trialV );
                const double minimumImprovement = std::max( 1.0e-7, bestMismatch * 1.0e-4 );
                if( candidateMismatch + minimumImprovement >= bestMismatch ) break;
                sweptUVolume_.swap( trialU );
                sweptVVolume_.swap( trialV );
                bestMismatch = candidateMismatch;
                timeSamples = candidateSamples;
                ++sweptFaceFlux_.refinements;
            }
        }
    }

    double absoluteFlux = 0.0;
    for( const float flux : sweptUVolume_ ) absoluteFlux += std::abs( flux );
    for( const float flux : sweptVVolume_ ) absoluteFlux += std::abs( flux );
    sweptFaceFlux_.timeSamples = timeSamples;
    sweptFaceFlux_.totalAbsoluteFlux = static_cast<float>( absoluteFlux );
}

void fluidField2::reconcileSweptSolidFlux()
{
    // 기본 Cut-cell 경로에 잔차 vector 할당/추가 순회를 발생시키지 않는다.
    // 보정 미사용 시 raw 값은 prepareCutCellVolumeChange의 최종 관측값과 같다.
    if( !sweptFaceFlux_.usedSweptFlux || !sweptGeometryCorrectionEnabled_ ) return;

    // 순수 기하 discrepancy: 시작/종료 점유율 변화와 경로 적분된 공유 면
    // 체적의 divergence 간 차이. 외벽의 swept flux는 항상 0.
    const double area = static_cast<double>( cellSize_ ) * cellSize_;
    std::vector<double> residual( cellFluidFraction_.size() );
    double mismatch = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            const double snapshot = static_cast<double>(
                cellFluidFraction_[cell] - previousCellFluidFraction_[cell] ) * area;
            const double swept =
                static_cast<double>( sweptUVolume_[uIndex( x + 1, y )] ) -
                static_cast<double>( sweptUVolume_[uIndex( x, y )] ) +
                static_cast<double>( sweptVVolume_[vIndex( x, y + 1 )] ) -
                static_cast<double>( sweptVVolume_[vIndex( x, y )] );
            residual[cell] = snapshot - swept;
            mismatch += std::abs( residual[cell] );
        }
    sweptFaceFlux_.rawGeometryMismatch = static_cast<float>( mismatch );
    // 서로 부호가 반대인 이웃의 discrepancy만 공유 edge에서 국소 상쇄.
    // 각 update는 두 셀의 잔차를 같은 크기·반대 부호로 변경해
    // 전체 체적 부호합 및 닫힌 외벽을 보존하고 L1 오차를 증가시키지 않는다.
    // 더 먼 이웃까지 강제로 전달하거나 total ΔV의 비영(非零) 합을 지우지 않는다.
    const auto reconcile = [&]( int left, int right, float& face )
    {
        const double a = residual[left];
        const double b = residual[right];
        double delta = 0.0;
        if( a > 0.0 && b < 0.0 ) delta = std::min( a, -b );
        else if( a < 0.0 && b > 0.0 ) delta = -std::min( -a, b );
        if( delta == 0.0 ) return;

        const double before = face;
        double next = before + delta;
        // 원래 측정된 면 속도를 정반대로 뒤집지 않음.
        if( before != 0.0 && ( before > 0.0 ) != ( next > 0.0 ) ) next = 0.0;
        if( !std::isfinite( next ) || std::abs( next ) > std::numeric_limits<float>::max() ) return;
        face = static_cast<float>( next );
        const double applied = static_cast<double>( face ) - before;
        if( applied == 0.0 ) return;
        residual[left] -= applied;
        residual[right] += applied;
        sweptFaceFlux_.correctionAbsoluteFlux += static_cast<float>( std::abs( applied ) );
        ++sweptFaceFlux_.correctedFaces;
    };

    // 변한 셀에 인접하거나 원래 swept flux가 있는 면만 보정한다.
    // 먼 정적 영역을 통과하는 전역적 보정 흐름은 생성하지 않는다.
    constexpr int passes = 4;
    for( int pass = 0; pass < passes; ++pass )
    {
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const int left = cellIndex( x - 1, y );
                const int right = cellIndex( x, y );
                float& face = sweptUVolume_[uIndex( x, y )];
                if( face == 0.0f &&
                    cellFluidFraction_[left] == previousCellFluidFraction_[left] &&
                    cellFluidFraction_[right] == previousCellFluidFraction_[right] ) continue;
                reconcile( left, right, face );
            }
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int bottom = cellIndex( x, y - 1 );
                const int top = cellIndex( x, y );
                float& face = sweptVVolume_[vIndex( x, y )];
                if( face == 0.0f &&
                    cellFluidFraction_[bottom] == previousCellFluidFraction_[bottom] &&
                    cellFluidFraction_[top] == previousCellFluidFraction_[top] ) continue;
                reconcile( bottom, top, face );
            }
    }

    // final geometryMismatch는 기존 prepareCutCellVolumeChange가 새 face를
    // 이용해 계산한다. rawGeometryMismatch와 직접 비교할 수 있다.
}

void fluidField2::prepareCutCellVolumeChange( float timeStep )
{
    std::fill( cutCellVolumeRate_.begin(), cutCellVolumeRate_.end(), 0.0f );
    cutCellPressure_.hasVolumeHistory = volumeSourceValid_;
    if( !volumeSourceValid_ ) return;

    // Δfraction = (new fluid area - old fluid area) / h². 최초 배치에는 적용하지 않는다.
    // 현재 완전히 solid가 된 cell에는 압력 미지수가 없으므로 가장 가까운 활성
    // fluid cell로 해당 Δfraction을 이송한다. 이는 swept 경로가 아닌 공간 근사다.
    cutCellQueue_.clear();
    std::fill( cutCellNearestFluid_.begin(), cutCellNearestFluid_.end(), -1 );
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int index = cellIndex( x, y );
            if( cellFluidFraction_[index] <= 0.0f ) continue;
            cutCellNearestFluid_[index] = index;
            cutCellQueue_.push_back( index );
        }

    for( std::size_t read = 0; read < cutCellQueue_.size(); ++read )
    {
        const int index = cutCellQueue_[read];
        const int x = index % width_;
        const int y = index / width_;
        const auto visit = [&]( int neighbor )
        {
            if( cutCellNearestFluid_[neighbor] >= 0 ) return;
            cutCellNearestFluid_[neighbor] = cutCellNearestFluid_[index];
            cutCellQueue_.push_back( neighbor );
        };
        if( x > 0 ) visit( index - 1 );
        if( x + 1 < width_ ) visit( index + 1 );
        if( y > 0 ) visit( index - width_ );
        if( y + 1 < height_ ) visit( index + width_ );
    }

    double volumeRate = 0.0;
    double relocatedRate = 0.0;
    double unresolvedRate = 0.0;
    const double scale = static_cast<double>( cellSize_ ) * cellSize_ / timeStep;
    for( std::size_t i = 0; i < cellFluidFraction_.size(); ++i )
    {
        const float snapshotChange = cellFluidFraction_[i] - previousCellFluidFraction_[i];
        volumeRate += static_cast<double>( snapshotChange ) * scale;
        const int x = static_cast<int>( i ) % width_;
        const int y = static_cast<int>( i ) / width_;
        const float change = sweptFaceFlux_.usedSweptFlux
            ? ( sweptUVolume_[uIndex( x + 1, y )] - sweptUVolume_[uIndex( x, y )] +
                sweptVVolume_[vIndex( x, y + 1 )] - sweptVVolume_[vIndex( x, y )] ) /
                ( cellSize_ * cellSize_ )
            : snapshotChange;
        if( sweptFaceFlux_.usedSweptFlux )
            sweptFaceFlux_.geometryMismatch += std::abs( change - snapshotChange ) * cellSize_ * cellSize_;
        if( cellFluidFraction_[i] > 0.0f )
            cutCellVolumeRate_[i] += change;
        else if( change != 0.0f )
        {
            const int destination = cutCellNearestFluid_[i];
            if( destination >= 0 )
            {
                cutCellVolumeRate_[destination] += change;
                relocatedRate += static_cast<double>( change ) * scale;
            }
            else unresolvedRate += static_cast<double>( change ) * scale;
        }
    }
    if( sweptFaceFlux_.usedSweptFlux && !sweptGeometryCorrectionEnabled_ )
        sweptFaceFlux_.rawGeometryMismatch = sweptFaceFlux_.geometryMismatch;
    cutCellPressure_.volumeRate = static_cast<float>( volumeRate );
    cutCellPressure_.redistributedVolumeRate = static_cast<float>( relocatedRate );
    cutCellPressure_.unresolvedVolumeRate = static_cast<float>( unresolvedRate );
}

void fluidField2::projectCutCell( float timeStep, int pressureIterations,
    std::span<const fluidShapeObstacle2> obstacles, std::span<fluidObstacleForce2> feedback )
{
    // 유체 점유 면적의 시간 변화는 다음 보존형 연속 방정식에 직접 들어간다:
    // div(a*u_fluid) + ΔfluidFraction / dt = 0.
    // 차단된 face의 solid 속도를 여기에 다시 더하면 경계 이동을 이중 계상하므로
    // face의 유체 개방율 a만 압력 gradient/flux에 적용한다.
    const float inverseCell = 1.0f / cellSize_;
    const float pressureScale = cellSize_ * cellSize_ / timeStep;
    const float gradientScale = timeStep * inverseCell;
    cutCellPressure_ = {};
    prepareCutCellVolumeChange( timeStep );
    cutCellPressure_.divergenceBefore = cutCellFluxDivergenceRms( 1.0f / timeStep );
    double sumDivergence = 0.0;

    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            if( cellFluidFraction_[cell] <= 0.0f )
            {
                divergence_[cell] = 0.0f;
                continue;
            }
            const auto fluxU = [&]( int i ) { return cutCellUOpen( i, y ) * u_[uIndex( i, y )]; };
            const auto fluxV = [&]( int j ) { return cutCellVOpen( x, j ) * v_[vIndex( x, j )]; };
            divergence_[cell] = ( fluxU( x + 1 ) - fluxU( x ) +
                fluxV( y + 1 ) - fluxV( y ) ) * inverseCell +
                cutCellVolumeRate_[cell] / timeStep;
            sumDivergence += divergence_[cell];
            ++cutCellPressure_.activeCells;
        }

    // 닫힌 외벽과 고체 사이의 압력 Poisson은 각 연결 영역마다 Neumann 문제다.
    // 전체 평균만 빼면 서로 분리된 유체 포켓의 독립적인 질량 불일치를 풀 수 없다.
    // 개방 face로 연결된 영역별 source 평균을 빼고 그 크기를 진단값으로 남긴다.
    cutCellPressure_.sourceImbalance =
        static_cast<float>( sumDivergence * cellSize_ * cellSize_ ) + cutCellPressure_.unresolvedVolumeRate;
    std::fill( cutCellComponent_.begin(), cutCellComponent_.end(), -1 );
    std::vector<double> regionSums;
    std::vector<int> regionSizes;
    cutCellQueue_.clear();
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int start = cellIndex( x, y );
            if( cellFluidFraction_[start] <= 0.0f || cutCellComponent_[start] >= 0 ) continue;
            const int region = static_cast<int>( regionSums.size() );
            regionSums.push_back( 0.0 );
            regionSizes.push_back( 0 );
            cutCellQueue_.clear();
            cutCellQueue_.push_back( start );
            cutCellComponent_[start] = region;

            for( std::size_t read = 0; read < cutCellQueue_.size(); ++read )
            {
                const int index = cutCellQueue_[read];
                const int cx = index % width_;
                const int cy = index / width_;
                regionSums[region] += divergence_[index];
                ++regionSizes[region];
                const auto visit = [&]( int neighbor, float opening )
                {
                    if( opening <= 0.0f || cutCellComponent_[neighbor] >= 0 ) return;
                    cutCellComponent_[neighbor] = region;
                    cutCellQueue_.push_back( neighbor );
                };
                if( cx > 0 ) visit( index - 1, cutCellUOpen( cx, cy ) );
                if( cx + 1 < width_ ) visit( index + 1, cutCellUOpen( cx + 1, cy ) );
                if( cy > 0 ) visit( index - width_, cutCellVOpen( cx, cy ) );
                if( cy + 1 < height_ ) visit( index + width_, cutCellVOpen( cx, cy + 1 ) );
            }
        }
    cutCellPressure_.connectedRegions = static_cast<int>( regionSums.size() );
    double incompatible = std::abs( static_cast<double>( cutCellPressure_.unresolvedVolumeRate ) );
    for( const double sum : regionSums ) incompatible += std::abs( sum ) * cellSize_ * cellSize_;
    cutCellPressure_.compatibilityCorrection = static_cast<float>( incompatible );

    for( std::size_t i = 0; i < divergence_.size(); ++i )
    {
        const int region = cutCellComponent_[i];
        if( region >= 0 )
            divergence_[i] -= static_cast<float>( regionSums[region] / regionSizes[region] );
    }

    // Cut-cell + 질량 결합은 별도 선택 경로. PCG가 실패하면 명시적 Jacobi
    // 압력 반력을 Dynamic Body에 적용하지 않고 해당 Step을 격리한다.
    const bool coupled = cutCellCoupledProjectionEnabled_ &&
        std::any_of( obstacles.begin(), obstacles.end(), []( const fluidShapeObstacle2& obstacle )
        {
            return obstacle.inverseMass > 0.0f || obstacle.inverseInertia > 0.0f;
        } );
    const bool solvedCoupled = coupled && solveCoupledPressure( timeStep, pressureIterations, obstacles, true );
    cutCellPressure_.usedCoupledProjection = solvedCoupled;
    cutCellPressure_.coupledStatus = coupled ? pressureMomentum_.coupledStatus :
        fluidCoupledPressureStatus2::notRequested;
    cutCellPressure_.coupledIterations = coupled ? pressureMomentum_.coupledIterations : 0;
    cutCellPressure_.coupledRelativeResidual = coupled ? pressureMomentum_.coupledRelativeResidual : 0.0f;
    cutCellPressure_.coupledSolveFailed = coupled && !solvedCoupled &&
        cutCellPressure_.coupledStatus != fluidCoupledPressureStatus2::noBoundary;
    const float coupledPressureResidual = pressureMomentum_.coupledFinalResidualRms;
    pressureMomentum_ = {}; // 이진 MAC 압력 운동량 장부를 Cut-cell의 반력과 혼합하지 않는다.

    if( cutCellPressure_.coupledSolveFailed )
        std::fill( pressure_.begin(), pressure_.end(), 0.0f );
    else if( !solvedCoupled )
    {
        std::fill( pressure_.begin(), pressure_.end(), 0.0f );
        for( int iteration = 0; iteration < pressureIterations; ++iteration )
        {
            for( int y = 0; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const int cell = cellIndex( x, y );
                    if( cellFluidFraction_[cell] <= 0.0f ) { nextPressure_[cell] = 0.0f; continue; }
                    const float left = cutCellUOpen( x, y );
                    const float right = cutCellUOpen( x + 1, y );
                    const float bottom = cutCellVOpen( x, y );
                    const float top = cutCellVOpen( x, y + 1 );
                    const float diagonal = left + right + bottom + top;
                    const float sum =
                        ( left > 0.0f ? left * pressure_[cellIndex( x - 1, y )] : 0.0f ) +
                        ( right > 0.0f ? right * pressure_[cellIndex( x + 1, y )] : 0.0f ) +
                        ( bottom > 0.0f ? bottom * pressure_[cellIndex( x, y - 1 )] : 0.0f ) +
                        ( top > 0.0f ? top * pressure_[cellIndex( x, y + 1 )] : 0.0f );
                    nextPressure_[cell] = diagonal > 0.0f
                        ? ( sum - divergence_[cell] * pressureScale ) / diagonal : 0.0f;
                }
            pressure_.swap( nextPressure_ );
        }
    }

    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( cutCellUOpen( x, y ) > 0.0f )
                u_[uIndex( x, y )] -= gradientScale *
                    ( pressure_[cellIndex( x, y )] - pressure_[cellIndex( x - 1, y )] );

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( cutCellVOpen( x, y ) > 0.0f )
                v_[vIndex( x, y )] -= gradientScale *
                    ( pressure_[cellIndex( x, y )] - pressure_[cellIndex( x, y - 1 )] );

    cutCellPressure_.divergenceAfter = cutCellFluxDivergenceRms( 1.0f / timeStep );
    double squaredResidual = 0.0;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            if( cellFluidFraction_[cell] == 0.0f ) continue;
            const float p = pressure_[cell];
            const float sum = cutCellUOpen( x, y ) *
                    ( x > 0 ? pressure_[cellIndex( x - 1, y )] - p : 0.0f ) +
                cutCellUOpen( x + 1, y ) *
                    ( x + 1 < width_ ? pressure_[cellIndex( x + 1, y )] - p : 0.0f ) +
                cutCellVOpen( x, y ) *
                    ( y > 0 ? pressure_[cellIndex( x, y - 1 )] - p : 0.0f ) +
                cutCellVOpen( x, y + 1 ) *
                    ( y + 1 < height_ ? pressure_[cellIndex( x, y + 1 )] - p : 0.0f );
            const double residual = static_cast<double>(
                sum - divergence_[cell] * pressureScale ) * ( timeStep / cellSize_ );
            squaredResidual += residual * residual;
        }
    // 결합 PCG의 압력 방정식 잔차는 dt/h²를 곱하면 연속성 잔차 (1/s)다.
    // 기존 Jacobi의 legacy pressureResidual 표시 단위는 이 단계에서 변경하지 않는다.
    cutCellPressure_.pressureResidual = solvedCoupled
        ? coupledPressureResidual * ( timeStep / ( cellSize_ * cellSize_ ) ) :
        ( cutCellPressure_.activeCells > 0
            ? static_cast<float>( std::sqrt( squaredResidual / cutCellPressure_.activeCells ) ) : 0.0f );

    // 수렴한 Schur 해에서는 행렬에 들어간 B 자체를 pressure traction으로
    // 사용한다. 별도의 COM 근사 압력 적분은 중복하면 안 된다.
    if( solvedCoupled )
    {
        for( const coupledPressureFace& face : coupledPressureScratch_.faces )
        {
            const double p = pressure_[face.cell];
            fluidObstacleForce2& force = feedback[face.owner];
            force.force += static_cast<float>( p ) *
                vec2{ static_cast<float>( face.fx ), static_cast<float>( face.fy ) };
            force.torque += static_cast<float>( p * face.torque );
        }
        synchronizeCutCellBoundaries( timeStep, obstacles, feedback );

        // 동일 sub-face 교환의 선형화된 volume source Δ(f)/dt를 추가한다.
        // 마지막 geometry snapshot 자체를 변경하지 않으므로 다음 Step의
        // swept volume·tracer mass와 이 예측 증분을 이중 적용하지 않는다.
        double squaredContinuity = 0.0;
        double volumeL1 = 0.0;
        for( int y = 0; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int cell = cellIndex( x, y );
                if( cellFluidFraction_[cell] <= 0.0f ) continue;
                const float fluidFlux =
                    cutCellUOpen( x + 1, y ) * u_[uIndex( x + 1, y )] -
                    cutCellUOpen( x, y ) * u_[uIndex( x, y )] +
                    cutCellVOpen( x, y + 1 ) * v_[vIndex( x, y + 1 )] -
                    cutCellVOpen( x, y ) * v_[vIndex( x, y )];
                const float solidFlux =
                    pressureUSolidFlux_[uIndex( x + 1, y )] - pressureUSolidFlux_[uIndex( x, y )] +
                    pressureVSolidFlux_[vIndex( x, y + 1 )] - pressureVSolidFlux_[vIndex( x, y )];
                const int region = cutCellComponent_[cell];
                const double mean = region >= 0 ? regionSums[region] / regionSizes[region] : 0.0;
                const double error = static_cast<double>( fluidFlux + solidFlux ) / cellSize_ +
                    static_cast<double>( cutCellVolumeRate_[cell] ) / timeStep - mean;
                squaredContinuity += error * error;
                volumeL1 += std::abs( static_cast<double>( solidFlux ) ) * cellSize_ * timeStep;
            }
        cutCellPressure_.predictedContinuityRms = cutCellPressure_.activeCells > 0
            ? static_cast<float>( std::sqrt( squaredContinuity / cutCellPressure_.activeCells ) ) : 0.0f;
        cutCellPressure_.predictedSolidVolumeL1 = static_cast<float>( volumeL1 );
        return;
    }

    // 비결합 Cut-cell의 기존 셀 aperture/nearest COM 표면 반력 경로.
    if( obstacles.empty() ) return;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            if( cellFluidFraction_[cell] == 0.0f ) continue;
            const float l = x == 0 ? cellFluidFraction_[cell] : cutCellUOpen( x, y );
            const float r = x == width_ - 1 ? cellFluidFraction_[cell] : cutCellUOpen( x + 1, y );
            const float b = y == 0 ? cellFluidFraction_[cell] : cutCellVOpen( x, y );
            const float t = y == height_ - 1 ? cellFluidFraction_[cell] : cutCellVOpen( x, y + 1 );
            const vec2 force = -pressure_[cell] * cellSize_ * vec2{ r - l, t - b };
            if( LengthSquared( force ) == 0.0f ) continue;
            const vec2 center = getCellCenter( x, y );
            std::size_t owner = 0;
            float nearest = std::numeric_limits<float>::max();
            for( std::size_t i = 0; i < obstacles.size(); ++i )
            {
                if( !IsValidFluidObstacle( obstacles[i] ) ) continue;
                const float distance = LengthSquared( center - obstacles[i].center );
                if( distance < nearest ) { nearest = distance; owner = i; }
            }
            if( nearest == std::numeric_limits<float>::max() ) continue;
            feedback[owner].force += force;
            feedback[owner].torque += Cross( center - obstacles[owner].center, force );
        }
}

bool fluidField2::solveCoupledPressure( float timeStep, int iterations,
    std::span<const fluidShapeObstacle2> obstacles, bool cutCell )
{
    // 두 모드 모두 대칭인 A = L + B M_body^-1 B^T를 PCG로 푼다.
    // Cut-cell L은 개방 면적 가중치, B는 반환하는 표면 압력 반력과 동일한 계수다.
    // 정확한 cut interface 적분이 아닌 기존 cell aperture traction 근사만 결합한다.
    pressureMomentum_.coupledIterationLimit = iterations;
    const auto fail = [&]( fluidCoupledPressureStatus2 status )
    {
        pressureMomentum_.coupledStatus = status;
        return false;
    };
    auto& scratch = coupledPressureScratch_;
    auto& faces = scratch.faces;
    faces.clear();
    auto& diagonal = scratch.diagonal;
    diagonal.assign( pressure_.size(), 0.0 );
    int active = 0;
    double rhsSum = 0.0;
    const double scale = static_cast<double>( cellSize_ ) * cellSize_ / timeStep;
    auto& rhs = scratch.rhs;
    rhs.assign( pressure_.size(), 0.0 );
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            if( cutCell ? cellFluidFraction_[cell] <= 0.0f : solidOwner_[cell] >= 0 ) continue;
            const double count = cutCell
                ? cutCellUOpen( x, y ) + cutCellUOpen( x + 1, y ) +
                    cutCellVOpen( x, y ) + cutCellVOpen( x, y + 1 )
                : ( x > 0 && !isSolidCell( x - 1, y ) ? 1.0 : 0.0 ) +
                    ( x + 1 < width_ && !isSolidCell( x + 1, y ) ? 1.0 : 0.0 ) +
                    ( y > 0 && !isSolidCell( x, y - 1 ) ? 1.0 : 0.0 ) +
                    ( y + 1 < height_ && !isSolidCell( x, y + 1 ) ? 1.0 : 0.0 );
            diagonal[cell] = count;
            rhs[cell] = -static_cast<double>( divergence_[cell] ) * scale;
            rhsSum += rhs[cell];
            ++active;
        }
    if( active == 0 )
    {
        std::fill( pressure_.begin(), pressure_.end(), 0.0f );
        pressureMomentum_.coupledStatus = fluidCoupledPressureStatus2::converged;
        return true;
    }

    int dynamicBoundaryFaces = 0;
    const auto addBoundary = [&]( int cell, int owner, vec2 force, vec2 point )
    {
        if( owner < 0 || cell < 0 ) return;
        const fluidShapeObstacle2& obstacle = obstacles[static_cast<std::size_t>( owner )];
        const bool dynamic = obstacle.inverseMass > 0.0f || obstacle.inverseInertia > 0.0f;
        // 이진 MAC은 종전과 같이 dynamic interface만 기록한다.
        // Cut-cell은 static/kinematic pressure traction도 feedback에 남긴다.
        if( !cutCell && !dynamic ) return;
        const double torque = Cross( point - obstacle.center, force );
        faces.push_back( { cell, owner, force.x, force.y, torque } );
        if( !dynamic ) return;
        ++dynamicBoundaryFaces;
        // Diagonal-only (Jacobi) preconditioner. Same-body cross terms are
        // ignored in the preconditioner, not in the Schur matrix product.
        diagonal[cell] += static_cast<double>( obstacle.inverseMass ) *
            ( static_cast<double>( force.x ) * force.x + static_cast<double>( force.y ) * force.y ) +
            static_cast<double>( obstacle.inverseInertia ) * torque * torque;
    };
    if( cutCell )
    {
        // 막힌 공유 sub-face를 힘/토크 적분점으로 사용한다. 점유율 및
        // 경계 속도와 같은 표본 수를 사용해 B와 B^T의 adjoint를 유지한다.
        const int sampleCount = cutCellSampleCount_;
        const float sampleScale = 1.0f / sampleCount;
        std::vector<double> bounds( obstacles.size(), -1.0 );
        for( std::size_t i = 0; i < obstacles.size(); ++i )
            if( IsValidFluidObstacle( obstacles[i] ) )
                bounds[i] = FluidObstacleBoundRadius( obstacles[i] );
        const auto firstOwner = [&]( vec2 point )
        {
            for( std::size_t i = 0; i < obstacles.size(); ++i )
            {
                if( bounds[i] < 0.0 ) continue;
                const vec2 center = obstacles[i].transform.position;
                if( std::abs( static_cast<double>( point.x ) - center.x ) > bounds[i] ||
                    std::abs( static_cast<double>( point.y ) - center.y ) > bounds[i] ) continue;
                if( ContainsFluidObstacle( obstacles[i], point ) ) return static_cast<int>( i );
            }
            return -1;
        };

        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const int left = cellIndex( x - 1, y );
                const int right = cellIndex( x, y );
                if( cellFluidFraction_[left] <= 0.0f && cellFluidFraction_[right] <= 0.0f ) continue;
                const vec2 force{ cellSize_ * sampleScale, 0.0f };
                for( int sample = 0; sample < sampleCount; ++sample )
                {
                    const vec2 point = origin_ + cellSize_ * vec2{
                        static_cast<float>( x ), static_cast<float>( y ) + ( sample + 0.5f ) * sampleScale };
                    const int owner = firstOwner( point );
                    if( owner < 0 ) continue;
                    if( cellFluidFraction_[left] > 0.0f ) addBoundary( left, owner, force, point );
                    if( cellFluidFraction_[right] > 0.0f ) addBoundary( right, owner, -force, point );
                }
            }

        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int bottom = cellIndex( x, y - 1 );
                const int top = cellIndex( x, y );
                if( cellFluidFraction_[bottom] <= 0.0f && cellFluidFraction_[top] <= 0.0f ) continue;
                const vec2 force{ 0.0f, cellSize_ * sampleScale };
                for( int sample = 0; sample < sampleCount; ++sample )
                {
                    const vec2 point = origin_ + cellSize_ * vec2{
                        static_cast<float>( x ) + ( sample + 0.5f ) * sampleScale, static_cast<float>( y ) };
                    const int owner = firstOwner( point );
                    if( owner < 0 ) continue;
                    if( cellFluidFraction_[bottom] > 0.0f ) addBoundary( bottom, owner, force, point );
                    if( cellFluidFraction_[top] > 0.0f ) addBoundary( top, owner, -force, point );
                }
            }
    }
    else
    {
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const int left = cellIndex( x - 1, y );
                const int right = cellIndex( x, y );
                const int leftOwner = solidOwner_[left];
                const int rightOwner = solidOwner_[right];
                const vec2 point = origin_ + cellSize_ *
                    vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
                if( leftOwner < 0 && rightOwner >= 0 ) addBoundary( left, rightOwner, { cellSize_, 0.0f }, point );
                if( rightOwner < 0 && leftOwner >= 0 ) addBoundary( right, leftOwner, { -cellSize_, 0.0f }, point );
            }
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int bottom = cellIndex( x, y - 1 );
                const int top = cellIndex( x, y );
                const int bottomOwner = solidOwner_[bottom];
                const int topOwner = solidOwner_[top];
                const vec2 point = origin_ + cellSize_ *
                    vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
                if( bottomOwner < 0 && topOwner >= 0 ) addBoundary( bottom, topOwner, { 0.0f, cellSize_ }, point );
                if( topOwner < 0 && bottomOwner >= 0 ) addBoundary( top, bottomOwner, { 0.0f, -cellSize_ }, point );
            }
    }
    if( dynamicBoundaryFaces == 0 ) return fail( fluidCoupledPressureStatus2::noBoundary );

    // Closed-domain Neumann pressure has an arbitrary gauge. Remove the global
    // incompatible source; disconnected cavities need a future component treatment.
    if( !cutCell )
    {
        const double mean = rhsSum / active;
        for( std::size_t i = 0; i < rhs.size(); ++i )
            if( solidOwner_[i] < 0 ) rhs[i] -= mean;
    }
    // Cut-cell의 region별 호환성 보정은 projectCutCell()에서 이미 완료했다.

    auto& response = scratch.response;
    response.assign( obstacles.size(), {} );
    const auto multiply = [&]( const std::vector<double>& values, std::vector<double>& result )
    {
        std::fill( result.begin(), result.end(), 0.0 );
        std::fill( response.begin(), response.end(), coupledBodyResponse{} );
        for( int y = 0; y < height_; ++y )
            for( int x = 1; x < width_; ++x )
            {
                const int left = cellIndex( x - 1, y ), right = cellIndex( x, y );
                if( !cutCell && ( solidOwner_[left] >= 0 || solidOwner_[right] >= 0 ) ) continue;
                const double weight = cutCell ? cutCellUOpen( x, y ) : 1.0;
                const double d = weight * ( values[left] - values[right] );
                result[left] += d;
                result[right] -= d;
            }
        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int bottom = cellIndex( x, y - 1 ), top = cellIndex( x, y );
                if( !cutCell && ( solidOwner_[bottom] >= 0 || solidOwner_[top] >= 0 ) ) continue;
                const double weight = cutCell ? cutCellVOpen( x, y ) : 1.0;
                const double d = weight * ( values[bottom] - values[top] );
                result[bottom] += d;
                result[top] -= d;
            }
        for( const coupledPressureFace& face : faces )
        {
            coupledBodyResponse& body = response[face.owner];
            const double p = values[face.cell];
            body.fx += p * face.fx;
            body.fy += p * face.fy;
            body.torque += p * face.torque;
        }
        for( const coupledPressureFace& face : faces )
        {
            const fluidShapeObstacle2& obstacle = obstacles[static_cast<std::size_t>( face.owner )];
            const coupledBodyResponse& body = response[face.owner];
            result[face.cell] += static_cast<double>( obstacle.inverseMass ) *
                ( face.fx * body.fx + face.fy * body.fy ) +
                static_cast<double>( obstacle.inverseInertia ) * face.torque * body.torque;
        }
    };

    auto& x = scratch.x;
    auto& residual = scratch.residual;
    auto& z = scratch.z;
    auto& direction = scratch.direction;
    auto& product = scratch.product;
    x.assign( rhs.size(), 0.0 );
    residual = rhs;
    z.assign( rhs.size(), 0.0 );
    direction.assign( rhs.size(), 0.0 );
    product.assign( rhs.size(), 0.0 );
    double rz = 0.0, squared = 0.0;
    for( std::size_t i = 0; i < rhs.size(); ++i )
    {
        if( cutCell ? cellFluidFraction_[i] <= 0.0f : solidOwner_[i] >= 0 ) continue;
        z[i] = residual[i] / std::max( 1.0e-8, diagonal[i] );
        direction[i] = z[i];
        rz += residual[i] * z[i];
        squared += residual[i] * residual[i];
    }
    if( !std::isfinite( rz ) || !std::isfinite( squared ) )
        return fail( fluidCoupledPressureStatus2::numericalBreakdown );
    const double initialSquared = squared;
    const double tolerance = std::max( 1.0e-18, initialSquared * 1.0e-10 );
    pressureMomentum_.coupledInitialResidualRms = static_cast<float>( std::sqrt( initialSquared / active ) );
    for( int iteration = 0; iteration < iterations && squared > tolerance; ++iteration )
    {
        multiply( direction, product );
        double denom = 0.0;
        for( std::size_t i = 0; i < rhs.size(); ++i )
            denom += direction[i] * product[i];
        if( !std::isfinite( denom ) || denom <= 0.0 )
            return fail( fluidCoupledPressureStatus2::numericalBreakdown );
        const double alpha = rz / denom;
        if( !std::isfinite( alpha ) )
            return fail( fluidCoupledPressureStatus2::numericalBreakdown );
        squared = 0.0;
        for( std::size_t i = 0; i < rhs.size(); ++i )
        {
            x[i] += alpha * direction[i];
            residual[i] -= alpha * product[i];
            squared += residual[i] * residual[i];
        }
        pressureMomentum_.coupledIterations = iteration + 1;
        if( !std::isfinite( squared ) )
            return fail( fluidCoupledPressureStatus2::numericalBreakdown );
        if( squared <= tolerance ) break;
        double nextRz = 0.0;
        for( std::size_t i = 0; i < rhs.size(); ++i )
        {
            z[i] = residual[i] / std::max( 1.0e-8, diagonal[i] );
            nextRz += residual[i] * z[i];
        }
        if( !std::isfinite( nextRz ) || rz <= 0.0 )
            return fail( fluidCoupledPressureStatus2::numericalBreakdown );
        const double beta = nextRz / rz;
        if( !std::isfinite( beta ) )
            return fail( fluidCoupledPressureStatus2::numericalBreakdown );
        for( std::size_t i = 0; i < rhs.size(); ++i )
            direction[i] = z[i] + beta * direction[i];
        rz = nextRz;
    }

    // Recursive CG 잔차는 반올림으로 실제 b-A*x와 달라질 수 있으므로
    // 종료 시 항상 원래 연산자로 직접 재계산한다. 반복 한도는 성공 조건이 아님.
    multiply( x, product );
    double trueSquared = 0.0;
    for( std::size_t i = 0; i < rhs.size(); ++i )
    {
        if( cutCell ? cellFluidFraction_[i] <= 0.0f : solidOwner_[i] >= 0 ) continue;
        const double error = rhs[i] - product[i];
        trueSquared += error * error;
    }
    if( !std::isfinite( trueSquared ) )
        return fail( fluidCoupledPressureStatus2::numericalBreakdown );
    pressureMomentum_.coupledFinalResidualRms = static_cast<float>( std::sqrt( trueSquared / active ) );
    pressureMomentum_.coupledRelativeResidual = initialSquared > 0.0
        ? static_cast<float>( std::sqrt( trueSquared / initialSquared ) ) : 0.0f;
    if( trueSquared > tolerance )
        return fail( fluidCoupledPressureStatus2::iterationLimit );

    double pressureSum = 0.0;
    for( std::size_t i = 0; i < rhs.size(); ++i )
        if( cutCell ? cellFluidFraction_[i] > 0.0f : solidOwner_[i] < 0 ) pressureSum += x[i];
    // B*invM*B^T가 있는 Cut-cell에서는 불완전 점유/외벽 근사로
    // 상수 압력 방향이 정확한 nullspace가 아닐 수 있다. 수렴한 x에서
    // 임의의 gauge를 빼면 A*x=b를 다시 위반하므로 그대로 사용한다.
    const double gauge = cutCell ? 0.0 : pressureSum / active;
    for( std::size_t i = 0; i < rhs.size(); ++i )
    {
        if( cutCell ? cellFluidFraction_[i] <= 0.0f : solidOwner_[i] >= 0 ) { pressure_[i] = 0.0f; continue; }
        const double value = x[i] - gauge;
        if( !std::isfinite( value ) || std::abs( value ) > std::numeric_limits<float>::max() )
            return fail( fluidCoupledPressureStatus2::invalidPressure );
        pressure_[i] = static_cast<float>( value );
    }
    pressureMomentum_.coupledStatus = fluidCoupledPressureStatus2::converged;
    return true;
}

void fluidField2::project( float timeStep, int pressureIterations,
    std::span<const fluidShapeObstacle2> obstacles, std::span<fluidObstacleForce2> feedback )
{
    // div(v) / dt = Laplace(p). 벽/solid no-through, Neumann 압력 경계.
    // 압력 경계가 fluid-fluid face 운동량을 얼마나 변경하는지 독립 측정한다.
    pressureMomentum_ = {};
    pressureMomentum_.energy.fluidBefore = getFluidKineticEnergy();
    const auto rigidKineticEnergy = []( std::span<const fluidShapeObstacle2> shapes,
        std::span<const fluidObstacleForce2> impulses, float dt )
    {
        double energy = 0.0;
        for( std::size_t i = 0; i < shapes.size(); ++i )
        {
            const fluidShapeObstacle2& shape = shapes[i];
            if( shape.inverseMass > 0.0f )
            {
                const vec2 velocity = shape.linearVelocity + ( impulses.empty() ? vec2{} :
                    ( dt * shape.inverseMass ) * impulses[i].force );
                energy += 0.5 * static_cast<double>( Dot( velocity, velocity ) ) / shape.inverseMass;
            }
            if( shape.inverseInertia > 0.0f )
            {
                const double omega = static_cast<double>( shape.angularVelocity ) + ( impulses.empty() ? 0.0 :
                    static_cast<double>( dt ) * shape.inverseInertia * impulses[i].torque );
                energy += 0.5 * omega * omega / shape.inverseInertia;
            }
        }
        return energy;
    };
    pressureMomentum_.energy.rigidBefore = rigidKineticEnergy( obstacles, {}, 0.0f );
    const vec2 beforePressure = getFluidMomentum();
    const float inverseCell = 1.0f / cellSize_;
    const float scale = cellSize_ * cellSize_ / timeStep;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int cell = cellIndex( x, y );
            divergence_[cell] = solidOwner_[cell] < 0
                ? ( u_[uIndex( x + 1, y )] - u_[uIndex( x, y )] +
                    v_[vIndex( x, y + 1 )] - v_[vIndex( x, y )] ) * inverseCell
                : 0.0f;
        }

    const bool coupled = std::any_of( obstacles.begin(), obstacles.end(),
        []( const fluidShapeObstacle2& obstacle )
        {
            return obstacle.inverseMass > 0.0f || obstacle.inverseInertia > 0.0f;
        } );
    const bool solvedCoupled = coupled && solveCoupledPressure( timeStep, pressureIterations, obstacles );
    pressureMomentum_.usedCoupledProjection = solvedCoupled;
    pressureMomentum_.coupledSolveFailed = coupled && !solvedCoupled &&
        pressureMomentum_.coupledStatus != fluidCoupledPressureStatus2::noBoundary;
    // 연립 압력 계산에 실패했을 때 질량을 무시하는 Jacobi 반력을 강체에
    // 주입하면 added-mass 폭주가 재발할 수 있다. 실패 Step은 압력 반력을
    // 건너뛰고 진단을 남긴다. 이 경우 비압축성 제약은 만족하지 않을 수 있다.
    if( pressureMomentum_.coupledSolveFailed )
        std::fill( pressure_.begin(), pressure_.end(), 0.0f );
    else if( !solvedCoupled )
    {
        std::fill( pressure_.begin(), pressure_.end(), 0.0f );
        for( int iteration = 0; iteration < pressureIterations; ++iteration )
        {
            for( int y = 0; y < height_; ++y )
                for( int x = 0; x < width_; ++x )
                {
                    const int cell = cellIndex( x, y );
                    if( solidOwner_[cell] >= 0 ) { nextPressure_[cell] = 0.0f; continue; }
                float sum = 0.0f;
                int neighbors = 0;
                if( x > 0 && !isSolidCell( x - 1, y ) ) { sum += pressure_[cellIndex( x - 1, y )]; ++neighbors; }
                if( x + 1 < width_ && !isSolidCell( x + 1, y ) ) { sum += pressure_[cellIndex( x + 1, y )]; ++neighbors; }
                if( y > 0 && !isSolidCell( x, y - 1 ) ) { sum += pressure_[cellIndex( x, y - 1 )]; ++neighbors; }
                if( y + 1 < height_ && !isSolidCell( x, y + 1 ) ) { sum += pressure_[cellIndex( x, y + 1 )]; ++neighbors; }
                nextPressure_[cell] = neighbors > 0 ? ( sum - divergence_[cell] * scale ) / neighbors : 0.0f;
            }
            pressure_.swap( nextPressure_ );
        }
    }

    const float gradientScale = timeStep * inverseCell;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            if( !isSolidCell( x - 1, y ) && !isSolidCell( x, y ) )
                u_[uIndex( x, y )] -= gradientScale * ( pressure_[cellIndex( x, y )] - pressure_[cellIndex( x - 1, y )] );

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            if( !isSolidCell( x, y - 1 ) && !isSolidCell( x, y ) )
                v_[vIndex( x, y )] -= gradientScale * ( pressure_[cellIndex( x, y )] - pressure_[cellIndex( x, y - 1 )] );

    // Pressure traction: solid-fluid 경계의 fluid cell 압력을 solid 외향 법선에 적분.
    // 2D unit-depth, rho=1 근사이므로 반력은 정밀한 cut-cell 표면 적분이 아님.
    const auto accumulate = [&]( int owner, vec2 normal, vec2 point, float pressure )
    {
        fluidObstacleForce2& result = feedback[static_cast<std::size_t>( owner )];
        const vec2 force = pressure * cellSize_ * normal;
        result.force += force;
        result.torque += Cross( point - obstacles[static_cast<std::size_t>( owner )].center, force );
    };
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const int left = solidOwner_[cellIndex( x - 1, y )];
            const int right = solidOwner_[cellIndex( x, y )];
            if( left == right ) continue;
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
            if( left < 0 && right >= 0 ) accumulate( right, { 1.0f, 0.0f }, point, pressure_[cellIndex( x - 1, y )] );
            else if( right < 0 && left >= 0 ) accumulate( left, { -1.0f, 0.0f }, point, pressure_[cellIndex( x, y )] );
        }
    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int bottom = solidOwner_[cellIndex( x, y - 1 )];
            const int top = solidOwner_[cellIndex( x, y )];
            if( bottom == top ) continue;
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
            if( bottom < 0 && top >= 0 ) accumulate( top, { 0.0f, 1.0f }, point, pressure_[cellIndex( x, y - 1 )] );
            else if( top < 0 && bottom >= 0 ) accumulate( bottom, { 0.0f, -1.0f }, point, pressure_[cellIndex( x, y )] );
        }

    // 내부 face와 강체 사이의 같은 압력해를 적용한다. pressure traction으로
    // 예측되는 Body 속도 증분을 solid-normal MAC face에도 갱신한다.
    // World의 Drag/Contact/Joint로 뒤이어 변하는 속도까지 미리 반영하지는 않는다.
    if( solvedCoupled ) applySolidBoundaries( obstacles, feedback, timeStep );

    // 닫힌 격자의 외벽도 유체 압력으로 반력을 받는다.
    // Solid가 외곽에 걸쳐 있는 셀은 외벽에 유체 압력이 없으므로 제외한다.
    // fluid+solid+wall = 0: 동일 pressure 미지수의 내부 face gradient가
    // 전부 telescope된 결과 (단, float 반올림에 따른 잔차는 남음).
    double wallX = 0.0;
    double wallY = 0.0;
    const double faceArea = static_cast<double>( cellSize_ ); // unit depth
    for( int y = 0; y < height_; ++y )
    {
        if( !isSolidCell( 0, y ) ) wallX -= pressure_[cellIndex( 0, y )] * faceArea;
        if( !isSolidCell( width_ - 1, y ) ) wallX += pressure_[cellIndex( width_ - 1, y )] * faceArea;
    }
    for( int x = 0; x < width_; ++x )
    {
        if( !isSolidCell( x, 0 ) ) wallY -= pressure_[cellIndex( x, 0 )] * faceArea;
        if( !isSolidCell( x, height_ - 1 ) ) wallY += pressure_[cellIndex( x, height_ - 1 )] * faceArea;
    }

    pressureMomentum_.energy.fluidAfter = getFluidKineticEnergy();
    pressureMomentum_.energy.rigidAfter = rigidKineticEnergy( obstacles, feedback, timeStep );

    vec2 solidForce{};
    for( const fluidObstacleForce2& force : feedback ) solidForce += force.force;
    pressureMomentum_.fluid = getFluidMomentum() - beforePressure;
    pressureMomentum_.solid = timeStep * solidForce;
    pressureMomentum_.wall = timeStep * vec2{ static_cast<float>( wallX ), static_cast<float>( wallY ) };
}

float fluidField2::getDivergenceRms() const noexcept
{
    double sum = 0.0;
    int fluidCells = 0;
    const float inverseCell = 1.0f / cellSize_;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            if( isSolidCell( x, y ) ) continue;
            const float divergence =
                ( u_[uIndex( x + 1, y )] - u_[uIndex( x, y )] +
                  v_[vIndex( x, y + 1 )] - v_[vIndex( x, y )] ) * inverseCell;
            sum += static_cast<double>( divergence ) * divergence;
            ++fluidCells;
        }
    return fluidCells > 0 ? static_cast<float>( std::sqrt( sum / fluidCells ) ) : 0.0f;
}

} // namespace zonai
