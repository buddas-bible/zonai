#include "fluid/fluidField2.h"

#include <algorithm>
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

bool IsValidFluidObstacle( const fluidShapeObstacle2& obstacle )
{
    return IsFinite( obstacle.center ) && IsFinite( obstacle.transform.position ) &&
        std::isfinite( obstacle.transform.rotation.c ) && std::isfinite( obstacle.transform.rotation.s ) &&
        IsFinite( obstacle.linearVelocity ) && std::isfinite( obstacle.angularVelocity );
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
    pressure_.resize( static_cast<std::size_t>( width ) * height );
    nextPressure_.resize( pressure_.size() );
    divergence_.resize( pressure_.size() );
    solidOwner_.assign( pressure_.size(), -1 );
    previousSolidOwner_.assign( pressure_.size(), -1 );
    cellFluidFraction_.assign( pressure_.size(), 1.0f );
    previousCellFluidFraction_.assign( pressure_.size(), 1.0f );
    uOpenFraction_.assign( u_.size(), 1.0f );
    vOpenFraction_.assign( v_.size(), 1.0f );
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

void fluidField2::clear() noexcept
{
    std::fill( u_.begin(), u_.end(), 0.0f );
    std::fill( v_.begin(), v_.end(), 0.0f );
    std::fill( nextU_.begin(), nextU_.end(), 0.0f );
    std::fill( nextV_.begin(), nextV_.end(), 0.0f );
    std::fill( pressure_.begin(), pressure_.end(), 0.0f );
    std::fill( nextPressure_.begin(), nextPressure_.end(), 0.0f );
    std::fill( divergence_.begin(), divergence_.end(), 0.0f );
    std::fill( solidOwner_.begin(), solidOwner_.end(), -1 );
    std::fill( previousSolidOwner_.begin(), previousSolidOwner_.end(), -1 );
    std::fill( cellFluidFraction_.begin(), cellFluidFraction_.end(), 1.0f );
    std::fill( previousCellFluidFraction_.begin(), previousCellFluidFraction_.end(), 1.0f );
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
    pressureMomentum_ = {};
    stepMomentum_ = {};
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

        // 최초의 원이 cell을 소유함. 이 API는 겹치는 solid의 union을 처리하지만
        // 겹친 부분의 상대 운동은 물리적으로 해결하지 않음 (World에서 충돌 해결).
        for( int y = 0; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                const int cell = cellIndex( x, y );
                if( solidOwner_[cell] >= 0 ) continue;
                if( ContainsFluidObstacle( obstacle, getCellCenter( x, y ) ) )
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

void fluidField2::updateCutCellGeometry( std::span<const fluidShapeObstacle2> obstacles )
{
    // Cell-center solid mask를 변경하지 않은 독립적인 기하 관측 레이어.
    // 4x4 sub-cell sample과 4개 face sample로 union geometry의 부분 점유를 추정한다.
    // 이 배열을 pressure gradient/flux에 사용하려면 cut-cell 연산자 전체를 함께 변경해야 한다.
    constexpr int sampleCount = 4;
    constexpr float inverseCount = 1.0f / sampleCount;
    constexpr float inverseCellSamples = 1.0f / ( sampleCount * sampleCount );
    std::copy( cellFluidFraction_.begin(), cellFluidFraction_.end(), previousCellFluidFraction_.begin() );
    cutCellTransition_ = {};

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
        const auto isOccupied = [&]( vec2 point )
        {
            for( const fluidShapeObstacle2& obstacle : obstacles )
                if( IsValidFluidObstacle( obstacle ) && ContainsFluidObstacle( obstacle, point ) ) return true;
            return false;
        };

        for( int y = 0; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
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
                int blocked = 0;
                for( int sample = 0; sample < sampleCount; ++sample )
                {
                    const vec2 point = origin_ + cellSize_ * vec2{
                        static_cast<float>( x ), static_cast<float>( y ) + ( sample + 0.5f ) * inverseCount };
                    if( isOccupied( point ) ) ++blocked;
                }
                uOpenFraction_[uIndex( x, y )] = 1.0f - blocked * inverseCount;
            }

        for( int y = 1; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                int blocked = 0;
                for( int sample = 0; sample < sampleCount; ++sample )
                {
                    const vec2 point = origin_ + cellSize_ * vec2{
                        static_cast<float>( x ) + ( sample + 0.5f ) * inverseCount, static_cast<float>( y ) };
                    if( isOccupied( point ) ) ++blocked;
                }
                vOpenFraction_[vIndex( x, y )] = 1.0f - blocked * inverseCount;
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

void fluidField2::applySolidBoundaries( std::span<const fluidShapeObstacle2> obstacles )
{
    // Solid와 인접한 모든 face의 normal velocity를 rigid body 표면속도로 지정.
    // 유체-유체 face만 pressure gradient로 보정하며 가장자리 수직 flux는 0.
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
        {
            const int left = solidOwner_[cellIndex( x - 1, y )];
            const int right = solidOwner_[cellIndex( x, y )];
            const int owner = left >= 0 ? left : right;
            if( owner < 0 ) continue;
            const fluidShapeObstacle2& solid = obstacles[static_cast<std::size_t>( owner )];
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
            u_[uIndex( x, y )] = ( solid.linearVelocity + Cross( solid.angularVelocity, point - solid.center ) ).x;
        }

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const int bottom = solidOwner_[cellIndex( x, y - 1 )];
            const int top = solidOwner_[cellIndex( x, y )];
            const int owner = bottom >= 0 ? bottom : top;
            if( owner < 0 ) continue;
            const fluidShapeObstacle2& solid = obstacles[static_cast<std::size_t>( owner )];
            const vec2 point = origin_ + cellSize_ * vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
            v_[vIndex( x, y )] = ( solid.linearVelocity + Cross( solid.angularVelocity, point - solid.center ) ).y;
        }
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
    remapExposedFaces();
    stepMomentum_.afterObstacles = getFluidMomentum();
    applySolidBoundaries( obstacles );

    // 반 라그랑주 이류: face마다 이전 속도장을 역추적해 보간함.
    // 기존 u/v는 읽기 전용으로 두고 next buffer에 기록하므로 순서 의존성이 없음.
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
    applySolidBoundaries( obstacles );
    stepMomentum_.afterAdvection = getFluidMomentum();
    project( timeStep, std::clamp( pressureIterations, 1, 200 ), obstacles, feedback );
    stepMomentum_.afterPressure = getFluidMomentum();
}

void fluidField2::project( float timeStep, int pressureIterations,
    std::span<const fluidShapeObstacle2> obstacles, std::span<fluidObstacleForce2> feedback )
{
    // div(v) / dt = Laplace(p). 벽/solid no-through, Neumann 압력 경계.
    // 압력 경계가 fluid-fluid face 운동량을 얼마나 변경하는지 독립 측정한다.
    pressureMomentum_ = {};
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
