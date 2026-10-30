#include "fluid/fluidField2.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace zonai
{

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

void fluidField2::step( float timeStep, int pressureIterations )
{
    if( !std::isfinite( timeStep ) || timeStep <= 0.0f ) return;

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
    project( timeStep, std::clamp( pressureIterations, 1, 200 ) );
}

void fluidField2::project( float timeStep, int pressureIterations )
{
    // div(v) / dt = Laplace(p). 벽에서는 수직 face 속도 0, Neumann 압력 경계.
    const float inverseCell = 1.0f / cellSize_;
    const float scale = cellSize_ * cellSize_ / timeStep;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            divergence_[cellIndex( x, y )] =
                ( u_[uIndex( x + 1, y )] - u_[uIndex( x, y )] +
                  v_[vIndex( x, y + 1 )] - v_[vIndex( x, y )] ) * inverseCell;
        }

    std::fill( pressure_.begin(), pressure_.end(), 0.0f );
    for( int iteration = 0; iteration < pressureIterations; ++iteration )
    {
        for( int y = 0; y < height_; ++y )
            for( int x = 0; x < width_; ++x )
            {
                float sum = 0.0f;
                int neighbors = 0;
                if( x > 0 ) { sum += pressure_[cellIndex( x - 1, y )]; ++neighbors; }
                if( x + 1 < width_ ) { sum += pressure_[cellIndex( x + 1, y )]; ++neighbors; }
                if( y > 0 ) { sum += pressure_[cellIndex( x, y - 1 )]; ++neighbors; }
                if( y + 1 < height_ ) { sum += pressure_[cellIndex( x, y + 1 )]; ++neighbors; }
                nextPressure_[cellIndex( x, y )] =
                    ( sum - divergence_[cellIndex( x, y )] * scale ) / neighbors;
            }
        pressure_.swap( nextPressure_ );
    }

    const float gradientScale = timeStep * inverseCell;
    for( int y = 0; y < height_; ++y )
        for( int x = 1; x < width_; ++x )
            u_[uIndex( x, y )] -= gradientScale * ( pressure_[cellIndex( x, y )] - pressure_[cellIndex( x - 1, y )] );

    for( int y = 1; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
            v_[vIndex( x, y )] -= gradientScale * ( pressure_[cellIndex( x, y )] - pressure_[cellIndex( x, y - 1 )] );
}

float fluidField2::getDivergenceRms() const noexcept
{
    double sum = 0.0;
    const float inverseCell = 1.0f / cellSize_;
    for( int y = 0; y < height_; ++y )
        for( int x = 0; x < width_; ++x )
        {
            const float divergence =
                ( u_[uIndex( x + 1, y )] - u_[uIndex( x, y )] +
                  v_[vIndex( x, y + 1 )] - v_[vIndex( x, y )] ) * inverseCell;
            sum += static_cast<double>( divergence ) * divergence;
        }
    return static_cast<float>( std::sqrt( sum / static_cast<double>( width_ * height_ ) ) );
}

} // namespace zonai
