#include "fluid/fluidField2.h"
#include "math/deterministicMath.h"
#include "collision/shapeGeometry.h"
#include "rigidBodyDemo.h"
#include "world_state_snapshot.h"
#include "joint_substep_trace.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace zonai
{

// Test-only access to raw, authoritative MAC storage. Never snapshots padding,
// allocator capacity, wall-clock timings, or disposable PCG scratch buffers.
struct fluidStateTraceProbe2
{
    template<typename T>
    static void appendScalar( std::string& trace, const std::string& key, T value )
    {
        char bits[32]{};
        if constexpr( std::is_same_v<T, float> )
            std::snprintf( bits, sizeof( bits ), "%08x/4", std::bit_cast<std::uint32_t>( value ) );
        else if constexpr( std::is_same_v<T, double> )
            std::snprintf( bits, sizeof( bits ), "%016llx/8",
                static_cast<unsigned long long>( std::bit_cast<std::uint64_t>( value ) ) );
        else
            std::snprintf( bits, sizeof( bits ), "%08x/4", static_cast<std::uint32_t>( value ) );

        trace.append( key ).append( "\t" ).append( bits ).append( "\n" );
    }

    template<typename T>
    static void appendVector( std::string& trace, const std::string& key, const std::vector<T>& values )
    {
        appendScalar( trace, key + ".size", static_cast<int>( values.size() ) );
        for( std::size_t i = 0; i < values.size(); ++i )
            appendScalar( trace, key + "[" + std::to_string( i ) + "]", values[i] );
    }

    static void setStageObserver( fluidField2& field,
        std::function<void(const fluidField2&, int)> callback )
    {
        field.determinismStageObserver_ = std::move( callback );
    }


    // R3e: reconstruct the exact semi-Lagrangian U(3,4) backtrace and
    // bilinear stencil from the read-only Stage-05 input. Mirror the R3g
    // production interpolation and require bitwise agreement with Step.
    static float captureComponent( const std::vector<float>& values,
        int width, int height, float x, float y,
        std::string& trace, const std::string& key )
    {
        appendScalar( trace, key + ".inputX", x );
        appendScalar( trace, key + ".inputY", y );
        x = std::clamp( x, 0.0f, static_cast<float>( width - 1 ) );
        y = std::clamp( y, 0.0f, static_cast<float>( height - 1 ) );
        const int i = static_cast<int>( x );
        const int j = static_cast<int>( y );
        const int i1 = std::min( i + 1, width - 1 );
        const int j1 = std::min( j + 1, height - 1 );
        const float tx = x - i;
        const float ty = y - j;

        const float a = values[j * width + i];
        const float b = values[j * width + i1];
        const float c = values[j1 * width + i];
        const float d = values[j1 * width + i1];
        const float bottom = deterministicLerp( a, b, tx );
        const float top = deterministicLerp( c, d, tx );
        const float result = deterministicLerp( bottom, top, ty );

        appendScalar( trace, key + ".i", i );
        appendScalar( trace, key + ".j", j );
        appendScalar( trace, key + ".i1", i1 );
        appendScalar( trace, key + ".j1", j1 );
        appendScalar( trace, key + ".tx", tx );
        appendScalar( trace, key + ".ty", ty );
        appendScalar( trace, key + ".a", a );
        appendScalar( trace, key + ".b", b );
        appendScalar( trace, key + ".c", c );
        appendScalar( trace, key + ".d", d );
        appendScalar( trace, key + ".bottom", bottom );
        appendScalar( trace, key + ".top", top );
        appendScalar( trace, key + ".output", result );
        return result;
    }

    static float readTargetU( const fluidField2& field )
    {
        return field.u_[4 * ( field.width_ + 1 ) + 3];
    }

    static float captureUAdvectionProbe( const fluidField2& field,
        float timeStep, std::string& trace, const std::string& key )
    {
        // R3d first divergence: 16x12 domain, U face index 71 = U(3,4).
        constexpr int targetX = 3;
        constexpr int targetY = 4;
        if( field.getWidth() != 16 || field.getHeight() != 12 )
            throw std::runtime_error( "advection probe expects the R3d 16x12 fixture" );

        const vec2 point = field.origin_ + field.cellSize_ *
            vec2{ static_cast<float>( targetX ), static_cast<float>( targetY ) + 0.5f };
        const vec2 current = field.sampleFrom( field.u_, field.v_, point );
        const vec2 previous = point - timeStep * current;
        const vec2 result = field.sampleFrom( field.u_, field.v_, previous );

        appendScalar( trace, key + ".point.x", point.x );
        appendScalar( trace, key + ".point.y", point.y );
        appendScalar( trace, key + ".current.x", current.x );
        appendScalar( trace, key + ".current.y", current.y );
        appendScalar( trace, key + ".previous.x", previous.x );
        appendScalar( trace, key + ".previous.y", previous.y );

        const vec2 sourceGrid = ( point - field.origin_ ) / field.cellSize_;
        appendScalar( trace, key + ".sourceGrid.x", sourceGrid.x );
        appendScalar( trace, key + ".sourceGrid.y", sourceGrid.y );
        const float currentU = captureComponent( field.u_, field.width_ + 1, field.height_,
            sourceGrid.x, sourceGrid.y - 0.5f, trace, key + ".sourceU" );
        const float currentV = captureComponent( field.v_, field.width_, field.height_ + 1,
            sourceGrid.x - 0.5f, sourceGrid.y, trace, key + ".sourceV" );
        appendScalar( trace, key + ".sourceUMatches", static_cast<int>(
            std::bit_cast<std::uint32_t>( currentU ) == std::bit_cast<std::uint32_t>( current.x ) ) );
        appendScalar( trace, key + ".sourceVMatches", static_cast<int>(
            std::bit_cast<std::uint32_t>( currentV ) == std::bit_cast<std::uint32_t>( current.y ) ) );

        const vec2 destinationGrid = ( previous - field.origin_ ) / field.cellSize_;
        appendScalar( trace, key + ".destinationGrid.x", destinationGrid.x );
        appendScalar( trace, key + ".destinationGrid.y", destinationGrid.y );
        const float reconstructed = captureComponent( field.u_, field.width_ + 1,
            field.height_, destinationGrid.x, destinationGrid.y - 0.5f,
            trace, key + ".destinationU" );
        appendScalar( trace, key + ".reconstructedMatches", static_cast<int>(
            std::bit_cast<std::uint32_t>( reconstructed ) ==
                std::bit_cast<std::uint32_t>( result.x ) ) );
        appendScalar( trace, key + ".result.u", result.x );
        if( std::bit_cast<std::uint32_t>( currentU ) != std::bit_cast<std::uint32_t>( current.x ) ||
            std::bit_cast<std::uint32_t>( currentV ) != std::bit_cast<std::uint32_t>( current.y ) ||
            std::bit_cast<std::uint32_t>( reconstructed ) != std::bit_cast<std::uint32_t>( result.x ) )
            throw std::runtime_error( "advection interpolation probe differs from production sampling" );
        return result.x;
    }

    // R3i: read-only reconstruction of updateSweptSolidFlux's rotation
    // extraction. Called immediately after the swept stage, before remap,
    // only when the real solver reports that the swept path executed.
    // R3j mirrors the production deterministicAtan2 path. Historical
    // candidate fields are retained as aliases for row-by-row comparison;
    // the original libm witnesses live in the separate FP diagnostic.
    static void captureSweptRotation( const fluidField2& field,
        std::span<const fluidShapeObstacle2> obstacles, float timeStep,
        std::string& trace, const std::string& key )
    {
        const auto& swept = field.getSweptFaceFlux();
        if( !swept.usedSweptFlux ) return;
        if( field.previousObstacles_.size() != obstacles.size() || swept.timeSamples <= 0 )
            throw std::runtime_error( "R3i swept probe has inconsistent history" );

        constexpr float twoPi = 2.0f * std::numbers::pi_v<float>;
        for( std::size_t i = 0; i < obstacles.size(); ++i )
        {
            const auto& start = field.previousObstacles_[i];
            const auto& finish = obstacles[i];
            const std::string prefix = key + ".body[" + std::to_string( i ) + "]";
            const auto scalar = [&]( const char* name, auto value )
            {
                appendScalar( trace, prefix + name, value );
            };

            scalar( ".start.c", start.transform.rotation.c );
            scalar( ".start.s", start.transform.rotation.s );
            scalar( ".finish.c", finish.transform.rotation.c );
            scalar( ".finish.s", finish.transform.rotation.s );
            const float startAngle = deterministicAtan2( start.transform.rotation.s, start.transform.rotation.c );
            const float endAngle = deterministicAtan2( finish.transform.rotation.s, finish.transform.rotation.c );
            const float deltaAngle = std::remainder( endAngle - startAngle, twoPi );
            const float angularSpeed = deltaAngle / timeStep;
            const vec2 linearSpeed = ( finish.transform.position - start.transform.position ) / timeStep;
            scalar( ".startAngle", startAngle );
            scalar( ".endAngle", endAngle );
            scalar( ".deltaAngle", deltaAngle );
            scalar( ".angularSpeed", angularSpeed );
            scalar( ".linearSpeed.x", linearSpeed.x );
            scalar( ".linearSpeed.y", linearSpeed.y );
            // The selected number of samples is reported by the real solver.
            // Reconstruct the first intermediate orientation, not a new solver.
            const float alpha = 0.5f / swept.timeSamples;
            const float intermediateAngle = startAngle + angularSpeed * timeStep * alpha;
            const auto intermediateRotation = rot2::FromRadians( intermediateAngle );
            scalar( ".samples", swept.timeSamples );
            scalar( ".intermediateAngle", intermediateAngle );
            scalar( ".intermediate.c", intermediateRotation.c );
            scalar( ".intermediate.s", intermediateRotation.s );
            scalar( ".candidateStartAngle", startAngle );
            scalar( ".candidateEndAngle", endAngle );
        }
    }

    // R3v: measure only interior open MAC faces, with the same effective
    // aperture as the actual Cut-cell pressure update. This is a partial
    // dual-face ledger, not the kinetic energy of the entire coupled system.
    struct stabilitySample
    {
        double momentumX = 0.0, momentumY = 0.0, energy = 0.0;
        float peak = 0.0f, weightedPeak = 0.0f;
        float tinyPeak = 0.0f, narrowPeak = 0.0f;
        int tinyCount = 0, narrowCount = 0;
        bool peakIsU = true;
        int peakX = -1, peakY = -1;
        float peakOpen = 0.0f;
        float deltaPeak = 0.0f;
        bool deltaIsU = true;
        int deltaX = -1, deltaY = -1;
    };

    static stabilitySample captureStabilityPhase( const fluidField2& field, std::string& trace,
        const std::string& key, const std::vector<float>* beforeU = nullptr,
        const std::vector<float>* beforeV = nullptr )
    {
        stabilitySample result{};
        const double area = static_cast<double>( field.cellSize_ ) * field.cellSize_;
        const auto observe = [&]( bool isU, int x, int y )
        {
            const int index = isU ? field.uIndex( x, y ) : field.vIndex( x, y );
            const float open = isU ? field.cutCellUOpen( x, y ) : field.cutCellVOpen( x, y );
            const float v = isU ? field.u_[index] : field.v_[index];
            if( !std::isfinite( open ) || open < 0.0f || open > 1.0f || !std::isfinite( v ) )
                throw std::runtime_error( key + " invalid face velocity or aperture" );
            if( open == 0.0f ) return;
            const double mass = area * open;
            if( isU ) result.momentumX += mass * v;
            else result.momentumY += mass * v;
            result.energy += 0.5 * mass * static_cast<double>( v ) * v;
            const float magnitude = std::abs( v );
            result.weightedPeak = std::max( result.weightedPeak, open * magnitude );
            if( magnitude > result.peak || result.peakX < 0 )
            {
                result.peak = magnitude;
                result.peakIsU = isU;
                result.peakX = x; result.peakY = y; result.peakOpen = open;
            }
            if( open <= 0.01f )
            {
                ++result.narrowCount;
                result.narrowPeak = std::max( result.narrowPeak, magnitude );
            }
            if( open <= 0.001f )
            {
                ++result.tinyCount;
                result.tinyPeak = std::max( result.tinyPeak, magnitude );
            }
            if( beforeU && beforeV )
            {
                const float before = isU ? ( *beforeU )[index] : ( *beforeV )[index];
                const float delta = std::abs( v - before );
                if( delta > result.deltaPeak || result.deltaX < 0 )
                {
                    result.deltaPeak = delta;
                    result.deltaIsU = isU; result.deltaX = x; result.deltaY = y;
                }
            }
        };
        for( int y = 0; y < field.height_; ++y )
            for( int x = 1; x < field.width_; ++x ) observe( true, x, y );
        for( int y = 1; y < field.height_; ++y )
            for( int x = 0; x < field.width_; ++x ) observe( false, x, y );
        if( result.peakX < 0 || !std::isfinite( result.energy ) ||
            !std::isfinite( result.momentumX ) || !std::isfinite( result.momentumY ) )
            throw std::runtime_error( key + " invalid open-face ledger" );
        const auto scalar = [&]( const char* name, auto value )
        {
            appendScalar( trace, key + name, value );
        };
        scalar( ".momentum.x", result.momentumX );
        scalar( ".momentum.y", result.momentumY );
        scalar( ".energy", result.energy );
        scalar( ".peak", result.peak );
        scalar( ".weightedPeak", result.weightedPeak );
        scalar( ".peakIsU", static_cast<int>( result.peakIsU ) );
        scalar( ".peakX", result.peakX );
        scalar( ".peakY", result.peakY );
        scalar( ".peakOpen", result.peakOpen );
        scalar( ".openLe1Percent.count", result.narrowCount );
        scalar( ".openLe1Percent.peak", result.narrowPeak );
        scalar( ".openLePoint1Percent.count", result.tinyCount );
        scalar( ".openLePoint1Percent.peak", result.tinyPeak );
        if( beforeU && beforeV )
        {
            if( result.deltaX < 0 ) throw std::runtime_error( key + " no projected face" );
            const auto captureFace = [&]( const std::string& prefix, bool isU, int x, int y )
            {
                const int index = isU ? field.uIndex( x, y ) : field.vIndex( x, y );
                const int lower = field.cellIndex( x - ( isU ? 1 : 0 ), y - ( isU ? 0 : 1 ) );
                const int upper = field.cellIndex( x, y );
                const float open = isU ? field.cutCellUOpen( x, y ) : field.cutCellVOpen( x, y );
                const float before = isU ? ( *beforeU )[index] : ( *beforeV )[index];
                const float after = isU ? field.u_[index] : field.v_[index];
                const auto emit = [&]( const char* name, auto value )
                {
                    appendScalar( trace, key + prefix + name, value );
                };
                emit( ".isU", static_cast<int>( isU ) );
                emit( ".x", x ); emit( ".y", y ); emit( ".open", open );
                emit( ".before", before ); emit( ".after", after );
                emit( ".delta", after - before );
                emit( ".lowerPressure", field.pressure_[lower] );
                emit( ".upperPressure", field.pressure_[upper] );
                emit( ".pressureJump", field.pressure_[upper] - field.pressure_[lower] );
                emit( ".lowerFraction", field.cellFluidFraction_[lower] );
                emit( ".upperFraction", field.cellFluidFraction_[upper] );
                emit( ".lowerVolumeRate", field.cutCellVolumeRate_[lower] );
                emit( ".upperVolumeRate", field.cutCellVolumeRate_[upper] );
            };
            captureFace( ".peakFace", result.peakIsU, result.peakX, result.peakY );
            captureFace( ".deltaFace", result.deltaIsU, result.deltaX, result.deltaY );
            float minimum = std::numeric_limits<float>::max();
            float maximum = -minimum;
            for( std::size_t i = 0; i < field.pressure_.size(); ++i )
            {
                if( field.cellFluidFraction_[i] <= 0.0f ) continue;
                const float pressure = field.pressure_[i];
                if( !std::isfinite( pressure ) )
                    throw std::runtime_error( key + " nonfinite active-cell pressure" );
                minimum = std::min( minimum, pressure );
                maximum = std::max( maximum, pressure );
            }
            scalar( ".pressureMin", minimum );
            scalar( ".pressureMax", maximum );
            const auto& p = field.getCutCellPressure();
            scalar( ".pressureResidual", p.pressureResidual );
            scalar( ".predictedContinuityRms", p.predictedContinuityRms );
            scalar( ".hasAnalyticImpulseBudget", static_cast<int>( p.hasAnalyticImpulseBudget ) );
            // Only valid when the flag is set; always emit fixed schema.
            const auto vec = [&]( const std::string& name, vec2 value )
            {
                if( !std::isfinite( value.x ) || !std::isfinite( value.y ) )
                    throw std::runtime_error( key + " nonfinite pressure impulse ledger" );
                appendScalar( trace, key + name + ".x", value.x );
                appendScalar( trace, key + name + ".y", value.y );
            };
            vec( ".impulse.fluid", p.impulse.fluid );
            vec( ".impulse.solid", p.impulse.solid );
            vec( ".impulse.wall", p.impulse.wall );
            vec( ".impulse.residual", p.impulse.residual() );
        }
        return result;
    }

    static void captureProjection( const fluidField2& field, std::string& trace, const std::string& key )
    {
        appendVector( trace, key + ".u", field.u_ );
        appendVector( trace, key + ".v", field.v_ );
        appendVector( trace, key + ".pressure", field.pressure_ );
        appendVector( trace, key + ".divergence", field.divergence_ );
        appendVector( trace, key + ".volumeRate", field.cutCellVolumeRate_ );
    }

    static void captureTargetV( const fluidField2& field, std::string& trace,
        const std::string& key, int x, int y )
    {
        appendScalar( trace, key + ".raw", field.v_[field.vIndex( x, y )] );
        appendScalar( trace, key + ".open", field.cutCellVOpen( x, y ) );
    }

    static void capturePrePressure( const fluidField2& field,
        std::vector<float>& u, std::vector<float>& v )
    {
        u = field.u_; v = field.v_;
    }

    static void capture( const fluidField2& field, std::string& trace, const std::string& key )
    {
        appendVector( trace, key + ".u", field.u_ );
        appendVector( trace, key + ".v", field.v_ );
        appendVector( trace, key + ".pressure", field.pressure_ );
        appendVector( trace, key + ".divergence", field.divergence_ );
        appendVector( trace, key + ".tracerMass", field.tracerMass_ );
        appendVector( trace, key + ".solidOwner", field.solidOwner_ );
        appendVector( trace, key + ".previousSolidOwner", field.previousSolidOwner_ );
        appendVector( trace, key + ".fluidFraction", field.cellFluidFraction_ );
        appendVector( trace, key + ".previousFluidFraction", field.previousCellFluidFraction_ );
        appendVector( trace, key + ".uOpen", field.uOpenFraction_ );
        appendVector( trace, key + ".vOpen", field.vOpenFraction_ );
        appendVector( trace, key + ".uSolidVelocity", field.uSolidVelocity_ );
        appendVector( trace, key + ".vSolidVelocity", field.vSolidVelocity_ );
        appendVector( trace, key + ".pressureUSolidFlux", field.pressureUSolidFlux_ );
        appendVector( trace, key + ".pressureVSolidFlux", field.pressureVSolidFlux_ );
        appendVector( trace, key + ".volumeRate", field.cutCellVolumeRate_ );
        appendVector( trace, key + ".sweptUVolume", field.sweptUVolume_ );
        appendVector( trace, key + ".sweptVVolume", field.sweptVVolume_ );

        // Geometry and temporal history influence the following pressure solve.
        appendScalar( trace, key + ".hasCutCellHistory", static_cast<int>( field.hasCutCellHistory_ ) );
        appendScalar( trace, key + ".volumeSourceValid", static_cast<int>( field.volumeSourceValid_ ) );
        appendScalar( trace, key + ".solidCellCount", field.solidCellCount_ );
        appendScalar( trace, key + ".obstacleCount", static_cast<int>( field.previousObstacles_.size() ) );
        for( std::size_t i = 0; i < field.previousObstacles_.size(); ++i )
        {
            const auto& obstacle = field.previousObstacles_[i];
            const std::string prefix = key + ".previousObstacle[" + std::to_string( i ) + "]";
            appendScalar( trace, prefix + ".position.x", obstacle.transform.position.x );
            appendScalar( trace, prefix + ".position.y", obstacle.transform.position.y );
            appendScalar( trace, prefix + ".rotation.c", obstacle.transform.rotation.c );
            appendScalar( trace, prefix + ".rotation.s", obstacle.transform.rotation.s );
            appendScalar( trace, prefix + ".center.x", obstacle.center.x );
            appendScalar( trace, prefix + ".center.y", obstacle.center.y );
            appendScalar( trace, prefix + ".linearVelocity.x", obstacle.linearVelocity.x );
            appendScalar( trace, prefix + ".linearVelocity.y", obstacle.linearVelocity.y );
            appendScalar( trace, prefix + ".angularVelocity", obstacle.angularVelocity );
        }

        // These diagnostics use no wall clock and help locate pressure divergence.
        appendScalar( trace, key + ".divergenceBefore", field.cutCellPressure_.divergenceBefore );
        appendScalar( trace, key + ".divergenceAfter", field.cutCellPressure_.divergenceAfter );
        appendScalar( trace, key + ".pressureResidual", field.cutCellPressure_.pressureResidual );
        appendScalar( trace, key + ".sourceImbalance", field.cutCellPressure_.sourceImbalance );
        appendScalar( trace, key + ".connectedRegions", field.cutCellPressure_.connectedRegions );
    }
};

} // namespace zonai

namespace
{

constexpr float timeStep = 1.0f / 60.0f;
constexpr int tickCount = 32;

void check( bool condition, const char* message )
{
    if( !condition ) throw std::runtime_error( message );
}

struct fluidScene
{
    zonai::fluidField2 field{ 16, 12, 0.25f, { -2.0f, -1.5f } };
    zonai::fluidShapeObstacle2 obstacle{};
    std::array<zonai::fluidObstacleForce2, 1> feedback{};
    bool sawFractionalCell = false;
    bool sawFlow = false;
    bool traceStages = true;
    int capturedStages = 0;
    std::string stageTrace{};
    std::string advectionTrace{};
    float advectionPreviewU = 0.0f;

    explicit fluidScene( bool analytic, bool enableStages = true )
        : traceStages( enableStages )
    {
        field.setCutCellProjectionEnabled( true );
        field.setAnalyticCircleCutCellEnabled( analytic );
        field.setTracerConcentration( 3, 6, 1.5f );
        field.addVelocity( { -0.75f, 0.0f }, { 1.25f, 0.3f }, 0.85f );

        obstacle.geometry = zonai::circle2{ {}, 0.53f };
        obstacle.transform.position = { 0.12f, 0.08f };
        obstacle.center = obstacle.transform.position;
        obstacle.linearVelocity = { 0.14f, -0.06f };
    }

    void advance( int tick, const char* sceneName )
    {
        if( tick % 6 == 0 )
            field.addVelocity( { -0.7f, 0.05f }, { 0.15f, -0.04f }, 0.6f );

        obstacle.transform.position += timeStep * obstacle.linearVelocity;
        obstacle.center = obstacle.transform.position;
        if( traceStages && tick == 1 )
        {
            const std::string prefix = std::string( sceneName ) + ".tick.1.stage.";
            const std::string probePrefix = std::string( sceneName ) + ".tick.1.advectionProbe";
            zonai::fluidStateTraceProbe2::setStageObserver( field,
                [this, prefix, probePrefix]( const zonai::fluidField2& snapshot, int phase )
                {
                    static constexpr std::array<const char*, 10> names{
                        "00-entry", "01-occupancy", "02-geometry",
                        "03-swept", "04-remap", "05-firstBoundary",
                        "06-advection", "07-secondBoundary",
                        "08-pressure", "09-tracer"
                    };
                    check( phase >= 0 && phase < static_cast<int>( names.size() ),
                        "Fluid emitted an unknown Step phase" );
                    if( phase == 5 )
                        advectionPreviewU = zonai::fluidStateTraceProbe2::captureUAdvectionProbe(
                            snapshot, timeStep, advectionTrace, probePrefix );
                    if( phase == 6 )
                    {
                        // Observe whether a read-only evaluation actually reproduces
                        // the real Step's U(3,4) output in this compiler.
                        const float real = zonai::fluidStateTraceProbe2::readTargetU( snapshot );
                        zonai::fluidStateTraceProbe2::appendScalar( advectionTrace,
                            probePrefix + ".previewMatchesActual", static_cast<int>(
                                std::bit_cast<std::uint32_t>( real ) ==
                                std::bit_cast<std::uint32_t>( advectionPreviewU ) ) );
                        zonai::fluidStateTraceProbe2::appendScalar( advectionTrace,
                            probePrefix + ".actualU", real );
                        check( std::bit_cast<std::uint32_t>( real ) ==
                            std::bit_cast<std::uint32_t>( advectionPreviewU ),
                            "advection probe differs from production Step" );
                    }
                    zonai::fluidStateTraceProbe2::capture( snapshot, stageTrace,
                        prefix + names[phase] );
                    ++capturedStages;
                } );
        }
        field.step( timeStep, std::span<const zonai::fluidShapeObstacle2>( &obstacle, 1 ),
            feedback, 60 );
        if( traceStages && tick == 1 )
        {
            zonai::fluidStateTraceProbe2::setStageObserver( field, {} );
            check( capturedStages == 10, "Fluid Stage hook did not visit all ten phases" );
        }

        for( int y = 0; y < field.getHeight(); ++y )
            for( int x = 0; x < field.getWidth(); ++x )
            {
                const float fraction = field.getCellFluidFraction( x, y );
                sawFractionalCell |= fraction > 0.0f && fraction < 1.0f;
            }

        const auto audit = field.getFlowAudit( timeStep );
        sawFlow |= audit.maxFluidFaceSpeed > 0.01f;
        check( audit.nonFiniteFluidFaces == 0 && std::isfinite( audit.continuityRms ),
            "Fluid state contains non-finite face velocity or continuity" );
    }
};

std::string runScene( const char* sceneName, bool analytic )
{
    fluidScene first{ analytic };
    fluidScene second{ analytic };
    fluidScene control{ analytic, false };
    std::string trace;
    std::string secondTrace;
    std::string controlTrace;

    for( int tick = 0; tick <= tickCount; ++tick )
    {
        if( tick != 0 )
        {
            first.advance( tick, sceneName );
            second.advance( tick, sceneName );
            control.advance( tick, sceneName );
        }

        if( tick == 1 )
        {
            check( first.stageTrace == second.stageTrace &&
                first.advectionTrace == second.advectionTrace,
                "two independently instrumented Fields differ within one process" );
            // Compare the detailed backtrace/stencil before Stage-06 raw grid.
            trace.append( first.advectionTrace );
            trace.append( first.stageTrace );
        }

        const std::string prefix = std::string( sceneName ) + ".tick." + std::to_string( tick );
        const std::size_t firstStart = trace.size();
        zonai::fluidStateTraceProbe2::capture( first.field, trace, prefix );
        secondTrace.clear();
        zonai::fluidStateTraceProbe2::capture( second.field, secondTrace, prefix );
        check( trace.size() - firstStart == secondTrace.size() &&
            trace.compare( firstStart, secondTrace.size(), secondTrace ) == 0,
            "two independent Fluid simulations diverged in one process" );
        controlTrace.clear();
        zonai::fluidStateTraceProbe2::capture( control.field, controlTrace, prefix );
        check( trace.size() - firstStart == controlTrace.size() &&
            trace.compare( firstStart, controlTrace.size(), controlTrace ) == 0,
            "Fluid Stage observer perturbed authoritative field state" );

        zonai::fluidStateTraceProbe2::appendScalar( trace, prefix + ".force.x", first.feedback[0].force.x );
        zonai::fluidStateTraceProbe2::appendScalar( trace, prefix + ".force.y", first.feedback[0].force.y );
        zonai::fluidStateTraceProbe2::appendScalar( trace, prefix + ".torque", first.feedback[0].torque );
        const auto& force = first.feedback[0];
        const auto sameFeedback = [&]( const zonai::fluidObstacleForce2& other )
        {
            return std::bit_cast<std::uint32_t>( force.force.x ) ==
                    std::bit_cast<std::uint32_t>( other.force.x ) &&
                std::bit_cast<std::uint32_t>( force.force.y ) ==
                    std::bit_cast<std::uint32_t>( other.force.y ) &&
                std::bit_cast<std::uint32_t>( force.torque ) ==
                    std::bit_cast<std::uint32_t>( other.torque );
        };
        check( std::isfinite( force.force.x ) && std::isfinite( force.force.y ) &&
            std::isfinite( force.torque ), "non-finite Fluid pressure feedback" );
        check( sameFeedback( second.feedback[0] ) && sameFeedback( control.feedback[0] ),
            "Fluid pressure feedback diverged in one process" );
    }

    check( first.sawFractionalCell && second.sawFractionalCell && control.sawFractionalCell &&
        first.sawFlow && second.sawFlow && control.sawFlow,
        "Fluid fixture did not exercise Cut-cell and flow" );
    return trace;
}

// R3h: keep the original R3c-g fixtures byte-for-byte. New scenes
// exercise independent geometry/pressure paths and feed coupled pressure
// feedback into obstacle snapshots, without pretending to be World::Step.
enum class geometrySceneKind
{
    sampledMixed,
    analyticCapsule,
    analyticPolygon,
    coupledDisjoint
};

struct geometryScene
{
    zonai::fluidField2 field{ 24, 16, 0.5f, { -6.0f, -4.0f } };
    std::vector<zonai::fluidShapeObstacle2> obstacles;
    std::vector<zonai::fluidObstacleForce2> feedback;
    std::vector<float> angles;
    geometrySceneKind kind;
    bool observed;
    bool sawFractionalCell = false;
    bool sawAnalyticFraction = false;
    bool sawSwept = false;
    bool sawFlow = false;
    bool sawFeedback = false;
    bool sawCoupled = false;
    bool sawIc0 = false;
    bool sawDisjoint = false;
    std::string stages;

    explicit geometryScene( geometrySceneKind sceneKind, bool observe )
        : kind( sceneKind ), observed( observe )
    {
        using namespace zonai;
        field.setCutCellProjectionEnabled( true );
        field.setCutCellSampleCount( 4 );
        field.setAnalyticCapsuleCutCellEnabled( kind == geometrySceneKind::analyticCapsule );
        field.setAnalyticPolygonCutCellEnabled( kind == geometrySceneKind::analyticPolygon );
        field.setDisjointAnalyticCutCellEnabled( kind == geometrySceneKind::coupledDisjoint );
        field.setCutCellCoupledProjectionEnabled( kind == geometrySceneKind::coupledDisjoint );
        field.setTracerConcentration( 8, 7, 1.0f );
        field.addVelocity( { -1.4f, 0.2f }, { 1.2f, -0.1f }, 1.5f );

        const auto add = [&]( shapeGeometry geometry, vec2 center, float angle,
            vec2 velocity, float omega, float inverseMass = 0.0f, float inverseInertia = 0.0f )
        {
            obstacles.push_back( { std::move( geometry ), { center, rot2::FromRadians( angle ) },
                center, velocity, omega, inverseMass, inverseInertia } );
            angles.push_back( angle );
        };

        if( kind == geometrySceneKind::analyticCapsule )
            add( capsule2{ { -0.70f, 0.0f }, { 0.70f, 0.0f }, 0.30f },
                { 0.13f, -0.17f }, 0.19f, { 0.16f, -0.07f }, 0.31f );
        else if( kind == geometrySceneKind::analyticPolygon )
            add( MakeBox( { 0.63f, 0.33f } ), { 0.13f, -0.20f },
                0.41f, { 0.15f, 0.08f }, -0.29f );
        else
        {
            const bool coupled = kind == geometrySceneKind::coupledDisjoint;
            polygon2 rounded = MakeBox( { 0.34f, 0.29f } );
            rounded.radius = 0.17f;
            add( circle2{ {}, 0.42f }, { -3.4f, -1.15f }, 0.0f, {}, 0.0f );
            add( capsule2{ { -0.4f, 0.0f }, { 0.4f, 0.0f }, 0.23f },
                { -1.5f, 1.1f }, 0.11f, { 0.16f, -0.07f }, 0.21f,
                coupled ? 0.5f : 0.0f, coupled ? 1.0f : 0.0f );
            add( MakeBox( { 0.46f, 0.31f } ), { 1.4f, -1.1f }, 0.32f,
                { -0.08f, 0.12f }, -0.17f,
                coupled ? 0.4f : 0.0f, coupled ? 0.8f : 0.0f );
            add( rounded, { 3.65f, 1.0f }, 0.21f, {}, 0.0f );
        }
        feedback.resize( obstacles.size() );
    }

    void capture( std::string& trace, const std::string& key ) const
    {
        using probe = zonai::fluidStateTraceProbe2;
        const auto scalar = [&]( const char* name, auto value )
        {
            probe::appendScalar( trace, key + name, value );
        };
        probe::capture( field, trace, key );
        const auto& geometry = field.getCutCellTransition();
        const auto& swept = field.getSweptFaceFlux();
        const auto& pressure = field.getCutCellPressure();
        scalar( ".geometry.disjoint", static_cast<int>( geometry.usedDisjointAnalyticGeometry ) );
        scalar( ".geometry.partialCells", geometry.partialCells );
        scalar( ".geometry.disjointShapes", geometry.disjointAnalyticShapes );
        scalar( ".swept.used", static_cast<int>( swept.usedSweptFlux ) );
        scalar( ".swept.samples", swept.timeSamples );
        scalar( ".swept.mismatch", swept.geometryMismatch );
        scalar( ".pcg.used", static_cast<int>( pressure.usedCoupledProjection ) );
        scalar( ".pcg.failed", static_cast<int>( pressure.coupledSolveFailed ) );
        scalar( ".pcg.status", static_cast<int>( pressure.coupledStatus ) );
        scalar( ".pcg.iterations", pressure.coupledIterations );
        scalar( ".pcg.relativeResidual", pressure.coupledRelativeResidual );
        scalar( ".pcg.ic0", static_cast<int>( pressure.usedIncompleteCholesky ) );
        scalar( ".pcg.synchronizedPartialFaces", pressure.synchronizedPartialFaces );
        scalar( ".pcg.predictedContinuityRms", pressure.predictedContinuityRms );

        for( std::size_t i = 0; i < obstacles.size(); ++i )
        {
            const auto& body = obstacles[i];
            const auto& reaction = feedback[i];
            const std::string prefix = key + ".body[" + std::to_string( i ) + "]";
            probe::appendScalar( trace, prefix + ".position.x", body.transform.position.x );
            probe::appendScalar( trace, prefix + ".position.y", body.transform.position.y );
            probe::appendScalar( trace, prefix + ".rotation.c", body.transform.rotation.c );
            probe::appendScalar( trace, prefix + ".rotation.s", body.transform.rotation.s );
            probe::appendScalar( trace, prefix + ".angle", angles[i] );
            probe::appendScalar( trace, prefix + ".velocity.x", body.linearVelocity.x );
            probe::appendScalar( trace, prefix + ".velocity.y", body.linearVelocity.y );
            probe::appendScalar( trace, prefix + ".omega", body.angularVelocity );
            probe::appendScalar( trace, prefix + ".inverseMass", body.inverseMass );
            probe::appendScalar( trace, prefix + ".inverseInertia", body.inverseInertia );
            probe::appendScalar( trace, prefix + ".force.x", reaction.force.x );
            probe::appendScalar( trace, prefix + ".force.y", reaction.force.y );
            probe::appendScalar( trace, prefix + ".torque", reaction.torque );
        }
    }

    [[nodiscard]] bool traceTick( int tick ) const
    {
        // Retain R3h's first two ticks and extend through each scene's
        // first mismatching end-of-tick snapshot. Earlier intermediate
        // differences could have been invisible in the old end snapshots.
        const int last = kind == geometrySceneKind::sampledMixed ? 4 :
            kind == geometrySceneKind::analyticCapsule ? 3 :
            kind == geometrySceneKind::analyticPolygon ? 2 : 5;
        return observed && tick <= last;
    }

    void advance( int tick, const std::string& key )
    {
        using namespace zonai;
        if( tick % 4 == 0 )
            field.addVelocity( { -1.4f, 0.2f }, { 0.13f, -0.04f }, 1.5f );
        for( std::size_t i = 0; i < obstacles.size(); ++i )
        {
            auto& body = obstacles[i];
            body.transform.position += timeStep * body.linearVelocity;
            body.center = body.transform.position;
            angles[i] += timeStep * body.angularVelocity;
            body.transform.rotation = rot2::FromRadians( angles[i] );
        }
        stages.clear();
        int observedStages = 0;
        if( traceTick( tick ) )
        {
            fluidStateTraceProbe2::setStageObserver( field,
                [&]( const fluidField2& snapshot, int phase )
                {
                    check( phase == observedStages, "R3h phase order changed" );
                    if( phase == 3 )
                        fluidStateTraceProbe2::captureSweptRotation( snapshot, obstacles,
                            timeStep, stages, key + ".sweptProbe" );
                    capture( stages, key + ".stage." + std::to_string( phase ) );
                    ++observedStages;
                } );
        }

        field.step( timeStep, obstacles, feedback, 200 );
        fluidStateTraceProbe2::setStageObserver( field, {} );
        if( traceTick( tick ) )
            check( observedStages == 10, "R3i missed a stage" );

        for( int y = 0; y < field.getHeight(); ++y )
            for( int x = 0; x < field.getWidth(); ++x )
            {
                const float fraction = field.getCellFluidFraction( x, y );
                check( std::isfinite( fraction ) && fraction >= 0.0f && fraction <= 1.0f,
                    "R3h invalid fluid fraction" );
                sawFractionalCell |= fraction > 0.0f && fraction < 1.0f;
                // Four-by-four sampled geometry has multiples of 1/16.
                sawAnalyticFraction |= fraction * 16.0f != std::floor( fraction * 16.0f );
            }

        const auto audit = field.getFlowAudit( timeStep );
        check( audit.nonFiniteFluidFaces == 0 && std::isfinite( audit.continuityRms ),
            "R3h non-finite velocity or continuity" );
        sawFlow |= audit.maxFluidFaceSpeed > 0.01f;
        sawSwept |= field.getSweptFaceFlux().usedSweptFlux;
        const auto& pressure = field.getCutCellPressure();
        sawDisjoint |= field.getCutCellTransition().usedDisjointAnalyticGeometry;
        sawCoupled |= pressure.usedCoupledProjection &&
            pressure.coupledStatus == fluidCoupledPressureStatus2::converged;
        sawIc0 |= pressure.usedCoupledProjection && pressure.usedIncompleteCholesky;
        for( std::size_t i = 0; i < obstacles.size(); ++i )
        {
            auto& body = obstacles[i];
            const auto& reaction = feedback[i];
            check( std::isfinite( reaction.force.x ) && std::isfinite( reaction.force.y ) &&
                std::isfinite( reaction.torque ), "R3h non-finite pressure feedback" );
            if( body.inverseMass > 0.0f )
            {
                sawFeedback |= reaction.force.x != 0.0f || reaction.force.y != 0.0f ||
                    reaction.torque != 0.0f;
                body.linearVelocity += ( timeStep * body.inverseMass ) * reaction.force;
                body.angularVelocity += timeStep * body.inverseInertia * reaction.torque;
                check( std::isfinite( body.linearVelocity.x ) &&
                    std::isfinite( body.linearVelocity.y ) && std::isfinite( body.angularVelocity ),
                    "R3h non-finite pressure-driven body velocity" );
            }
        }
    }

    void verifyCoverage() const
    {
        check( sawFractionalCell && sawFlow && sawSwept,
            "R3h fixture must exercise partial cells, flow and swept history" );
        if( kind != geometrySceneKind::sampledMixed )
            check( sawAnalyticFraction, "R3h analytic geometry fell back to sampled cells" );
        if( kind == geometrySceneKind::coupledDisjoint )
            check( sawDisjoint && sawCoupled && sawIc0 && sawFeedback,
                "R3h fixture must exercise analytic Schur IC0 and nonzero feedback" );
    }
};

std::string runGeometryScene( const char* name, geometrySceneKind kind )
{
    constexpr int geometryTickCount = 12;
    geometryScene first{ kind, true };
    geometryScene control{ kind, false };
    std::string trace;
    for( int tick = 0; tick <= geometryTickCount; ++tick )
    {
        const std::string key = std::string( name ) + ".tick." + std::to_string( tick );
        if( tick != 0 )
        {
            first.advance( tick, key );
            control.advance( tick, key );
            trace.append( first.stages );
        }
        std::string a, b;
        first.capture( a, key );
        control.capture( b, key );
        check( a == b, "R3h independent observed/control state or feedback differs" );
        trace.append( a );
    }
    first.verifyCoverage();
    control.verifyCoverage();
    return trace;
}

// R3m uses the real headless Sandbox interleave, not a duplicated FSI
// integrator. Keep the standalone Fluid fixtures and their keys unchanged.
enum class fluidJointKind { none, revolute, weld, anchoredAssembly };

struct worldFluidScene
{
    static constexpr int worldSubsteps = 8;
    static constexpr int fluidSubsteps = 4;
    zonai::sandbox::rigidBodyDemo demo{ zonai::sandbox::demoKind::windFlow };
    std::vector<zonai::bodyId> bodies;
    bool dragEnabled;
    fluidJointKind jointKind;
    zonai::jointId joint{};
    zonai::jointId anchorJoint{};
    zonai::bodyId anchorBody{};
    bool contactScenario = false;

    explicit worldFluidScene( bool drag, fluidJointKind kind = fluidJointKind::none,
        bool contact = false )
        : dragEnabled( drag ), jointKind( kind ), contactScenario( contact )
    {
        demo.configureWindField( 24, 14, 0.5f );
        demo.setWindCutCellProjectionEnabled( true );
        demo.setWindCutCellCoupledProjectionEnabled( true );
        demo.setWindDisjointAnalyticEnabled( true );
        demo.setWindObstacleCouplingEnabled( true );
        demo.setWindSynchronizedCouplingEnabled( true );
        demo.setWindFluidSubstepCount( fluidSubsteps );
        demo.setWindSourceEnabled( true );
        demo.setWindSourceStrength( 8.0f );
        demo.setWindDragCoefficient( drag ? 3.0f : 0.0f );
        demo.setWindStepProfilingEnabled( false );
        demo.setWindCouplingAuditEnabled( false );
        for( const auto& shape : demo.getShapes() ) bodies.push_back( shape.bodyHandle );
        check( bodies.size() == 3, "R3m expected three Sandbox bodies" );
        demo.getWorld().SetBodyLinearVelocity( bodies[0], { 0.7f, -0.10f } );
        demo.getWorld().SetBodyAngularVelocity( bodies[1], 0.5f );
        if( contactScenario )
        {
            check( jointKind == fluidJointKind::anchoredAssembly && drag,
                "R3u contact scene requires the anchored assembly and two-way drag" );
            // Capsule right extent initially x=1.1, box left extent x=1.19.
            // A closing velocity and a small tangential component exercise a
            // genuine Contact solve without starting with overlapping solids.
            demo.getWorld().SetBodyTransform( bodies[2], { { 1.64f, 1.0f }, {} } );
            demo.getWorld().SetBodyLinearVelocity( bodies[2], { -1.5f, 0.15f } );
        }

        // Join two dynamic obstacles, with coincident initial world anchors.
        // Neither geometry nor fluid coupling is replaced by a special fixture.
        auto& world = demo.getWorld();
        if( jointKind != fluidJointKind::none )
        {
            const auto a = world.GetBodyTransform( bodies[0] );
            const auto b = world.GetBodyTransform( bodies[1] );
            const auto anchor = 0.5f * ( a.position + b.position );
            const auto localA = zonai::InverseRotate( a.rotation, anchor - a.position );
            const auto localB = zonai::InverseRotate( b.rotation, anchor - b.position );
            if( jointKind == fluidJointKind::revolute )
            {
                zonai::revoluteJointDef def{};
                def.bodyA = bodies[0];
                def.bodyB = bodies[1];
                def.localAnchorA = localA;
                def.localAnchorB = localB;
                joint = world.createRevoluteJoint( def );
            }
            else
            {
                zonai::weldJointDef def{};
                def.bodyA = bodies[0];
                def.bodyB = bodies[1];
                def.localAnchorA = localA;
                def.localAnchorB = localB;
                joint = world.createWeldJoint( def );
            }
            if( jointKind == fluidJointKind::anchoredAssembly )
            {
                // The original joint is a Weld. Attach its circle to a
                // shapeless static Body with a limited Revolute Joint.
                // The anchor is a World constraint, not a Fluid obstacle.
                anchorBody = world.CreateBody( zonai::bodyType::Static, { a.position, {} } );
                zonai::revoluteJointDef def{};
                def.bodyA = anchorBody;
                def.bodyB = bodies[0];
                def.enableLimit = true;
                def.lowerAngle = -0.35f;
                def.upperAngle = 0.35f;
                anchorJoint = world.createRevoluteJoint( def );
                check( world.IsValid( anchorBody ) && world.IsValid( anchorJoint ),
                    "R3t expected a valid static pivot" );
            }
            check( world.IsValid( joint ) &&
                world.getJointCount() == ( jointKind == fluidJointKind::anchoredAssembly ? 2u : 1u ),
                "expected valid dynamic-to-dynamic Joint and optional pivot" );
        }
    }

    static void appendSnapshot( std::string& trace, const std::string& key,
        const zonai::determinism::test::stateSnapshot& snapshot )
    {
        for( const auto& field : snapshot.fields )
        {
            char bits[32]{};
            std::snprintf( bits, sizeof( bits ), "%0*llx/%u",
                static_cast<int>( field.byteCount ) * 2,
                static_cast<unsigned long long>( field.bits ), static_cast<unsigned>( field.byteCount ) );
            trace.append( key ).append( "." ).append( field.name )
                .append( "\t" ).append( bits ).append( "\n" );
        }
    }

    void captureJoint( std::string& trace, const std::string& key ) const
    {
        if( jointKind == fluidJointKind::none ) return;
        using probe = zonai::fluidStateTraceProbe2;
        const auto capture = [&]( const auto& data, const std::string& prefix )
        {
            const auto scalar = [&]( const char* name, float value )
            {
                check( std::isfinite( value ), "R3n nonfinite Joint reaction/anchor" );
                probe::appendScalar( trace, prefix + name, value );
            };
            scalar( ".anchorA.x", data.anchorA.x );
            scalar( ".anchorA.y", data.anchorA.y );
            scalar( ".anchorB.x", data.anchorB.x );
            scalar( ".anchorB.y", data.anchorB.y );
            scalar( ".angle", data.currentAngle );
            scalar( ".force.x", data.force.x );
            scalar( ".force.y", data.force.y );
            scalar( ".torque", data.torque );
        };
        if( jointKind == fluidJointKind::revolute )
            capture( demo.getWorld().getRevoluteJointData( joint ), key );
        else capture( demo.getWorld().getWeldJointData( joint ), key );
        if( jointKind == fluidJointKind::anchoredAssembly )
        {
            capture( demo.getWorld().getRevoluteJointData( anchorJoint ), key + ".pivot" );
            const std::array<zonai::jointId, 2> joints{ joint, anchorJoint };
            appendSnapshot( trace, key,
                zonai::determinism::test::jointCacheSnapshotAccess::capture( demo.getWorld(), joints ) );
        }
        else
        {
            const std::array<zonai::jointId, 1> joints{ joint };
            appendSnapshot( trace, key,
                zonai::determinism::test::jointCacheSnapshotAccess::capture( demo.getWorld(), joints ) );
        }
    }

    bool hasJointReaction() const
    {
        if( jointKind == fluidJointKind::none ) return false;
        const auto hasReaction = []( const auto& data )
        {
            return data.force.x != 0.0f || data.force.y != 0.0f || data.torque != 0.0f;
        };
        if( jointKind == fluidJointKind::revolute )
            return hasReaction( demo.getWorld().getRevoluteJointData( joint ) );
        return hasReaction( demo.getWorld().getWeldJointData( joint ) );
    }

    float domainMargin() const
    {
        const auto& field = *demo.getWindField();
        const auto lower = field.getOrigin();
        const auto upper = lower + field.getCellSize() *
            zonai::vec2{ static_cast<float>( field.getWidth() ), static_cast<float>( field.getHeight() ) };
        float margin = std::numeric_limits<float>::max();
        for( const auto& visual : demo.getShapes() )
        {
            // Recompute geometric bounds, not the speculative broad-phase AABB.
            const auto box = zonai::ComputeShapeAABB(
                demo.getWorld().GetShape( visual.shapeHandle ).geometry,
                demo.getWorld().GetBodyTransform( visual.bodyHandle ) );
            check( zonai::IsValidAABB( box ), "R3t invalid geometric bounds" );
            margin = std::min( { margin, box.min.x - lower.x, box.min.y - lower.y,
                upper.x - box.max.x, upper.y - box.max.y } );
        }
        return margin;
    }

    void captureWorld( std::string& trace, const std::string& key, int tick ) const
    {
        auto tracked = bodies;
        if( jointKind == fluidJointKind::anchoredAssembly ) tracked.push_back( anchorBody );
        const auto snapshot = zonai::determinism::test::captureWorldState(
            demo.getWorld(), tracked, static_cast<std::uint64_t>( tick ), timeStep, worldSubsteps );
        appendSnapshot( trace, key + ".world", snapshot );
        captureJoint( trace, key + ".joint" );
    }

    void captureFluid( std::string& trace, const std::string& key,
        const zonai::fluidField2& field ) const
    {
        using probe = zonai::fluidStateTraceProbe2;
        probe::capture( field, trace, key + ".fluid" );
        const auto& p = field.getCutCellPressure();
        const auto& g = field.getCutCellTransition();
        probe::appendScalar( trace, key + ".pcg.used", static_cast<int>( p.usedCoupledProjection ) );
        probe::appendScalar( trace, key + ".pcg.failed", static_cast<int>( p.coupledSolveFailed ) );
        probe::appendScalar( trace, key + ".pcg.status", static_cast<int>( p.coupledStatus ) );
        probe::appendScalar( trace, key + ".pcg.iterations", p.coupledIterations );
        probe::appendScalar( trace, key + ".pcg.ic0", static_cast<int>( p.usedIncompleteCholesky ) );
        probe::appendScalar( trace, key + ".pcg.relativeResidual", p.coupledRelativeResidual );
        probe::appendScalar( trace, key + ".geometry.disjoint", static_cast<int>( g.usedDisjointAnalyticGeometry ) );
        probe::appendScalar( trace, key + ".geometry.partialCells", g.partialCells );
    }

    void captureFrame( std::string& trace, const std::string& key, int tick ) const
    {
        using probe = zonai::fluidStateTraceProbe2;
        captureWorld( trace, key, tick );
        captureFluid( trace, key, *demo.getWindField() );
        probe::appendScalar( trace, key + ".fluidSteps", demo.getWindCouplingSteps() );
        const auto vec = [&]( const std::string& name, zonai::vec2 v )
        {
            check( std::isfinite( v.x ) && std::isfinite( v.y ), "R3m nonfinite exchange" );
            probe::appendScalar( trace, key + name + ".x", v.x );
            probe::appendScalar( trace, key + name + ".y", v.y );
        };
        const auto forces = demo.getWindPressureForces();
        check( forces.size() == bodies.size(), "R3m pressure feedback count" );
        for( std::size_t i = 0; i < forces.size(); ++i )
        {
            const auto prefix = ".pressureForce[" + std::to_string( i ) + "]";
            vec( prefix, forces[i].force );
            check( std::isfinite( forces[i].torque ), "R3m nonfinite torque" );
            probe::appendScalar( trace, key + prefix + ".torque", forces[i].torque );
        }
        vec( ".dragBodyImpulse", demo.getWindDragBodyImpulse() );
        vec( ".dragFluidImpulse", demo.getWindDragFluidImpulse() );
        if( contactScenario )
        {
            using access = zonai::determinism::test::jointSubstepTraceAccess;
            appendSnapshot( trace, key + ".persistentContact",
                access::capturePersistentContacts( demo.getWorld() ) );
            appendSnapshot( trace, key + ".contactLifecycle",
                access::captureContactLifecycle( demo.getWorld() ) );
        }
        const auto flow = demo.getWindField()->getFlowAudit( timeStep / fluidSubsteps );
        check( flow.nonFiniteFluidFaces == 0 && std::isfinite( flow.continuityRms ) &&
            std::isfinite( flow.maxFluidFaceSpeed ), "R3m nonfinite live field" );
    }
};

std::string runWorldFluidScene( const char* name, bool drag,
    fluidJointKind jointKind = fluidJointKind::none, int frameCount = 12,
    const char* continuationName = nullptr, bool contactScenario = false )
{
    using probe = zonai::fluidStateTraceProbe2;
    using namespace zonai::determinism::test;
    worldFluidScene observed{ drag, jointKind, contactScenario }, control{ drag, jointKind, contactScenario };
    jointSubstepTraceRecorder jointTrace{};
    jointTrace.captureContactAndIsland = contactScenario;
    if( jointKind != fluidJointKind::none )
    {
        jointTrace.bodies = observed.bodies;
        jointTrace.joints.push_back( observed.joint );
        if( jointKind == fluidJointKind::anchoredAssembly )
        {
            jointTrace.bodies.push_back( observed.anchorBody );
            jointTrace.joints.push_back( observed.anchorJoint );
        }
    }
    bool sawJointReaction = false, sawAnchorReaction = false;
    bool sawCombinedSolve = false;
    int firstCombinedTick = -1, firstCombinedInterval = -1;
    int combinedIntervals = 0;
    const bool sustained = continuationName != nullptr || jointKind == fluidJointKind::anchoredAssembly;
    int activeIntervals = 0, exchangeFrames = 0;
    float minimumMargin = std::numeric_limits<float>::max();
    float maximumFaceSpeed = 0.0f, maximumContinuity = 0.0f;
    const auto checkDomain = [&]( const std::string& key )
    {
        const float margin = observed.domainMargin();
        if( !std::isfinite( margin ) || margin < 0.0f )
            throw std::runtime_error( key + " geometric obstacle escaped Fluid domain" );
        minimumMargin = std::min( minimumMargin, margin );
    };
    // The demo owns a non-const Field; its public getter is intentionally
    // read-only. Use the existing test-only friend observer, not a new API.
    auto& field = const_cast<zonai::fluidField2&>( *observed.demo.getWindField() );
    std::string trace;
    std::string stabilityTrace;
    std::vector<float> prePressureU, prePressureV;
    double energyBeforePressure = 0.0;
    float auditMaxSpeed = 0.0f, auditMaxPressureDelta = 0.0f;
    int auditedIntervals = 0, analyticBudgets = 0;
    // Audit the two already verified sustained fixtures, not the pending
    // R3u contact fixture. Keep their old authoritative rows unchanged.
    const bool stabilityAudit = sustained && !contactScenario;
    bool sawPressure = false, sawDragBody = false, sawDragFluid = false;
    bool sawCoupled = false, sawDisjoint = false, sawFlow = false;

    // R3s continues the same observed/control Worlds, without resetting
    // the field or source clock. Keep the first 180 ticks' trace keys intact.
    // The second name labels only the continuation, not a new simulation.
    check( ( frameCount > 180 ) == ( continuationName != nullptr ),
        "extended Fluid replay must label its continuation" );
    for( int tick = 0; tick <= frameCount; ++tick )
    {
        const char* scene = tick > 180 ? continuationName : name;
        const std::string key = std::string( scene ) + ".tick." + std::to_string( tick );
        // Around the former 2*pi limit (near tick 222), capture all ten
        // Fluid phases. This narrows a new mismatch without tracing all
        // 1,440 intervals at full-grid granularity.
        const bool traceCrossing = continuationName && tick >= 221 && tick <= 223;
        if( tick != 0 )
        {
            int calls = 0;
            std::array<bool, worldFluidScene::fluidSubsteps> coupledIntervals{};
            const bool traceJoint = jointKind != fluidJointKind::none &&
                ( tick <= 2 || contactScenario );
            if( traceJoint )
            {
                jointTrace.records.clear();
                jointSubstepTraceAccess::install( observed.demo.getWorld(), jointTrace );
            }
            probe::setStageObserver( field,
                [&]( const zonai::fluidField2& snapshot, int phase )
                {
                    check( phase == calls % 10, "R3m unexpected Fluid stage sequence" );
                    const int interval = calls / 10;
                    ++calls;
                    if( sustained && phase == 0 )
                        checkDomain( key + ".interval." + std::to_string( interval ) );
                    if( stabilityAudit && phase >= 4 && phase <= 8 )
                    {
                        const std::string auditKey = "fluidStabilityAudit." + key + ".interval." +
                            std::to_string( interval ) + ".stage." + std::to_string( phase );
                        if( phase == 7 ) probe::capturePrePressure( snapshot, prePressureU, prePressureV );
                        const auto sample = probe::captureStabilityPhase( snapshot, stabilityTrace, auditKey,
                            phase == 8 ? &prePressureU : nullptr, phase == 8 ? &prePressureV : nullptr );
                        if( phase == 7 ) energyBeforePressure = sample.energy;
                        if( phase == 8 )
                        {
                            ++auditedIntervals;
                            analyticBudgets += static_cast<int>( snapshot.getCutCellPressure().hasAnalyticImpulseBudget );
                            auditMaxSpeed = std::max( auditMaxSpeed, sample.peak );
                            auditMaxPressureDelta = std::max( auditMaxPressureDelta, sample.deltaPeak );
                            probe::appendScalar( stabilityTrace, auditKey + ".energyDelta",
                                sample.energy - energyBeforePressure );
                        }
                    }
                    // Input after source injection and output before drag/World
                    // integration, for every Fluid interval of the first two ticks.
                    if( traceCrossing || ( tick <= 2 && ( phase == 0 || phase == 9 ) ) )
                    {
                        const std::string prefix = key + ".interval." + std::to_string( interval ) +
                            ".stage." + std::to_string( phase );
                        observed.captureWorld( trace, prefix, tick );
                        observed.captureFluid( trace, prefix, snapshot );
                    }
                    // In the extended run, retain compact per-interval
                    // observations instead of multiplying every raw-grid row
                    // by the number of intervals. Full raw fields are still
                    // compared every frame.
                    if( frameCount > 12 && phase == 0 )
                    {
                        const float width = snapshot.getWidth() * snapshot.getCellSize();
                        const zonai::vec2 source{ snapshot.getOrigin().x +
                            std::min( 2.5f, 0.25f * width ), 0.0f };
                        const auto velocity = snapshot.sampleVelocity( source );
                        check( std::isfinite( velocity.x ) && std::isfinite( velocity.y ),
                            "R3o nonfinite source-region velocity" );
                        const std::string prefix = key + ".interval." + std::to_string( interval ) +
                            ".sourceSample";
                        probe::appendScalar( trace, prefix + ".x", velocity.x );
                        probe::appendScalar( trace, prefix + ".y", velocity.y );
                    }
                    if( phase == 9 )
                    {
                        const auto& pressure = snapshot.getCutCellPressure();
                        sawCoupled |= pressure.usedCoupledProjection &&
                            !pressure.coupledSolveFailed && pressure.usedIncompleteCholesky;
                        sawDisjoint |= snapshot.getCutCellTransition().usedDisjointAnalyticGeometry;
                        if( contactScenario )
                            coupledIntervals[static_cast<std::size_t>( interval )] =
                                pressure.usedCoupledProjection &&
                                pressure.coupledStatus == zonai::fluidCoupledPressureStatus2::converged;
                        if( pressure.coupledSolveFailed || !std::isfinite( pressure.pressureResidual ) )
                            throw std::runtime_error( key + ".interval." + std::to_string( interval ) +
                                " pressure solve failure, status=" +
                                std::to_string( static_cast<int>( pressure.coupledStatus ) ) );
                        if( sustained )
                        {
                            if( !pressure.usedCoupledProjection ||
                                pressure.coupledStatus != zonai::fluidCoupledPressureStatus2::converged ||
                                snapshot.getCutCellTransition().partialCells <= 0 )
                                throw std::runtime_error( key + ".interval." + std::to_string( interval ) +
                                    " sustained coupled projection coverage lost" );
                            ++activeIntervals;
                        }
                        if( frameCount > 12 )
                        {
                            const auto flow = snapshot.getFlowAudit( timeStep / worldFluidScene::fluidSubsteps );
                            if( flow.nonFiniteFluidFaces != 0 || !std::isfinite( flow.continuityRms ) ||
                                !std::isfinite( flow.maxFluidFaceSpeed ) )
                                throw std::runtime_error( key + ".interval." + std::to_string( interval ) +
                                    " nonfinite flow after projection" );
                            if( sustained )
                            {
                                maximumFaceSpeed = std::max( maximumFaceSpeed, flow.maxFluidFaceSpeed );
                                maximumContinuity = std::max( maximumContinuity, flow.continuityRms );
                            }
                            const std::string prefix = key + ".interval." + std::to_string( interval ) + ".projection";
                            probe::appendScalar( trace, prefix + ".continuityRms", flow.continuityRms );
                            probe::appendScalar( trace, prefix + ".maxFaceSpeed", flow.maxFluidFaceSpeed );
                            probe::appendScalar( trace, prefix + ".pcg.used", static_cast<int>( pressure.usedCoupledProjection ) );
                            probe::appendScalar( trace, prefix + ".pcg.status", static_cast<int>( pressure.coupledStatus ) );
                            probe::appendScalar( trace, prefix + ".pcg.iterations", pressure.coupledIterations );
                        }
                    }
                } );
            observed.demo.step( timeStep, worldFluidScene::worldSubsteps );
            probe::setStageObserver( field, {} );
            if( traceJoint )
            {
                jointSubstepTraceAccess::uninstall( observed.demo.getWorld() );
                constexpr int intervals = worldFluidScene::fluidSubsteps;
                constexpr int substeps = worldFluidScene::worldSubsteps / intervals;
                constexpr int recordsPerInterval = 3 + 5 * substeps;
                check( jointTrace.records.size() == intervals * recordsPerInterval,
                    "R3n expected Joint substep stages in all four World intervals" );
                std::array<bool, intervals> combined{};
                for( std::size_t i = 0; i < jointTrace.records.size(); ++i )
                {
                    const auto& record = jointTrace.records[i];
                    const int local = static_cast<int>( i % recordsPerInterval );
                    const auto expectedStage = local == 0 ? zonai::jointTraceStage2::prepare :
                        local == recordsPerInterval - 2 ? zonai::jointTraceStage2::restitution :
                        local == recordsPerInterval - 1 ? zonai::jointTraceStage2::store :
                        std::array<zonai::jointTraceStage2, 5>{
                            zonai::jointTraceStage2::integratedVelocity,
                            zonai::jointTraceStage2::warmStart,
                            zonai::jointTraceStage2::solveBias,
                            zonai::jointTraceStage2::integratedPosition,
                            zonai::jointTraceStage2::solveRelax
                        }[( local - 1 ) % 5];
                    const int expectedIndex = local == 0 ? -1 :
                        local >= recordsPerInterval - 2 ? substeps : ( local - 1 ) / 5;
                    check( record.stage == expectedStage && record.subStepIndex == expectedIndex,
                        "R3n unexpected World substep order" );
                    const auto prefix = key + ".jointInterval." +
                        std::to_string( i / recordsPerInterval ) + ".substep." +
                        std::to_string( record.subStepIndex ) + "." + jointTraceStageName( record.stage );
                    worldFluidScene::appendSnapshot( trace, prefix, record.snapshot );
                    if( contactScenario && record.stage == zonai::jointTraceStage2::solveBias )
                    {
                        // A positive Contact impulse and both joints in a
                        // shared island, during the interval that just
                        // received converged Fluid pressure. Check actual
                        // captured solver fields, not broad-phase pairs.
                        const auto& fields = record.snapshot.fields;
                        bool sharedImpulse = false;
                        const auto fieldValue = [&]( const std::string& name )
                        {
                            const auto item = std::find_if( fields.begin(), fields.end(),
                                [&]( const auto& value ) { return value.name == name; } );
                            check( item != fields.end(), "R3u missing island range field" );
                            return item->bits;
                        };
                        for( const auto& f : fields )
                        {
                            if( !f.name.starts_with( "island[" ) ||
                                !f.name.ends_with( ".jointCount" ) || f.bits < 2 ) continue;
                            const auto island = f.name.substr( 0, f.name.size() - 11 );
                            const auto first = fieldValue( island + ".contactStart" );
                            const auto count = fieldValue( island + ".contactCount" );
                            // Contact snapshots follow the flat island graph order.
                            for( auto c = first; c < first + count; ++c )
                            {
                                const auto contact = "contact[" + std::to_string( c ) + "].";
                                for( const auto& point : fields )
                                    if( point.name.starts_with( contact ) &&
                                        point.name.ends_with( ".normalImpulse" ) && point.byteCount == 4 )
                                        sharedImpulse |= std::bit_cast<float>(
                                            static_cast<std::uint32_t>( point.bits ) ) > 0.0f;
                            }
                        }
                        const auto interval = i / recordsPerInterval;
                        combined[interval] |= sharedImpulse && coupledIntervals[interval];
                    }
                }
                if( contactScenario )
                {
                    for( int interval = 0; interval < intervals; ++interval )
                    {
                        if( !combined[static_cast<std::size_t>( interval )] ) continue;
                        ++combinedIntervals;
                        if( !sawCombinedSolve )
                        {
                            firstCombinedTick = tick;
                            firstCombinedInterval = interval;
                        }
                        sawCombinedSolve = true;
                    }
                }
                jointTrace.records.clear();
            }
            sawJointReaction |= observed.hasJointReaction();
            if( jointKind == fluidJointKind::anchoredAssembly )
            {
                const auto reaction = observed.demo.getWorld().getRevoluteJointData( observed.anchorJoint );
                sawAnchorReaction |= reaction.force.x != 0.0f || reaction.force.y != 0.0f ||
                    reaction.torque != 0.0f;
            }
            control.demo.step( timeStep, worldFluidScene::worldSubsteps );
            check( calls == 10 * worldFluidScene::fluidSubsteps &&
                observed.demo.getWindCouplingSteps() == worldFluidScene::fluidSubsteps &&
                control.demo.getWindCouplingSteps() == worldFluidScene::fluidSubsteps,
                "R3m expected four actual Fluid/World intervals" );
            for( const auto& force : observed.demo.getWindPressureForces() )
                sawPressure |= force.force.x != 0.0f || force.force.y != 0.0f || force.torque != 0.0f;
            const auto bodyImpulse = observed.demo.getWindDragBodyImpulse();
            const auto fluidImpulse = observed.demo.getWindDragFluidImpulse();
            sawDragBody |= bodyImpulse.x != 0.0f || bodyImpulse.y != 0.0f;
            sawDragFluid |= fluidImpulse.x != 0.0f || fluidImpulse.y != 0.0f;
            if( stabilityAudit )
            {
                const double rx = static_cast<double>( bodyImpulse.x ) + fluidImpulse.x;
                const double ry = static_cast<double>( bodyImpulse.y ) + fluidImpulse.y;
                const std::string prefix = "fluidStabilityAudit." + key + ".dragResidual";
                probe::appendScalar( stabilityTrace, prefix + ".x", rx );
                probe::appendScalar( stabilityTrace, prefix + ".y", ry );
                probe::appendScalar( stabilityTrace, prefix + ".normSquared", rx * rx + ry * ry );
            }
            sawFlow |= field.getFlowAudit( timeStep / worldFluidScene::fluidSubsteps ).maxFluidFaceSpeed > 0.01f;
            if( sustained )
            {
                for( const auto& force : observed.demo.getWindPressureForces() )
                    if( force.force.x == 0.0f && force.force.y == 0.0f && force.torque == 0.0f )
                        throw std::runtime_error( key + " missing per-body pressure exchange" );
                if( ( bodyImpulse.x == 0.0f && bodyImpulse.y == 0.0f ) ||
                    ( fluidImpulse.x == 0.0f && fluidImpulse.y == 0.0f ) )
                    throw std::runtime_error( key + " missing aggregate two-way drag exchange" );
                ++exchangeFrames;
            }
        }
        if( sustained ) checkDomain( key );
        std::string a, b;
        observed.captureFrame( a, key, tick );
        control.captureFrame( b, key, tick );
        if( a != b )
            throw std::runtime_error( key + " observed and independent control World/Fluid states differ" );
        trace.append( a );
    }
    check( sawCoupled && sawDisjoint && sawPressure && sawFlow,
        "R3m did not exercise actual analytic coupled pressure feedback" );
    check( sawDragBody == drag && sawDragFluid == drag,
        "R3m drag path coverage differs from requested setting" );
    if( jointKind != fluidJointKind::none )
        check( sawJointReaction, "R3n expected a nonzero Joint reaction" );
    if( jointKind == fluidJointKind::anchoredAssembly )
        check( sawAnchorReaction, "R3t expected nonzero pivot reaction" );
    if( contactScenario )
    {
        check( jointTrace.maxContacts > 0 && jointTrace.sawNormalImpulse && sawCombinedSolve,
            "R3u expected a positive Contact impulse with both Joints in one island after coupled Fluid projection" );
        std::printf( "%s combined solve: %d intervals, first tick %d interval %d, "
            "max Contacts %zu, friction impulse observed %d\n",
            name, combinedIntervals, firstCombinedTick, firstCombinedInterval,
            jointTrace.maxContacts, static_cast<int>( jointTrace.sawTangentImpulse ) );
    }
    if( sustained )
    {
        check( activeIntervals == frameCount * worldFluidScene::fluidSubsteps &&
            exchangeFrames == frameCount, "R3t incomplete sustained coupling coverage" );
        std::printf( "%s sustained coupling: %d intervals, %d exchange frames, "
            "min geometric margin %.9g m, max face speed %.9g m/s, max continuity RMS %.9g 1/s\n",
            name, activeIntervals, exchangeFrames, minimumMargin, maximumFaceSpeed, maximumContinuity );
    }
    if( stabilityAudit )
    {
        check( auditedIntervals == frameCount * worldFluidScene::fluidSubsteps,
            "R3v incomplete phase audit" );
        std::printf( "%s stability audit: %d intervals, %d valid analytic budgets, "
            "max projected face %.9g m/s, max pressure velocity delta %.9g m/s\n",
            name, auditedIntervals, analyticBudgets, auditMaxSpeed, auditMaxPressureDelta );
        trace.append( stabilityTrace );
    }
    return trace;
}

// R3w: reproduce the actual historical fine-grid frozen-pressure fixture.
// Do not force a particular peak: deterministic math changes may move it.
// The fixed historical index is compared against the live projection, while
// all 64 intervals are inspected for the true maximum in the current build.
std::string runNearClosedFaceScene()
{
    using probe = zonai::fluidStateTraceProbe2;
    using namespace zonai;
    using namespace zonai::sandbox;
    constexpr int frames = 16;
    constexpr int fluidSubsteps = 4;
    constexpr int targetIndex = 43; // warmup 4 + measured frame 6, interval 3 (zero-based)
    constexpr float h = 0.25f;
    constexpr float subDt = timeStep / fluidSubsteps;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.configureWindField( 48, 28, h );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindDisjointAnalyticEnabled( true );
    scene.setWindFluidSubstepCount( fluidSubsteps );
    scene.setWindFaceStageAuditTarget( 17, 13, false );
    scene.setWindObstacleCouplingEnabled( true );
    scene.setWindSynchronizedCouplingEnabled( true );
    scene.setWindSourceEnabled( true );
    scene.setWindSourceStrength( 8.0f );
    scene.setWindDragCoefficient( 0.0f );
    scene.seedWindTracer();
    scene.getWorld().SetBodyLinearVelocity( scene.getShapes()[0].bodyHandle, { 0.7f, -0.10f } );
    scene.getWorld().SetBodyAngularVelocity( scene.getShapes()[1].bodyHandle, 0.5f );
    auto& field = const_cast<fluidField2&>( *scene.getWindField() );
    std::optional<fluidField2> frozen;
    std::vector<float> beforeU, beforeV;
    std::string trace, liveProjection;
    int calls = 0, frozenCalls = 0, validBudgets = 0;
    float maxSpeed = 0.0f, maxDelta = 0.0f;
    int peakIndex = -1, peakX = -1, peakY = -1;
    bool peakIsU = true;
    float peakOpen = 0.0f, targetOpen = 0.0f, targetBefore = 0.0f, targetAfter = 0.0f;
    scene.setWindPrePressureSnapshotObserver( [&]( const fluidField2& snapshot )
    {
        if( frozenCalls == targetIndex ) frozen.emplace( snapshot );
        ++frozenCalls;
    } );
    probe::setStageObserver( field, [&]( const fluidField2& snapshot, int phase )
    {
        check( phase == calls % 10, "R3w Fluid phase order" );
        const int index = calls / 10;
        ++calls;
        const std::string key = "nearClosedFace.tick." + std::to_string( index / fluidSubsteps + 1 ) +
            ".interval." + std::to_string( index % fluidSubsteps ) + ".stage." + std::to_string( phase );
        probe::captureTargetV( snapshot, trace, key + ".targetV", 17, 13 );
        if( phase >= 4 && phase <= 8 )
        {
            if( phase == 7 ) probe::capturePrePressure( snapshot, beforeU, beforeV );
            const auto sample = probe::captureStabilityPhase( snapshot, trace, key,
                phase == 8 ? &beforeU : nullptr, phase == 8 ? &beforeV : nullptr );
            if( phase == 8 )
            {
                const auto& p = snapshot.getCutCellPressure();
                if( !p.usedCoupledProjection || p.coupledSolveFailed ||
                    p.coupledStatus != fluidCoupledPressureStatus2::converged )
                    throw std::runtime_error( key + " coupled projection failed" );
                validBudgets += static_cast<int>( p.hasAnalyticImpulseBudget );
                maxDelta = std::max( maxDelta, sample.deltaPeak );
                if( sample.peak > maxSpeed )
                {
                    maxSpeed = sample.peak; peakIndex = index; peakX = sample.peakX;
                    peakY = sample.peakY; peakIsU = sample.peakIsU; peakOpen = sample.peakOpen;
                }
                if( index == targetIndex ) probe::captureProjection( snapshot, liveProjection, "projection" );
            }
        }
        if( phase == 9 )
        {
            const auto& a = snapshot.getFaceStageAudit();
            check( a.valid && !a.isU && a.x == 17 && a.y == 13,
                "R3w fixed face audit target missing" );
            const auto emit = [&]( const std::string& name, auto value )
            {
                if( !std::isfinite( static_cast<double>( value ) ) )
                    throw std::runtime_error( key + name + " nonfinite face audit" );
                probe::appendScalar( trace, key + name, value );
            };
            emit( ".face.open", a.openFraction );
            emit( ".face.beforeGeometry", a.beforeGeometry );
            emit( ".face.afterRemap", a.afterRemap );
            emit( ".face.afterBoundary", a.afterBoundary );
            emit( ".face.afterAdvection", a.afterAdvection );
            emit( ".face.afterPressure", a.afterPressure );
            emit( ".face.predictedDelta", a.predictedPressureDelta );
            emit( ".face.lowerFraction", a.lowerCellFraction );
            emit( ".face.upperFraction", a.upperCellFraction );
            emit( ".face.lowerVolumeRate", a.lowerVolumeRate );
            emit( ".face.upperVolumeRate", a.upperVolumeRate );
            emit( ".face.lowerPressure", a.lowerPressure );
            emit( ".face.upperPressure", a.upperPressure );
            const auto row = [&]( const std::string& prefix, const fluidFaceCellPressureAudit2& r )
            {
                emit( prefix + ".west", r.westOpen ); emit( prefix + ".east", r.eastOpen );
                emit( prefix + ".south", r.southOpen ); emit( prefix + ".north", r.northOpen );
                emit( prefix + ".rhs", r.rhs ); emit( prefix + ".laplacian", r.laplacian );
                emit( prefix + ".schur", r.schur ); emit( prefix + ".residual", r.rowResidual );
                emit( prefix + ".flux", r.finalOpenFlux ); emit( prefix + ".continuity", r.rawContinuity );
            };
            row( ".lowerRow", a.lowerRow ); row( ".upperRow", a.upperRow );
            if( index == targetIndex )
            {
                targetOpen = a.openFraction; targetBefore = a.afterAdvection; targetAfter = a.afterPressure;
            }
        }
    } );
    for( int tick = 0; tick < frames; ++tick ) scene.step( timeStep, 8 );
    probe::setStageObserver( field, {} );
    scene.setWindPrePressureSnapshotObserver( {} );
    check( calls == frames * fluidSubsteps * 10 && frozenCalls == frames * fluidSubsteps &&
        frozen.has_value() && !liveProjection.empty(), "R3w incomplete historical replay" );

    // Replay on independent deep copies of the same pre-pressure state.
    // Cached/default must reproduce live pressure exactly. The uncached
    // variant is a differential diagnostic, not a claim of same bits A/B.
    fluidField2 baseline = *frozen, reference = *frozen;
    for( auto* copy : { &baseline, &reference } )
    {
        copy->setPrePressureSnapshotObserver( {} );
        probe::setStageObserver( *copy, {} );
    }
    reference.setAnalyticCutCellCachedSchurEdgesEnabled( false );
    const auto a = baseline.replayFrozenCutCellPressure();
    const auto b = reference.replayFrozenCutCellPressure();
    check( a && b && a->size() == 3 && b->size() == 3, "R3w frozen replay feedback missing" );
    std::string baselineProjection;
    probe::captureProjection( baseline, baselineProjection, "projection" );
    check( baselineProjection == liveProjection, "R3w default frozen pressure differs from live stage 8" );
    const auto compareReplay = [&]( const char* label, const fluidField2& copy,
        const std::vector<fluidObstacleForce2>& feedback )
    {
        const std::string key = std::string( "nearClosedFace.frozen." ) + label;
        const auto& p = copy.getCutCellPressure();
        check( p.usedCoupledProjection && !p.coupledSolveFailed &&
            p.coupledStatus == fluidCoupledPressureStatus2::converged, "R3w frozen PCG failed" );
        probe::captureProjection( copy, trace, key );
        probe::captureTargetV( copy, trace, key + ".targetV", 17, 13 );
        probe::captureStabilityPhase( copy, trace, key + ".ledger", &beforeU, &beforeV );
        // Feedback is pressure-only (no tracer transport or World integration).
        for( std::size_t i = 0; i < feedback.size(); ++i )
        {
            const auto prefix = key + ".body[" + std::to_string( i ) + "]";
            probe::appendScalar( trace, prefix + ".force.x", feedback[i].force.x );
            probe::appendScalar( trace, prefix + ".force.y", feedback[i].force.y );
            probe::appendScalar( trace, prefix + ".torque", feedback[i].torque );
        }
        probe::appendScalar( trace, key + ".tracerMass", copy.getTracerMass() );
    };
    // Use the actual frozen pre-pressure velocities, not the last live interval.
    probe::capturePrePressure( *frozen, beforeU, beforeV );
    compareReplay( "cached", baseline, *a );
    compareReplay( "uncached", reference, *b );
    float forceDifference = 0.0f, torqueDifference = 0.0f;
    for( std::size_t i = 0; i < a->size(); ++i )
    {
        const auto d = ( *a )[i].force - ( *b )[i].force;
        forceDifference = std::max( forceDifference, std::max( std::abs( d.x ), std::abs( d.y ) ) );
        torqueDifference = std::max( torqueDifference, std::abs( ( *a )[i].torque - ( *b )[i].torque ) );
    }
    std::printf( "R3w fine grid: %d intervals, %d analytic budgets, peak %.9g m/s at index %d %c(%d,%d), "
        "opening %.9g, max projection delta %.9g m/s; historical V(17,13) index 43 open %.9g, "
        "before %.9g, after %.9g; cached/uncached max force component difference %.9g, torque %.9g\n",
        frozenCalls, validBudgets, maxSpeed, peakIndex, peakIsU ? 'U' : 'V', peakX, peakY,
        peakOpen, maxDelta, targetOpen, targetBefore, targetAfter, forceDifference, torqueDifference );
    return trace;
}

} // namespace

int main( int argc, char** argv )
{
    try
    {
        if( argc != 1 && ( argc != 3 || std::string( argv[1] ) != "--emit" ) )
        {
            std::fprintf( stderr, "Usage: fluidStateDeterminismTests [--emit TRACE.tsv]\n" );
            return EXIT_FAILURE;
        }

        std::string trace =
            runScene( "sampledCutCell", false ) + runScene( "analyticCircle", true );
        trace += runGeometryScene( "sampledMixed", geometrySceneKind::sampledMixed );
        trace += runGeometryScene( "analyticCapsule", geometrySceneKind::analyticCapsule );
        trace += runGeometryScene( "analyticPolygon", geometrySceneKind::analyticPolygon );
        trace += runGeometryScene( "coupledDisjoint", geometrySceneKind::coupledDisjoint );
        trace += runWorldFluidScene( "worldFluidPressure", false );
        trace += runWorldFluidScene( "worldFluidDrag", true );
        trace += runWorldFluidScene( "worldFluidRevolute", true, fluidJointKind::revolute );
        trace += runWorldFluidScene( "worldFluidWeld", true, fluidJointKind::weld );
        trace += runWorldFluidScene( "worldFluidDrag180", true, fluidJointKind::none, 360,
            "worldFluidDrag360Continuation" );
        trace += runWorldFluidScene( "worldFluidAnchoredAssembly", true,
            fluidJointKind::anchoredAssembly, 120 );
        trace += runWorldFluidScene( "worldFluidContactAssembly", true,
            fluidJointKind::anchoredAssembly, 60, nullptr, true );
        trace += runNearClosedFaceScene();

        if( argc == 3 )
        {
            std::FILE* output = std::fopen( argv[2], "wb" );
            if( !output ) throw std::runtime_error( "cannot create Fluid state trace" );
            const std::size_t written = std::fwrite( trace.data(), 1, trace.size(), output );
            const int closed = std::fclose( output );
            check( written == trace.size() && closed == 0, "cannot write Fluid state trace" );
        }

        std::puts( "Fluid standalone and World/Fluid interleave replay passed" );
        return EXIT_SUCCESS;
    }
    catch( const std::exception& error )
    {
        std::fprintf( stderr, "Fluid determinism test error: %s\n", error.what() );
        return EXIT_FAILURE;
    }
}
