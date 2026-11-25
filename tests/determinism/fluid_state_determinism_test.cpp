#include "fluid/fluidField2.h"
#include "fluid/fluidTransport2.h"
#include "fluid/fluidIntervalPressure2.h"
#include "math/deterministicMath.h"
#include "collision/shapeGeometry.h"
#include "rigidBodyDemo.h"
#include "world_state_snapshot.h"
#include "joint_substep_trace.h"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
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
    static inline bool benchmarkForward = false;
    static inline bool benchmarkTracer = false;
    using benchmarkClock = std::chrono::steady_clock;
    struct forwardTiming
    {
        double quadratureMs = 0.0, linearMs = 0.0, totalMs = 0.0;
        double setupMs = 0.0, pressureMs = 0.0, endpointMs = 0.0;
    };
    static double elapsedMs( benchmarkClock::time_point start, benchmarkClock::time_point finish )
    {
        return std::chrono::duration<double, std::milli>( finish - start ).count();
    }

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

    // R4ak: a real triangular fluid sliver below the R4aj candidate
    // threshold must not disappear just because its fraction is tiny.
    static void checkCertifiedCellCoverage()
    {
        constexpr float delta = 0x1p-23f;
        for( bool bounded : { false, true } )
        {
            for( float cut : { -delta, delta } )
            {
                // Target [0,1]^2. The diagonal edge x+y=2-cut
                // either covers the rectangle or leaves area cut^2/2.
                const std::array<vec2, 5> vertices{{
                    { -1.0f, -1.0f }, { 2.0f, -1.0f }, { 2.0f, -cut },
                    { -cut, 2.0f }, { -1.0f, 2.0f }
                }};
                const std::array<fluidShapeObstacle2, 2> obstacles{{
                    { MakePolygon( vertices ), { {}, rot2::FromRadians( 0.0f ) },
                        {}, {}, 0.0f, 0.0f, 0.0f },
                    { circle2{ {}, 0.25f }, { { 4.0f, 4.0f }, rot2::FromRadians( 0.0f ) },
                        { 4.0f, 4.0f }, {}, 0.0f, 0.0f, 0.0f }
                }};
                const auto measure = [&]( bool certified )
                {
                    fluidField2 field{ 8, 8, 1.0f, { -2.0f, -2.0f } };
                    field.setCutCellProjectionEnabled( true );
                    field.setDisjointAnalyticCutCellEnabled( true );
                    field.setAnalyticCutCellBoundedCellAreasEnabled( bounded );
                    field.certifiedFullCellCoverageEnabled_ = certified;
                    field.updateObstacles( obstacles );
                    field.updateCutCellGeometry( obstacles );
                    if( !field.cutCellTransition_.usedDisjointAnalyticGeometry )
                        throw std::runtime_error( "R4ak coverage fixture not analytic" );
                    return field.cellFluidFraction_[field.cellIndex( 2, 2 )];
                };
                const float raw = measure( false ), certified = measure( true );
                const double expected = cut > 0.0f ? 0.5 * static_cast<double>( cut ) * cut : 0.0;
                std::printf( "R4ak coverage bounded %d cut %.9g raw %.17g certified %.17g expected %.17g\n",
                    static_cast<int>( bounded ), cut, raw, certified, expected );
                if( cut > 0.0f )
                {
                    if( !( raw > 0.0f && raw <= 64.0 * std::numeric_limits<double>::epsilon() &&
                        raw == certified && std::abs( certified - expected ) <=
                            4.0 * std::numeric_limits<double>::epsilon() ) )
                        throw std::runtime_error( "R4ak genuine tiny sliver not preserved" );
                }
                else if( certified != 0.0f )
                    throw std::runtime_error( "R4ak fully covered cell not zero" );
            }
        }
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

    // R3x: replay existing volume-source choices on a private frozen copy.
    // Keep original geometry, U/V, obstacle state, and pressure operator.
    // Snapshot fallback is already implemented in prepareCutCellVolumeChange.
    static void useSnapshotVolumeSource( fluidField2& field )
    {
        if( !field.frozenPressureReplayReady_ || !field.volumeSourceValid_ )
            throw std::runtime_error( "R3x snapshot source requires valid frozen history" );
        field.sweptFaceFlux_.usedSweptFlux = false;
    }

    static void useCorrectedSweptVolumeSource( fluidField2& field )
    {
        if( !field.frozenPressureReplayReady_ || !field.sweptFaceFlux_.usedSweptFlux ||
            field.sweptGeometryCorrectionEnabled_ )
            throw std::runtime_error( "R3x correction requires uncorrected frozen swept flux" );
        field.setSweptGeometryCorrectionEnabled( true );
        field.reconcileSweptSolidFlux();
    }

    static void captureVolumeSourceAudit( const fluidField2& before, const fluidField2& after,
        std::string& trace, const std::string& key, int x, int y )
    {
        const int target = after.cellIndex( x, y );
        const float area = after.cellSize_ * after.cellSize_;
        const float dt = before.frozenPressureTimeStep_;
        const auto sweptChange = [&]( int i )
        {
            const int cx = i % after.width_, cy = i / after.width_;
            return ( before.sweptUVolume_[before.uIndex( cx + 1, cy )] -
                before.sweptUVolume_[before.uIndex( cx, cy )] +
                before.sweptVVolume_[before.vIndex( cx, cy + 1 )] -
                before.sweptVVolume_[before.vIndex( cx, cy )] ) / area;
        };
        const auto snapshotChange = [&]( int i )
        {
            return before.cellFluidFraction_[i] - before.previousCellFluidFraction_[i];
        };
        const auto usedChange = [&]( int i )
        {
            return before.sweptFaceFlux_.usedSweptFlux ? sweptChange( i ) : snapshotChange( i );
        };
        float reconstructed = 0.0f, relocated = 0.0f;
        int donors = 0;
        double globalSnapshot = 0.0, globalSwept = 0.0, globalAssigned = 0.0;
        for( int i = 0; i < static_cast<int>( before.cellFluidFraction_.size() ); ++i )
        {
            const float change = usedChange( i );
            globalSnapshot += snapshotChange( i );
            globalSwept += sweptChange( i );
            globalAssigned += after.cutCellVolumeRate_[i];
            if( i == target ) reconstructed += change;
            else if( before.cellFluidFraction_[i] <= 0.0f && change != 0.0f &&
                after.cutCellNearestFluid_[i] == target )
            {
                reconstructed += change;
                relocated += change;
                ++donors;
                const auto prefix = key + ".donor[" + std::to_string( i ) + "]";
                appendScalar( trace, prefix + ".snapshot", snapshotChange( i ) );
                appendScalar( trace, prefix + ".swept", sweptChange( i ) );
                appendScalar( trace, prefix + ".assigned", change );
            }
        }
        if( before.cellFluidFraction_[target] <= 0.0f ||
            std::bit_cast<std::uint32_t>( reconstructed ) !=
            std::bit_cast<std::uint32_t>( after.cutCellVolumeRate_[target] ) )
            throw std::runtime_error( key + " source attribution differs from actual projected row" );

        const float west = before.cutCellUOpen( x, y ) * before.u_[before.uIndex( x, y )];
        const float east = before.cutCellUOpen( x + 1, y ) * before.u_[before.uIndex( x + 1, y )];
        const float south = before.cutCellVOpen( x, y ) * before.v_[before.vIndex( x, y )];
        const float north = before.cutCellVOpen( x, y + 1 ) * before.v_[before.vIndex( x, y + 1 )];
        const float velocityDivergence = ( east - west + north - south ) * ( 1.0f / after.cellSize_ );
        const float sourceDivergence = reconstructed / dt;
        const float rawDivergence = velocityDivergence + sourceDivergence;
        const auto emit = [&]( const char* name, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( key + name + " nonfinite source audit" );
            appendScalar( trace, key + name, value );
        };
        emit( ".previousFraction", before.previousCellFluidFraction_[target] );
        emit( ".currentFraction", before.cellFluidFraction_[target] );
        emit( ".snapshotChange", snapshotChange( target ) );
        emit( ".sweptChange", sweptChange( target ) );
        emit( ".usedSwept", static_cast<int>( before.sweptFaceFlux_.usedSweptFlux ) );
        emit( ".directChange", usedChange( target ) );
        emit( ".relocatedChange", relocated );
        emit( ".donorCount", donors );
        emit( ".assignedChange", reconstructed );
        emit( ".westFlux", west ); emit( ".eastFlux", east );
        emit( ".southFlux", south ); emit( ".northFlux", north );
        emit( ".velocityDivergence", velocityDivergence );
        emit( ".sourceDivergence", sourceDivergence );
        emit( ".rawDivergence", rawDivergence );
        emit( ".compatibilityAdjustedDivergence", after.divergence_[target] );
        emit( ".compatibilityRemoved", rawDivergence - after.divergence_[target] );
        emit( ".globalSnapshotChange", globalSnapshot );
        emit( ".globalSweptChange", globalSwept );
        emit( ".globalAssignedChange", globalAssigned );
        emit( ".globalUnresolvedVolumeRate", after.cutCellPressure_.unresolvedVolumeRate );
        emit( ".globalRelocatedVolumeRate", after.cutCellPressure_.redistributedVolumeRate );
        emit( ".geometryMismatch", after.sweptFaceFlux_.geometryMismatch );
        const int lower = after.cellIndex( x, y - 1 );
        emit( ".faceSpeed", after.v_[after.vIndex( x, y )] );
        emit( ".faceOpen", after.cutCellVOpen( x, y ) );
        emit( ".pressureJump", after.pressure_[target] - after.pressure_[lower] );
        std::printf( "%s previous fraction %.9g current %.9g snapshot %.9g swept %.9g "
            "assigned %.9g relocated %.9g donors %d velocity divergence %.9g source divergence %.9g "
            "adjusted divergence %.9g pressure jump %.9g target speed %.9g\n",
            key.c_str(), before.previousCellFluidFraction_[target], before.cellFluidFraction_[target],
            snapshotChange( target ), sweptChange( target ), reconstructed, relocated, donors,
            velocityDivergence, sourceDivergence, after.divergence_[target],
            after.pressure_[target] - after.pressure_[lower], after.v_[after.vIndex( x, y )] );
    }

    // R3y: local flux feasibility for the actual projected sliver. The
    // predicted solid response and compatibility correction are held fixed.
    // A lower bound on max |u| is not a proposed pressure/velocity clamp.
    static void captureSliverFluxBudget( const fluidField2& before, const fluidField2& after,
        std::string& trace, const std::string& key, int x, int y )
    {
        const int cell = after.cellIndex( x, y );
        const double h = after.cellSize_;
        const double dt = before.frozenPressureTimeStep_;
        double opening = 0.0, preFlux = 0.0, postFlux = 0.0, solidFlux = 0.0;
        double peak = 0.0, weightedSquaredSpeed = 0.0, laplacian = 0.0;
        const auto emit = [&]( const std::string& name, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( key + name + " nonfinite flux budget" );
            appendScalar( trace, key + name, value );
        };
        const auto face = [&]( const char* name, bool isU, int fx, int fy, int neighbor, double sign )
        {
            const int index = isU ? after.uIndex( fx, fy ) : after.vIndex( fx, fy );
            const double a = isU ? after.cutCellUOpen( fx, fy ) : after.cutCellVOpen( fx, fy );
            const double u0 = isU ? before.u_[index] : before.v_[index];
            const double u1 = isU ? after.u_[index] : after.v_[index];
            const double ds = isU ? after.pressureUSolidFlux_[index] : after.pressureVSolidFlux_[index];
            opening += a;
            preFlux += sign * a * u0;
            postFlux += sign * a * u1;
            solidFlux += sign * ds;
            if( a > 0.0 )
            {
                peak = std::max( peak, std::abs( u1 ) );
                weightedSquaredSpeed += a * u1 * u1;
                laplacian += a * ( static_cast<double>( after.pressure_[cell] ) - after.pressure_[neighbor] );
            }
            const std::string p = std::string( "." ) + name;
            emit( p + ".open", a );
            emit( p + ".before", u0 );
            emit( p + ".after", u1 );
            emit( p + ".outwardFlux", sign * a * u1 );
            emit( p + ".predictedSolidFlux", sign * ds );
        };
        face( "west", true, x, y, after.cellIndex( x - 1, y ), -1.0 );
        face( "east", true, x + 1, y, after.cellIndex( x + 1, y ), 1.0 );
        face( "south", false, x, y, after.cellIndex( x, y - 1 ), -1.0 );
        face( "north", false, x, y + 1, after.cellIndex( x, y + 1 ), 1.0 );
        if( opening <= 0.0 ) throw std::runtime_error( key + " target cell has no open face" );

        // The adjusted pre-projection divergence is the actual stored RHS
        // source, including the component mean correction. Reconstruct the
        // required net open flux with the solid prediction held at its
        // computed value (rounding differs from the float production row).
        const double requiredFlux = preFlux - h * after.divergence_[cell] - solidFlux;
        const double requiredPeak = std::abs( requiredFlux ) / opening;
        const double observedBound = std::abs( postFlux ) / opening;
        if( observedBound > peak + 1.0e-12 * std::max( 1.0, peak ) )
            throw std::runtime_error( key + " triangle inequality violated" );

        // Aggregate B coefficients by body before taking the diagonal of
        // B M^-1 B^T; summing individual face squares would miss cancellation.
        const auto& obstacles = before.frozenPressureObstacles_;
        std::vector<fluidField2::coupledBodyResponse> local( obstacles.size() ), global( obstacles.size() );
        for( const auto& f : after.coupledPressureScratch_.faces )
        {
            auto& g = global[static_cast<std::size_t>( f.owner )];
            const double p = after.pressure_[f.cell];
            g.fx += p * f.fx; g.fy += p * f.fy; g.torque += p * f.torque;
            if( f.cell != cell ) continue;
            auto& b = local[static_cast<std::size_t>( f.owner )];
            b.fx += f.fx; b.fy += f.fy; b.torque += f.torque;
        }
        double schurDiagonal = 0.0, schurRow = 0.0;
        for( std::size_t i = 0; i < obstacles.size(); ++i )
        {
            const auto& b = local[i];
            const auto& g = global[i];
            const double im = obstacles[i].inverseMass, ii = obstacles[i].inverseInertia;
            schurDiagonal += im * ( b.fx * b.fx + b.fy * b.fy ) + ii * b.torque * b.torque;
            schurRow += im * ( b.fx * g.fx + b.fy * g.fy ) + ii * b.torque * g.torque;
        }
        const double rhs = after.coupledPressureScratch_.rhs[cell];
        emit( ".openingSum", opening );
        emit( ".preFlux", preFlux );
        emit( ".postFlux", postFlux );
        emit( ".solidFlux", solidFlux );
        emit( ".requiredOpenFlux", requiredFlux );
        emit( ".fluxError", postFlux - requiredFlux );
        emit( ".requiredPeakLowerBound", requiredPeak );
        emit( ".observedPeakLowerBound", observedBound );
        emit( ".actualLocalPeak", peak );
        emit( ".weightedSquaredSpeed", weightedSquaredSpeed );
        emit( ".minimumWeightedSquaredSpeed", postFlux * postFlux / opening );
        emit( ".rhs", rhs );
        emit( ".laplacianDiagonal", opening );
        emit( ".schurDiagonal", schurDiagonal );
        emit( ".schurRow", schurRow );
        emit( ".laplacianRow", laplacian );
        emit( ".rowResidual", rhs - laplacian - schurRow );
        emit( ".sourceFractionChange", after.cutCellVolumeRate_[cell] );
        emit( ".sourceRate", after.cutCellVolumeRate_[cell] / dt );
        std::printf( "%s opening sum %.9g required flux %.9g observed flux %.9g "
            "solid flux %.9g minimum required peak %.9g m/s actual local peak %.9g m/s "
            "Schur diagonal %.9g L diagonal %.9g row residual %.9g\n",
            key.c_str(), opening, requiredFlux, postFlux, solidFlux, requiredPeak, peak,
            schurDiagonal, opening, rhs - laplacian - schurRow );
    }

    // Read-only R3y candidate corresponding to the aperture-normalized
    // interpolation in experimental #322, using current effective openings.
    // This is not wired into the production sampler, drag, or advection.
    static float apertureSample( const fluidField2& field, bool isU, float x, float y )
    {
        const int width = field.width_ + ( isU ? 1 : 0 );
        const int height = field.height_ + ( isU ? 0 : 1 );
        const auto& values = isU ? field.u_ : field.v_;
        x = std::clamp( x, 0.0f, static_cast<float>( width - 1 ) );
        y = std::clamp( y, 0.0f, static_cast<float>( height - 1 ) );
        const int i = static_cast<int>( x ), j = static_cast<int>( y );
        const int i1 = std::min( i + 1, width - 1 ), j1 = std::min( j + 1, height - 1 );
        const float tx = x - i, ty = y - j;
        const auto open = [&]( int a, int b ) { return isU ? field.cutCellUOpen( a, b ) : field.cutCellVOpen( a, b ); };
        const float o00 = open( i, j ), o10 = open( i1, j ), o01 = open( i, j1 ), o11 = open( i1, j1 );
        if( o00 == 1.0f && o10 == 1.0f && o01 == 1.0f && o11 == 1.0f )
            return field.sampleComponent( values, width, height, x, y );
        const std::array<float, 4> weights{
            ( 1.0f - tx ) * ( 1.0f - ty ) * o00, tx * ( 1.0f - ty ) * o10,
            ( 1.0f - tx ) * ty * o01, tx * ty * o11 };
        const std::array<int, 4> indices{ j * width + i, j * width + i1, j1 * width + i, j1 * width + i1 };
        const float total = weights[0] + weights[1] + weights[2] + weights[3];
        if( total <= 0.0f ) return 0.0f;
        float sum = 0.0f;
        for( std::size_t k = 0; k < weights.size(); ++k )
            if( weights[k] > 0.0f ) sum += weights[k] * values[indices[k]];
        return sum / total;
    }

    static void captureSliverSampling( const fluidField2& field, std::string& trace,
        const std::string& key )
    {
        // Same frozen post-projection data, sample on the face, at both
        // adjacent cell centers and along the connecting segment.
        for( int i = 0; i <= 8; ++i )
        {
            const float y = 12.5f + 0.125f * i;
            const vec2 point = field.origin_ + field.cellSize_ * vec2{ 17.5f, y };
            const vec2 standard = field.sampleVelocity( point );
            const vec2 grid = ( point - field.origin_ ) / field.cellSize_;
            const vec2 candidate{ apertureSample( field, true, grid.x, grid.y - 0.5f ),
                apertureSample( field, false, grid.x - 0.5f, grid.y ) };
            if( !IsFinite( standard ) || !IsFinite( candidate ) )
                throw std::runtime_error( key + " nonfinite sampling witness" );
            const auto p = key + ".point[" + std::to_string( i ) + "]";
            appendScalar( trace, p + ".gridY", y );
            appendScalar( trace, p + ".standard.x", standard.x );
            appendScalar( trace, p + ".standard.y", standard.y );
            appendScalar( trace, p + ".candidate.x", candidate.x );
            appendScalar( trace, p + ".candidate.y", candidate.y );
            if( i == 0 || i == 4 || i == 8 )
                std::printf( "%s grid y %.9g standard v %.9g weighted v %.9g\n",
                    key.c_str(), y, standard.y, candidate.y );
        }
    }

    // R3z: integrate the effective open-face area along the same swept
    // transform path, on a geometry-only private copy. Neither its masks
    // nor its history are used by the live pressure or rigid body solver.
    static void captureSpaceTimeAperture( const fluidField2& before, const fluidField2& after,
        std::string& trace, const std::string& key, int x, int y )
    {
        const auto& start = before.previousObstacles_;
        const auto& finish = before.frozenPressureObstacles_;
        if( start.size() != finish.size() || start.empty() || !before.volumeSourceValid_ ||
            !before.sweptFaceFlux_.usedSweptFlux )
            throw std::runtime_error( key + " missing swept geometry history" );
        fluidField2 geometry = before;
        geometry.setPrePressureSnapshotObserver( {} );
        setStageObserver( geometry, {} );
        const int cell = before.cellIndex( x, y );
        const auto measure = [&]( const fluidField2& field )
        {
            return std::array<double, 5>{ field.cutCellUOpen( x, y ), field.cutCellUOpen( x + 1, y ),
                field.cutCellVOpen( x, y ), field.cutCellVOpen( x, y + 1 ), field.cellFluidFraction_[cell] };
        };
        const auto total = []( const std::array<double, 5>& a ) { return a[0] + a[1] + a[2] + a[3]; };
        const auto rebuild = [&]( std::span<const fluidShapeObstacle2> poses )
        {
            geometry.updateObstacles( poses );
            geometry.updateCutCellGeometry( poses );
            if( !geometry.cutCellTransition_.usedDisjointAnalyticGeometry )
                throw std::runtime_error( key + " intermediate geometry fell back from analytic mode" );
            const auto a = measure( geometry );
            for( double value : a )
                if( !std::isfinite( value ) || value < 0.0 || value > 1.0 )
                    throw std::runtime_error( key + " invalid intermediate geometry" );
            return a;
        };
        const auto first = rebuild( start );
        const auto last = rebuild( finish );
        if( first[4] != before.previousCellFluidFraction_[cell] || last != measure( before ) )
            throw std::runtime_error( key + " endpoint geometry reconstruction differs" );
        // Same signed flux definition as R3y. Freeze the original source,
        // compatibility correction and computed solid response for the bound.
        const double preFlux = last[1] * before.u_[before.uIndex( x + 1, y )] -
            last[0] * before.u_[before.uIndex( x, y )] +
            last[3] * before.v_[before.vIndex( x, y + 1 )] -
            last[2] * before.v_[before.vIndex( x, y )];
        const double solidFlux = static_cast<double>( after.pressureUSolidFlux_[after.uIndex( x + 1, y )] ) -
            after.pressureUSolidFlux_[after.uIndex( x, y )] +
            after.pressureVSolidFlux_[after.vIndex( x, y + 1 )] -
            after.pressureVSolidFlux_[after.vIndex( x, y )];
        const double requiredFlux = preFlux - static_cast<double>( after.cellSize_ ) *
            after.divergence_[cell] - solidFlux;
        if( total( last ) <= 0.0 ) throw std::runtime_error( key + " missing endpoint opening" );
        appendScalar( trace, key + ".requiredFlux", requiredFlux );
        appendScalar( trace, key + ".endBound", std::abs( requiredFlux ) / total( last ) );
        const auto emit = [&]( const std::string& prefix, const std::array<double, 5>& a )
        {
            constexpr std::array<const char*, 5> names{ ".west", ".east", ".south", ".north", ".fraction" };
            for( std::size_t i = 0; i < names.size(); ++i )
                appendScalar( trace, prefix + names[i], a[i] );
            appendScalar( trace, prefix + ".openingSum", total( a ) );
        };
        emit( key + ".start", first );
        emit( key + ".end", last );

        std::vector<fluidShapeObstacle2> poses( finish.begin(), finish.end() );
        std::vector<float> startAngles( start.size() ), angularSpeeds( start.size() );
        const float dt = before.frozenPressureTimeStep_;
        for( std::size_t i = 0; i < start.size(); ++i )
        {
            startAngles[i] = deterministicAtan2( start[i].transform.rotation.s, start[i].transform.rotation.c );
            const float endAngle = deterministicAtan2( finish[i].transform.rotation.s, finish[i].transform.rotation.c );
            const float delta = std::remainder( endAngle - startAngles[i], 2.0f * std::numbers::pi_v<float> );
            angularSpeeds[i] = delta / dt;
        }
        // Match production midpoint transform arithmetic; finer quadrature
        // measures the time aperture, not an alternate pressure solve.
        for( int samples : { 1, 2, 4, 8, 16, 32, 64 } )
        {
            std::array<double, 5> mean{};
            for( int step = 0; step < samples; ++step )
            {
                const float alpha = ( step + 0.5f ) / samples;
                for( std::size_t i = 0; i < poses.size(); ++i )
                {
                    poses[i].transform.position = start[i].transform.position +
                        alpha * ( finish[i].transform.position - start[i].transform.position );
                    poses[i].transform.rotation = rot2::FromRadians( startAngles[i] + angularSpeeds[i] * dt * alpha );
                }
                const auto a = rebuild( poses );
                for( std::size_t i = 0; i < a.size(); ++i ) mean[i] += a[i];
            }
            for( double& a : mean ) a /= samples;
            const double opening = total( mean );
            if( opening <= 0.0 ) throw std::runtime_error( key + " missing average opening" );
            const std::string prefix = key + ".midpoint" + std::to_string( samples );
            emit( prefix, mean );
            const double bound = std::abs( requiredFlux ) / opening;
            appendScalar( trace, prefix + ".conditionalBound", bound );
            appendScalar( trace, prefix + ".openingRatioToEnd", opening / total( last ) );
            std::printf( "%s average opening %.9g ratio to end %.9g conditional speed bound %.9g m/s\n",
                prefix.c_str(), opening, opening / total( last ), bound );
        }
    }

    // R4d: independent representations. Closed endpoint faces have no
    // endpoint *fluid* velocity, but can carry a historical interval transfer.
    // This is not a complete MAC state (solid velocities are separate).
    struct intervalTransport
    {
        std::vector<double> uTransfer, vTransfer;
        std::vector<double> uEndpointFluid, vEndpointFluid;
    };

    static void captureIntervalTransport( const fluidField2& before, const intervalTransport& state,
        const std::vector<double>& source, const std::vector<double>& expectedResidual,
        std::string& trace, const std::string& key,
        std::array<std::vector<double>, 2>* transportedMass = nullptr )
    {
        const std::size_t n = before.cellFluidFraction_.size();
        const double area = static_cast<double>( before.cellSize_ ) * before.cellSize_;
        struct transferEdge { int donor, recipient; double volume; bool closedEnd; };
        std::vector<transferEdge> edges;
        std::vector<double> residual = source, outgoing( n ), capacity( n );
        double closedTransfer = 0.0, peakEndpoint = 0.0, transferL1 = 0.0;
        int closedFaces = 0;
        const auto face = [&]( bool isU, int x, int y )
        {
            const int index = isU ? before.uIndex( x, y ) : before.vIndex( x, y );
            const double transfer = isU ? state.uTransfer[index] : state.vTransfer[index];
            const double endpoint = isU ? state.uEndpointFluid[index] : state.vEndpointFluid[index];
            const bool closed = ( isU ? before.cutCellUOpen( x, y ) : before.cutCellVOpen( x, y ) ) == 0.0f;
            if( !std::isfinite( transfer ) || !std::isfinite( endpoint ) || ( closed && endpoint != 0.0 ) )
                throw std::runtime_error( key + " invalid separated face state" );
            peakEndpoint = std::max( peakEndpoint, std::abs( endpoint ) );
            if( transfer == 0.0 ) return;
            const int a = before.cellIndex( x - ( isU ? 1 : 0 ), y - ( isU ? 0 : 1 ) );
            const int b = before.cellIndex( x, y );
            residual[a] += transfer; residual[b] -= transfer;
            const int donor = transfer > 0.0 ? a : b;
            const int recipient = transfer > 0.0 ? b : a;
            const double amount = std::abs( transfer );
            outgoing[donor] += amount;
            transferL1 += amount;
            edges.push_back( { donor, recipient, amount, closed } );
            if( closed ) { ++closedFaces; closedTransfer += amount; }
        };
        for( int y = 0; y < before.height_; ++y )
            for( int x = 1; x < before.width_; ++x ) face( true, x, y );
        for( int y = 1; y < before.height_; ++y )
            for( int x = 0; x < before.width_; ++x ) face( false, x, y );
        double identityError = 0.0, residualL1 = 0.0, unservedVolume = 0.0;
        int limitedDonors = 0;
        for( std::size_t i = 0; i < n; ++i )
        {
            capacity[i] = area * before.previousCellFluidFraction_[i];
            identityError = std::max( identityError, std::abs( residual[i] - expectedResidual[i] ) );
            residualL1 += std::abs( residual[i] );
            if( outgoing[i] > capacity[i] )
            {
                ++limitedDonors;
                unservedVolume += outgoing[i] - capacity[i];
            }
        }
        if( identityError != 0.0 )
            throw std::runtime_error( key + " stored transfer did not reproduce integrated volume ledger" );
        const auto out = [&]( const std::string& name, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( key + name + " nonfinite transport output" );
            appendScalar( trace, key + name, value );
        };
        out( ".closedFaces", closedFaces ); out( ".closedTransferL1", closedTransfer );
        out( ".peakEndpointFluid", peakEndpoint ); out( ".transferL1", transferL1 );
        out( ".identityError", identityError ); out( ".residualL1", residualL1 );
        out( ".limitedDonors", limitedDonors ); out( ".unservedDonorVolume", unservedVolume );
        appendVector( trace, key + ".uTransfer", state.uTransfer );
        appendVector( trace, key + ".vTransfer", state.vTransfer );
        appendVector( trace, key + ".uEndpointFluid", state.uEndpointFluid );
        appendVector( trace, key + ".vEndpointFluid", state.vEndpointFluid );

        // One simultaneous donor-limited scalar transport experiment. Each
        // signed interval transfer is consumed once, even if its face closed.
        // No swept tracer remap (would count displacement twice). No source
        // deletion or forced evacuation. Limit tracer, never the fluid ledger.
        for( bool uniform : { true, false } )
        {
            const auto& initial = uniform ? capacity : before.tracerMass_;
            if( initial.size() != n ) throw std::runtime_error( key + " tracer storage mismatch" );
            std::vector<double> remaining = initial, received( n );
            double beforeMass = 0.0, afterMass = 0.0, trappedBefore = 0.0, trappedAfter = 0.0;
            double closedMass = 0.0, movedMass = 0.0, endpointError = 0.0;
            double closingMassBefore = 0.0, closingMassAfter = 0.0;
            for( std::size_t i = 0; i < n; ++i )
            {
                if( !std::isfinite( initial[i] ) || initial[i] < 0.0 )
                    throw std::runtime_error( key + " invalid initial tracer mass" );
                beforeMass += initial[i];
                if( capacity[i] == 0.0 ) trappedBefore += initial[i];
                if( capacity[i] > 0.0 && before.cellFluidFraction_[i] == 0.0f )
                    closingMassBefore += initial[i];
            }
            for( const auto& e : edges )
            {
                if( capacity[e.donor] <= 0.0 ) continue;
                // Scale all simultaneous outgoing requests by the same
                // donor capacity; remaining cap is only a roundoff guard.
                const double requested = initial[e.donor] *
                    ( e.volume / std::max( capacity[e.donor], outgoing[e.donor] ) );
                const double mass = std::min( remaining[e.donor], requested );
                remaining[e.donor] -= mass;
                received[e.recipient] += mass;
                movedMass += mass;
                if( e.closedEnd ) closedMass += mass;
            }
            std::vector<double> result( n );
            for( std::size_t i = 0; i < n; ++i )
            {
                result[i] = remaining[i] + received[i];
                if( !std::isfinite( result[i] ) || result[i] < 0.0 )
                    throw std::runtime_error( key + " invalid transported mass" );
                afterMass += result[i];
                if( before.cellFluidFraction_[i] == 0.0f )
                {
                    trappedAfter += result[i];
                    if( capacity[i] > 0.0 ) closingMassAfter += result[i];
                }
                if( uniform )
                    endpointError += std::abs( result[i] - area * before.cellFluidFraction_[i] );
            }
            const double massError = afterMass - beforeMass;
            if( std::abs( massError ) > 1.0e-12 * std::max( 1.0, beforeMass ) )
                throw std::runtime_error( key + " interval tracer mass conservation failed" );
            const std::string prefix = uniform ? ".uniform" : ".seeded";
            out( prefix + ".beforeMass", beforeMass ); out( prefix + ".afterMass", afterMass );
            out( prefix + ".massError", massError ); out( prefix + ".movedMass", movedMass );
            out( prefix + ".closedFaceMass", closedMass );
            out( prefix + ".trappedBefore", trappedBefore ); out( prefix + ".trappedAfter", trappedAfter );
            out( prefix + ".closingMassBefore", closingMassBefore ); out( prefix + ".closingMassAfter", closingMassAfter );
            out( prefix + ".uniformEndpointError", endpointError );
            appendVector( trace, key + prefix + ".mass", result );
            if( transportedMass ) ( *transportedMass )[uniform ? 0 : 1] = result;
            std::printf( "%s%s mass before %.9g after %.9g error %.9g trapped %.9g closed face mass %.9g "
                "closing mass remaining %.9g uniform endpoint error %.9g\n",
                key.c_str(), prefix.c_str(), beforeMass, afterMass, massError, trappedAfter,
                closedMass, closingMassAfter, endpointError );
        }
        std::printf( "%s closed faces %d retained transfer %.9g m^2 endpoint fluid peak %.9g m/s "
            "limited donors %d unserved donor volume %.9g m^2 ledger identity %.9g\n",
            key.c_str(), closedFaces, closedTransfer, peakEndpoint, limitedDonors, unservedVolume, identityError );
    }

    // R4e: retain the quadrature's actual apertures and each panel's
    // start geometry. The solved interval velocity is constant in this
    // model; the tracer consumer alone advances chronologically.
    using transportPanel = fluidDetail::transportPanel;

    static void captureChronologicalTransport( const fluidField2& before,
        const intervalTransport& interval, const std::vector<double>& uVelocity,
        const std::vector<double>& vVelocity, const std::vector<transportPanel>& panels,
        double weightSum, std::string& trace, const std::string& key )
    {
        const std::size_t n = before.cellFluidFraction_.size();
        const double h = before.cellSize_, area = h * h, dt = before.frozenPressureTimeStep_;
        struct transferEdge { int donor, recipient; double volume; bool closedEnd; };
        struct panelTransfers { std::vector<transferEdge> edges; std::vector<double> outgoing; };
        std::vector<panelTransfers> transfers;
        std::vector<double> uSum( interval.uTransfer.size() ), vSum( interval.vTransfer.size() );
        std::vector<double> modelVolume( n );
        for( std::size_t i = 0; i < n; ++i )
            modelVolume[i] = area * before.previousCellFluidFraction_[i];
        double unserved = 0.0, closedTransfer = 0.0;
        int limitedDonors = 0;
        for( const auto& panel : panels )
        {
            if( panel.weight <= 0.0 || panel.startFraction.size() != n )
                throw std::runtime_error( key + " invalid transport panel" );
            auto& batch = transfers.emplace_back();
            batch.outgoing.resize( n );
            const auto face = [&]( bool isU, int x, int y )
            {
                const int index = isU ? before.uIndex( x, y ) : before.vIndex( x, y );
                const double aperture = isU ? panel.uOpen[index] : panel.vOpen[index];
                const double velocity = isU ? uVelocity[index] : vVelocity[index];
                const double amount = dt * h * ( panel.weight / weightSum ) * aperture * velocity;
                if( !std::isfinite( amount ) ) throw std::runtime_error( key + " invalid panel transfer" );
                ( isU ? uSum : vSum )[index] += amount;
                if( amount == 0.0 ) return;
                const int a = before.cellIndex( x - ( isU ? 1 : 0 ), y - ( isU ? 0 : 1 ) );
                const int b = before.cellIndex( x, y );
                const int donor = amount > 0.0 ? a : b, recipient = amount > 0.0 ? b : a;
                const bool closed = ( isU ? before.cutCellUOpen( x, y ) : before.cutCellVOpen( x, y ) ) == 0.0f;
                const double volume = std::abs( amount );
                batch.outgoing[donor] += volume;
                batch.edges.push_back( { donor, recipient, volume, closed } );
                if( closed ) closedTransfer += volume;
                modelVolume[a] -= amount; modelVolume[b] += amount;
            };
            for( int y = 0; y < before.height_; ++y )
                for( int x = 1; x < before.width_; ++x ) face( true, x, y );
            for( int y = 1; y < before.height_; ++y )
                for( int x = 0; x < before.width_; ++x ) face( false, x, y );
            for( std::size_t i = 0; i < n; ++i )
            {
                const double capacity = area * panel.startFraction[i];
                if( !std::isfinite( capacity ) || capacity < 0.0 || capacity > area )
                    throw std::runtime_error( key + " invalid panel start volume" );
                if( batch.outgoing[i] > capacity )
                {
                    ++limitedDonors;
                    unserved += batch.outgoing[i] - capacity;
                }
            }
        }
        double transferError = 0.0, transferL1 = 0.0;
        const auto compare = [&]( const std::vector<double>& actual, const std::vector<double>& expected )
        {
            for( std::size_t i = 0; i < actual.size(); ++i )
            {
                transferError += std::abs( actual[i] - expected[i] );
                transferL1 += std::abs( expected[i] );
            }
        };
        compare( uSum, interval.uTransfer ); compare( vSum, interval.vTransfer );
        if( transferError > 1.0e-12 * std::max( transferL1, 1.0e-12 ) )
            throw std::runtime_error( key + " chronological transfers differ from interval ledger" );
        double geometryModelL1 = 0.0, negativeModelVolume = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            geometryModelL1 += std::abs( modelVolume[i] - area * before.cellFluidFraction_[i] );
            negativeModelVolume += std::max( 0.0, -modelVolume[i] );
        }
        const auto out = [&]( const std::string& name, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( key + name + " nonfinite chronological output" );
            appendScalar( trace, key + name, value );
        };
        out( ".panels", static_cast<int>( panels.size() ) );
        out( ".transferErrorL1", transferError ); out( ".transferL1", transferL1 );
        out( ".closedTransferL1", closedTransfer ); out( ".limitedPanelDonors", limitedDonors );
        out( ".unservedPanelVolume", unserved );
        out( ".geometryModelL1", geometryModelL1 ); out( ".negativeModelVolume", negativeModelVolume );
        for( bool uniform : { true, false } )
        {
            std::vector<double> mass = before.tracerMass_;
            if( mass.size() != n ) throw std::runtime_error( key + " missing tracer state" );
            if( uniform )
                for( std::size_t i = 0; i < n; ++i ) mass[i] = area * before.previousCellFluidFraction_[i];
            double initialMass = 0.0, moved = 0.0, closedMass = 0.0, maxPanelError = 0.0;
            double trappedBefore = 0.0;
            for( std::size_t i = 0; i < n; ++i )
            {
                if( !std::isfinite( mass[i] ) || mass[i] < 0.0 )
                    throw std::runtime_error( key + " invalid initial mass" );
                initialMass += mass[i];
                if( before.previousCellFluidFraction_[i] == 0.0f ) trappedBefore += mass[i];
            }
            for( std::size_t step = 0; step < panels.size(); ++step )
            {
                const auto& panel = panels[step];
                const auto& batch = transfers[step];
                std::vector<double> remaining = mass, received( n );
                for( const auto& e : batch.edges )
                {
                    const double capacity = area * panel.startFraction[e.donor];
                    if( capacity <= 0.0 ) continue;
                    const double request = mass[e.donor] *
                        ( e.volume / std::max( capacity, batch.outgoing[e.donor] ) );
                    const double amount = std::min( remaining[e.donor], request );
                    remaining[e.donor] -= amount;
                    received[e.recipient] += amount;
                    moved += amount;
                    if( e.closedEnd ) closedMass += amount;
                }
                double total = 0.0;
                for( std::size_t i = 0; i < n; ++i )
                {
                    mass[i] = remaining[i] + received[i];
                    if( !std::isfinite( mass[i] ) || mass[i] < 0.0 )
                        throw std::runtime_error( key + " invalid panel tracer mass" );
                    total += mass[i];
                }
                maxPanelError = std::max( maxPanelError, std::abs( total - initialMass ) );
                if( maxPanelError > 1.0e-12 * std::max( 1.0, initialMass ) )
                    throw std::runtime_error( key + " chronological mass conservation failed" );
            }
            double finalMass = 0.0, trappedAfter = 0.0, closingMass = 0.0;
            double geometryError = 0.0, modelError = 0.0, maxConcentration = 0.0;
            for( std::size_t i = 0; i < n; ++i )
            {
                finalMass += mass[i];
                const double endVolume = area * before.cellFluidFraction_[i];
                if( endVolume == 0.0 )
                {
                    trappedAfter += mass[i];
                    if( before.previousCellFluidFraction_[i] > 0.0f ) closingMass += mass[i];
                }
                else maxConcentration = std::max( maxConcentration, mass[i] / endVolume );
                if( uniform )
                {
                    geometryError += std::abs( mass[i] - endVolume );
                    modelError += std::abs( mass[i] - modelVolume[i] );
                }
            }
            const std::string prefix = uniform ? ".uniform" : ".seeded";
            out( prefix + ".beforeMass", initialMass ); out( prefix + ".afterMass", finalMass );
            out( prefix + ".massError", finalMass - initialMass );
            out( prefix + ".maxPanelMassError", maxPanelError );
            out( prefix + ".movedMass", moved ); out( prefix + ".closedFaceMass", closedMass );
            out( prefix + ".trappedBefore", trappedBefore ); out( prefix + ".trappedAfter", trappedAfter );
            out( prefix + ".closingMass", closingMass );
            out( prefix + ".uniformGeometryError", geometryError );
            out( prefix + ".uniformModelError", modelError );
            out( prefix + ".maxEndConcentration", maxConcentration );
            appendVector( trace, key + prefix + ".mass", mass );
            std::printf( "%s%s panels %zu mass error %.9g trapped %.9g closing mass %.9g "
                "geometry error %.9g model error %.9g max end concentration %.9g\n",
                key.c_str(), prefix.c_str(), panels.size(), finalMass - initialMass,
                trappedAfter, closingMass, geometryError, modelError, maxConcentration );
        }
        std::printf( "%s transfer error %.9g unserved panel volume %.9g limited donors %d "
            "geometry/model gap %.9g negative model volume %.9g\n",
            key.c_str(), transferError, unserved, limitedDonors, geometryModelL1, negativeModelVolume );
    }

    // R4f: audit the model's predicted solid volume change against an
    // actual geometric rebuild after a pressure-induced pose perturbation.
    // This is a counterfactual free-body displacement, not a World step.
    static void captureGeometricConservation( const fluidField2& before,
        const intervalTransport& transport, const std::vector<fluidDetail::intervalPressureResponse>& force,
        const std::vector<double>& bodyVolume, const std::vector<double>& removedVolume,
        const std::vector<double>& correctedResidual, std::string& trace, const std::string& key,
        double responseScale = 1.0 )
    {
        const auto& obstacles = before.frozenPressureObstacles_;
        const std::size_t n = before.cellFluidFraction_.size();
        const double h = before.cellSize_, area = h * h, dt = before.frozenPressureTimeStep_;
        if( obstacles.size() != force.size() || bodyVolume.size() != n ||
            removedVolume.size() != n || correctedResidual.size() != n )
            throw std::runtime_error( key + " invalid conservation inputs" );

        std::vector<double> fluxVolume( n );
        for( std::size_t i = 0; i < n; ++i )
            fluxVolume[i] = area * before.previousCellFluidFraction_[i];
        for( int y = 0; y < before.height_; ++y )
            for( int x = 1; x < before.width_; ++x )
            {
                const double q = transport.uTransfer[before.uIndex( x, y )];
                fluxVolume[before.cellIndex( x - 1, y )] -= q;
                fluxVolume[before.cellIndex( x, y )] += q;
            }
        for( int y = 1; y < before.height_; ++y )
            for( int x = 0; x < before.width_; ++x )
            {
                const double q = transport.vTransfer[before.vIndex( x, y )];
                fluxVolume[before.cellIndex( x, y - 1 )] -= q;
                fluxVolume[before.cellIndex( x, y )] += q;
            }

        const auto out = [&]( const std::string& name, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( key + name + " nonfinite geometric conservation output" );
            appendScalar( trace, key + name, value );
        };
        double bodyL1 = 0.0, removedL1 = 0.0, residualL1 = 0.0, geometryGap = 0.0;
        double identityError = 0.0, negativeVolume = 0.0, bodySum = 0.0, removedSum = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double gap = fluxVolume[i] - area * before.cellFluidFraction_[i];
            identityError = std::max( identityError,
                std::abs( gap - ( bodyVolume[i] - removedVolume[i] - correctedResidual[i] ) ) );
            bodyL1 += std::abs( bodyVolume[i] );
            removedL1 += std::abs( removedVolume[i] );
            residualL1 += std::abs( correctedResidual[i] );
            geometryGap += std::abs( gap );
            negativeVolume += std::max( 0.0, -fluxVolume[i] );
            bodySum += bodyVolume[i];
            removedSum += removedVolume[i];
        }
        if( identityError > 1.0e-12 * std::max( 1.0, geometryGap ) )
            throw std::runtime_error( key + " geometric gap decomposition failed" );
        out( ".bodyVolumeL1", bodyL1 ); out( ".compatibilityL1", removedL1 );
        out( ".correctedResidualL1", residualL1 ); out( ".geometryGapL1", geometryGap );
        out( ".identityError", identityError ); out( ".negativeFluxVolume", negativeVolume );
        out( ".bodyVolumeSum", bodySum ); out( ".compatibilitySum", removedSum );

        // Same pressure impulse and dt^2 displacement as the Schur volume
        // term. Use exact endpoint poses, not reconstructed angles.
        std::vector<vec2> displacement( obstacles.size() );
        std::vector<float> rotation( obstacles.size() );
        for( std::size_t j = 0; j < obstacles.size(); ++j )
        {
            displacement[j] = { static_cast<float>( responseScale * ( dt * dt * obstacles[j].inverseMass * force[j].fx ) ),
                static_cast<float>( responseScale * ( dt * dt * obstacles[j].inverseMass * force[j].fy ) ) };
            rotation[j] = static_cast<float>( responseScale * ( dt * dt * obstacles[j].inverseInertia * force[j].torque ) );
            const auto k = ".body[" + std::to_string( j ) + "]";
            out( k + ".dx", displacement[j].x ); out( k + ".dy", displacement[j].y );
            out( k + ".angle", rotation[j] );
        }
        fluidField2 geometry = before;
        geometry.setPrePressureSnapshotObserver( {} );
        setStageObserver( geometry, {} );
        geometry.setCoupledPressureWarmStartEnabled( false );
        geometry.setCoupledPressureWarmStartProbeEnabled( false );
        const auto rebuild = [&]( float factor )
        {
            std::vector<fluidShapeObstacle2> poses( obstacles.begin(), obstacles.end() );
            if( factor != 0.0f )
                for( std::size_t j = 0; j < poses.size(); ++j )
                {
                    const rot2 delta = rot2::FromRadians( factor * rotation[j] );
                    poses[j].center = obstacles[j].center + factor * displacement[j];
                    poses[j].transform.position = poses[j].center +
                        Rotate( delta, obstacles[j].transform.position - obstacles[j].center );
                    poses[j].transform.rotation = delta * obstacles[j].transform.rotation;
                }
            geometry.updateObstacles( poses );
            geometry.updateCutCellGeometry( poses );
            if( !geometry.cutCellTransition_.usedDisjointAnalyticGeometry )
                throw std::runtime_error( key + " nonanalytic perturbed endpoint geometry" );
            std::vector<double> volumes( n );
            for( std::size_t i = 0; i < n; ++i )
            {
                const double fraction = geometry.cellFluidFraction_[i];
                if( !std::isfinite( fraction ) || fraction < 0.0 || fraction > 1.0 )
                    throw std::runtime_error( key + " invalid perturbed geometry" );
                volumes[i] = area * fraction;
            }
            return volumes;
        };
        const auto endpoint = rebuild( 0.0f );
        double endpointRebuildL1 = 0.0;
        for( std::size_t i = 0; i < n; ++i )
            endpointRebuildL1 += std::abs( endpoint[i] - area * before.cellFluidFraction_[i] );
        if( endpointRebuildL1 != 0.0 )
            throw std::runtime_error( key + " unchanged endpoint reconstruction differs" );
        out( ".endpointRebuildL1", endpointRebuildL1 );

        // Independent endpoint derivative from the same boundary coefficient
        // assembly, to distinguish time averaging from geometric linearization.
        std::fill( geometry.divergence_.begin(), geometry.divergence_.end(), 0.0f );
        if( !geometry.solveCoupledPressure( static_cast<float>( dt ), 200, obstacles, true ) )
            throw std::runtime_error( key + " endpoint coefficient assembly failed" );
        std::vector<double> endpointLinear( n );
        for( const auto& f : geometry.coupledPressureScratch_.faces )
        {
            const auto j = static_cast<std::size_t>( f.owner );
            endpointLinear[f.cell] += static_cast<double>( f.fx ) * displacement[j].x +
                static_cast<double>( f.fy ) * displacement[j].y + f.torque * rotation[j];
        }

        const auto plus = rebuild( 1.0f );
        const auto minus = rebuild( -1.0f );
        const auto half = rebuild( 0.5f );
        double actualL1 = 0.0, actualSum = 0.0, meanLinearError = 0.0, endLinearError = 0.0;
        double centeredError = 0.0, halfError = 0.0, predictedGapL1 = 0.0, predictedWithCompatibilityL1 = 0.0;
        double meanVsEndL1 = 0.0;
        int newlyClosed = 0, newlyOpen = 0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double change = plus[i] - endpoint[i];
            const double central = 0.5 * ( plus[i] - minus[i] );
            actualL1 += std::abs( change ); actualSum += change;
            meanLinearError += std::abs( change - bodyVolume[i] );
            endLinearError += std::abs( change - endpointLinear[i] );
            centeredError += std::abs( central - endpointLinear[i] );
            halfError += std::abs( 2.0 * ( half[i] - endpoint[i] ) - change );
            meanVsEndL1 += std::abs( bodyVolume[i] - endpointLinear[i] );
            predictedGapL1 += std::abs( fluxVolume[i] - plus[i] );
            predictedWithCompatibilityL1 += std::abs( fluxVolume[i] - plus[i] + removedVolume[i] );
            newlyClosed += endpoint[i] > 0.0 && plus[i] == 0.0;
            newlyOpen += endpoint[i] == 0.0 && plus[i] > 0.0;
        }
        out( ".actualVolumeChangeL1", actualL1 ); out( ".actualVolumeChangeSum", actualSum );
        out( ".meanLinearErrorL1", meanLinearError ); out( ".endpointLinearErrorL1", endLinearError );
        out( ".centeredLinearErrorL1", centeredError ); out( ".halfScaleErrorL1", halfError );
        out( ".meanVsEndpointLinearL1", meanVsEndL1 );
        out( ".predictedGeometryGapL1", predictedGapL1 );
        out( ".predictedWithCompatibilityGapL1", predictedWithCompatibilityL1 );
        out( ".newlyClosed", newlyClosed ); out( ".newlyOpen", newlyOpen );

        const int witness = before.cellIndex( 17, 13 );
        out( ".witness.endVolume", endpoint[witness] ); out( ".witness.fluxVolume", fluxVolume[witness] );
        out( ".witness.meanLinear", bodyVolume[witness] ); out( ".witness.endLinear", endpointLinear[witness] );
        out( ".witness.plusVolume", plus[witness] ); out( ".witness.minusVolume", minus[witness] );
        out( ".witness.halfVolume", half[witness] ); out( ".witness.compatibility", removedVolume[witness] );
        std::printf( "%s gap %.9g m^2 body L1 %.9g compatibility L1 %.9g residual L1 %.9g "
            "identity %.9g actual perturbation L1 %.9g mean linear error %.9g endpoint linear error %.9g "
            "perturbed gap %.9g compatibility-adjusted gap %.9g newly closed/open %d/%d\n",
            key.c_str(), geometryGap, bodyL1, removedL1, residualL1, identityError,
            actualL1, meanLinearError, endLinearError, predictedGapL1,
            predictedWithCompatibilityL1, newlyClosed, newlyOpen );
    }

    // R4h: matched fresh Worlds initialized from the actual frozen obstacle
    // state. Step the real rigid solver with/without pressure load, never
    // call the fluid step. No copy of live World contact/cache history.
    static void captureWorldPressureResponse( const fluidField2& before,
        std::span<const fluidObstacleForce2> baselineFeedback,
        const std::vector<fluidDetail::intervalPressureResponse>& candidate,
        std::string& trace, const std::string& key )
    {
        using namespace sandbox;
        const auto& initial = before.frozenPressureObstacles_;
        const float dt = before.frozenPressureTimeStep_;
        const double area = static_cast<double>( before.cellSize_ ) * before.cellSize_;
        if( initial.size() != 3 || baselineFeedback.size() != 3 || candidate.size() != 3 || dt <= 0.0f )
            throw std::runtime_error( key + " invalid World response fixture" );
        const auto run = [&]( int substeps, const std::array<fluidObstacleForce2, 3>& load )
        {
            rigidBodyDemo scene{ demoKind::windFlow };
            auto& world = scene.getWorld();
            if( scene.getShapes().size() != initial.size() )
                throw std::runtime_error( key + " World fixture shape count changed" );
            std::vector<fluidShapeObstacle2> result( initial.begin(), initial.end() );
            for( std::size_t j = 0; j < initial.size(); ++j )
            {
                const auto id = scene.getShapes()[j].bodyHandle;
                if( 1.0f / world.GetBodyMass( id ) != initial[j].inverseMass ||
                    1.0f / world.GetBodyRotationalInertia( id ) != initial[j].inverseInertia ||
                    world.GetBodyLinearDamping( id ) != 0.0f || world.GetBodyAngularDamping( id ) != 0.0f )
                    throw std::runtime_error( key + " World mass/damping differs from free response model" );
                world.SetBodyTransform( id, initial[j].transform );
                world.SetBodyLinearVelocity( id, initial[j].linearVelocity );
                world.SetBodyAngularVelocity( id, initial[j].angularVelocity );
                const vec2 center = TransformPoint( world.GetBodyTransform( id ), world.GetBodyLocalCenter( id ) );
                if( center.x != initial[j].center.x || center.y != initial[j].center.y )
                    throw std::runtime_error( key + " World initial COM differs" );
                world.ApplyForceToCenter( id, load[j].force );
                world.ApplyTorque( id, load[j].torque );
            }
            world.Step( dt, substeps );
            for( std::size_t j = 0; j < initial.size(); ++j )
            {
                const auto id = scene.getShapes()[j].bodyHandle;
                if( world.GetBody( id ).contactCount != 0 || world.GetBody( id ).jointCount != 0 )
                    throw std::runtime_error( key + " unexpected constraint in free-body comparison" );
                auto& body = result[j];
                body.transform = world.GetBodyTransform( id );
                body.center = TransformPoint( body.transform, world.GetBodyLocalCenter( id ) );
                body.linearVelocity = world.GetBodyLinearVelocity( id );
                body.angularVelocity = world.GetBodyAngularVelocity( id );
                if( !IsFinite( body.center ) || !IsFinite( body.linearVelocity ) ||
                    !std::isfinite( body.angularVelocity ) )
                    throw std::runtime_error( key + " nonfinite World response" );
            }
            return result;
        };
        fluidField2 geometry = before;
        geometry.setPrePressureSnapshotObserver( {} );
        setStageObserver( geometry, {} );
        const auto rebuild = [&]( const std::vector<fluidShapeObstacle2>& poses )
        {
            geometry.updateObstacles( poses );
            geometry.updateCutCellGeometry( poses );
            if( !geometry.cutCellTransition_.usedDisjointAnalyticGeometry )
                throw std::runtime_error( key + " World response geometry not analytic" );
            std::vector<double> volume;
            volume.reserve( geometry.cellFluidFraction_.size() );
            for( float fraction : geometry.cellFluidFraction_ )
            {
                if( !std::isfinite( fraction ) || fraction < 0.0f || fraction > 1.0f )
                    throw std::runtime_error( key + " invalid World response fraction" );
                volume.push_back( area * fraction );
            }
            return volume;
        };
        const std::array<fluidObstacleForce2, 3> zero{};
        for( int substeps : { 1, 2, 4, 8 } )
        {
            const auto control = run( substeps, zero );
            const auto controlVolume = rebuild( control );
            // Semi-implicit Euler under constant force, no damping,
            // constraints or velocity caps: sum the N acceleration kicks.
            const double cadence = ( substeps + 1.0 ) / ( 2.0 * substeps );
            for( bool corrected : { false, true } )
            {
                std::array<fluidObstacleForce2, 3> load{};
                for( std::size_t j = 0; j < load.size(); ++j )
                {
                    if( corrected )
                    {
                        load[j].force = { static_cast<float>( candidate[j].fx ), static_cast<float>( candidate[j].fy ) };
                        load[j].torque = static_cast<float>( candidate[j].torque );
                    }
                    else load[j] = baselineFeedback[j];
                }
                const auto loaded = run( substeps, load );
                const auto prefix = key + ".steps" + std::to_string( substeps ) + ( corrected ? ".candidate" : ".baseline" );
                const auto out = [&]( const std::string& name, auto value )
                {
                    if( !std::isfinite( static_cast<double>( value ) ) )
                        throw std::runtime_error( prefix + name + " nonfinite response output" );
                    appendScalar( trace, prefix + name, value );
                };
                auto full = control, discrete = control;
                double maxFullError = 0.0, maxDiscreteError = 0.0;
                for( std::size_t j = 0; j < load.size(); ++j )
                {
                    const auto k = ".body[" + std::to_string( j ) + "]";
                    const vec2 dx = loaded[j].center - control[j].center;
                    const vec2 dv = loaded[j].linearVelocity - control[j].linearVelocity;
                    const float dw = loaded[j].angularVelocity - control[j].angularVelocity;
                    const rot2 relative = loaded[j].transform.rotation * Inverse( control[j].transform.rotation );
                    const float da = deterministicAtan2( relative.s, relative.c );
                    const double fullX = static_cast<double>( dt ) * dt * initial[j].inverseMass * load[j].force.x;
                    const double fullY = static_cast<double>( dt ) * dt * initial[j].inverseMass * load[j].force.y;
                    const double fullAngle = static_cast<double>( dt ) * dt * initial[j].inverseInertia * load[j].torque;
                    const auto predict = [&]( fluidShapeObstacle2& pose, double factor )
                    {
                        const rot2 delta = rot2::FromRadians( static_cast<float>( factor * fullAngle ) );
                        pose.center = control[j].center + vec2{ static_cast<float>( factor * fullX ),
                            static_cast<float>( factor * fullY ) };
                        pose.transform.position = pose.center +
                            Rotate( delta, control[j].transform.position - control[j].center );
                        pose.transform.rotation = delta * control[j].transform.rotation;
                    };
                    predict( full[j], 1.0 ); predict( discrete[j], cadence );
                    out( k + ".force.x", load[j].force.x ); out( k + ".force.y", load[j].force.y );
                    out( k + ".torque", load[j].torque );
                    out( k + ".dx", dx.x ); out( k + ".dy", dx.y ); out( k + ".da", da );
                    out( k + ".dv.x", dv.x ); out( k + ".dv.y", dv.y ); out( k + ".dw", dw );
                    out( k + ".full.dx", fullX ); out( k + ".full.dy", fullY ); out( k + ".full.da", fullAngle );
                    out( k + ".cadence.dx", cadence * fullX ); out( k + ".cadence.dy", cadence * fullY );
                    out( k + ".cadence.da", cadence * fullAngle );
                    out( k + ".cadence.error.x", dx.x - cadence * fullX );
                    out( k + ".cadence.error.y", dx.y - cadence * fullY );
                    out( k + ".cadence.error.angle", da - cadence * fullAngle );
                    maxFullError = std::max( maxFullError, std::max( std::abs( dx.x - fullX ), std::abs( dx.y - fullY ) ) );
                    maxDiscreteError = std::max( maxDiscreteError, std::max(
                        std::abs( dx.x - cadence * fullX ), std::abs( dx.y - cadence * fullY ) ) );
                }
                const auto actualVolume = rebuild( loaded ), fullVolume = rebuild( full ),
                    discreteVolume = rebuild( discrete );
                double actualL1 = 0.0, fullError = 0.0, discreteError = 0.0, actualSum = 0.0;
                int closed = 0, opened = 0;
                for( std::size_t i = 0; i < actualVolume.size(); ++i )
                {
                    actualL1 += std::abs( actualVolume[i] - controlVolume[i] );
                    actualSum += actualVolume[i] - controlVolume[i];
                    fullError += std::abs( fullVolume[i] - actualVolume[i] );
                    discreteError += std::abs( discreteVolume[i] - actualVolume[i] );
                    closed += controlVolume[i] > 0.0 && actualVolume[i] == 0.0;
                    opened += controlVolume[i] == 0.0 && actualVolume[i] > 0.0;
                }
                out( ".cadenceFactor", cadence );
                out( ".maxFullPositionError", maxFullError ); out( ".maxCadencePositionError", maxDiscreteError );
                out( ".actualVolumeL1", actualL1 ); out( ".actualVolumeSum", actualSum );
                out( ".fullVolumeErrorL1", fullError ); out( ".cadenceVolumeErrorL1", discreteError );
                out( ".closed", closed ); out( ".opened", opened );
                std::printf( "%s factor %.9g position error full %.9g cadence %.9g m "
                    "actual volume L1 %.9g full error %.9g cadence error %.9g m^2 closed/open %d/%d\n",
                    prefix.c_str(), cadence, maxFullError, maxDiscreteError, actualL1,
                    fullError, discreteError, closed, opened );
            }
        }
    }

    // R4j: hand off accepted pressure/velocity/feedback together. The
    // historical interval transfer is not used as an endpoint flux.
    struct continuationPressure
    {
        std::vector<float> u, v, pressure;
        std::vector<fluidObstacleForce2> feedback;
        // Signed interval transfers accumulated in physical volume units.
        std::vector<double> fluxVolume;
        std::vector<double> meanU, meanV, intervalU, intervalV;
        std::vector<double> bodyVolume, compatibilityVolume, linearVolumeResidual;
        std::vector<transportPanel> geometricPanels;
        double geometricWeightSum = 0.0;
        std::vector<int> component;
        double trueResidualSquared = 0.0, initialResidualSquared = 0.0;
        double seedResidualSquared = 0.0;
        int initialIterations = 0;
        int correctionIterations = 0, defectEvaluations = 0;
        int missedMidpointCells = 0, selectedEndpointDepth = 0;
        double initialGap = 0.0, finalGap = 0.0, negativeVolume = 0.0, intervalPeak = 0.0;
        int uncoveredEndpointFaces = 0;
        bool stalled = false;
    };

    // R4a: independent, test-only interval-average pressure model. Reuse the
    // production analytic geometry and boundary assembly at quadrature poses.
    // Do not write averaged apertures into the live endpoint field.
    static void captureSpaceTimeProjection( const fluidField2& before, const fluidField2& baseline,
        std::span<const fluidObstacleForce2> baselineFeedback, std::string& trace,
        const std::string& key, int samples, const std::string& handoffKey,
        bool requireTarget = true, int endpointDepth = 0,
        const std::string& topologyKey = {}, double responseScale = 1.0,
        continuationPressure* continuation = nullptr, int correctionBudget = 3,
        bool adaptiveEndpoint = false, forwardTiming* timing = nullptr,
        double relativeSquaredTolerance = 1.0e-16, bool warmStart = false,
        std::span<const float> previousForwardPressure = {}, bool captureEndpoint = false,
        bool volumeWeightedCompatibility = false, bool skipFixedBodyProducts = false,
        double closingVolumeRelativeTolerance = 0.0, bool refineEndpointFaces = false )
    {
        if( refineEndpointFaces && ( !adaptiveEndpoint || !captureEndpoint || responseScale != 0.0 ) )
            throw std::runtime_error( key + " invalid endpoint-face refinement fixture" );
        if( closingVolumeRelativeTolerance != 0.0 &&
            ( !std::isfinite( closingVolumeRelativeTolerance ) || closingVolumeRelativeTolerance < 0.0 ||
                !skipFixedBodyProducts || responseScale != 0.0 || !warmStart || !volumeWeightedCompatibility ) )
            throw std::runtime_error( key + " invalid closing-cell residual probe" );
        if( !previousForwardPressure.empty() && !warmStart )
            throw std::runtime_error( key + " previous pressure requires warm start" );
        if( warmStart && ( !continuation || !adaptiveEndpoint || correctionBudget != 0 ||
            ( relativeSquaredTolerance != 1.0e-10 &&
                !( skipFixedBodyProducts && responseScale == 0.0 && relativeSquaredTolerance == 1.0e-16 ) ) ) )
            throw std::runtime_error( key + " invalid forward warm-start fixture" );
        if( relativeSquaredTolerance != 1.0e-16 && ( !adaptiveEndpoint ||
            ( relativeSquaredTolerance != 1.0e-12 && relativeSquaredTolerance != 1.0e-10 ) ) )
            throw std::runtime_error( key + " invalid experimental residual budget" );
        const bool fixedGeometry = responseScale == 0.0;
        if( fixedGeometry && ( !continuation || !captureEndpoint || correctionBudget != 0 ||
            !adaptiveEndpoint || ( !skipFixedBodyProducts &&
                ( warmStart || relativeSquaredTolerance != 1.0e-16 ) ) ) )
            throw std::runtime_error( key + " invalid actual-geometry diagnostic" );
        if( skipFixedBodyProducts && !fixedGeometry )
            throw std::runtime_error( key + " fast fixed operator requires fixed geometry" );
        if( volumeWeightedCompatibility && !fixedGeometry )
            throw std::runtime_error( key + " weighted compatibility requires fixed geometry" );
        if( !fixedGeometry && responseScale != 1.0 && responseScale != 0.75 )
            throw std::runtime_error( key + " unsupported free-body cadence response" );
        if( correctionBudget < 0 || correctionBudget > 3 || ( !continuation && correctionBudget != 3 ) )
            throw std::runtime_error( key + " invalid correction budget" );
        if( adaptiveEndpoint && ( !continuation || endpointDepth != 16 || correctionBudget != 0 ) )
            throw std::runtime_error( key + " invalid adaptive endpoint fixture" );
        const auto& start = before.previousObstacles_;
        const auto& finish = before.frozenPressureObstacles_;
        const std::size_t n = before.cellFluidFraction_.size(), owners = finish.size();
        const double h = before.cellSize_, dt = before.frozenPressureTimeStep_;
        if( samples <= 0 || owners != 3 || start.size() != owners || baselineFeedback.size() != owners ||
            !before.volumeSourceValid_ || dt <= 0.0 )
            throw std::runtime_error( key + " invalid frozen geometry history" );
        using response = fluidDetail::intervalPressureResponse;
        // Includes scratch preparation as well as the ordered quadrature.
        const auto quadratureStart = timing ? benchmarkClock::now() : benchmarkClock::time_point{};
        auto preparedGeometry = before.prepareIntervalPressureGeometry( samples, endpointDepth,
            adaptiveEndpoint, refineEndpointFaces );
        auto& [meanU, meanV, fraction, coefficients, transportPanels, weightSum] = preparedGeometry.integral;
        const auto& missed = preparedGeometry.missed;
        const auto& missedFaces = preparedGeometry.missedFaces;
        if( endpointDepth > 0 && missed.empty() && !continuation )
            throw std::runtime_error( key + " historical midpoint coverage failure not reproduced" );
        endpointDepth = preparedGeometry.endpointDepth;
        for( const auto& f : missedFaces )
        {
            const int index = f.isU ? before.uIndex( f.x, f.y ) : before.vIndex( f.x, f.y );
            const double mean = ( f.isU ? meanU : meanV )[index];
            const double end = f.isU ? before.cutCellUOpen( f.x, f.y ) : before.cutCellVOpen( f.x, f.y );
            std::printf( "%s R4au missed endpoint %c(%d,%d) end %.17g refinedMean %.17g depth %d\n",
                key.c_str(), f.isU ? 'U' : 'V', f.x, f.y, end, mean, endpointDepth );
            if( !( mean > 0.0 ) )
                throw std::runtime_error( key + " R4au endpoint-face refinement did not recover support" );
        }
        if( endpointDepth > 0 )
        {
            appendScalar( trace, topologyKey + ".missedCount", static_cast<int>( missed.size() ) );
            appendScalar( trace, topologyKey + ".panelCount", samples + 2 * endpointDepth );
            appendScalar( trace, topologyKey + ".weightSum", weightSum );
            for( int i : missed )
            {
                const int x = i % before.width_, y = i / before.width_;
                const double opening = meanU[before.uIndex( x, y )] + meanU[before.uIndex( x + 1, y )] +
                    meanV[before.vIndex( x, y )] + meanV[before.vIndex( x, y + 1 )];
                const auto prefix = topologyKey + ".cell[" + std::to_string( i ) + "]";
                appendScalar( trace, prefix + ".previous", before.previousCellFluidFraction_[i] );
                appendScalar( trace, prefix + ".current", before.cellFluidFraction_[i] );
                appendScalar( trace, prefix + ".meanFraction", fraction[i] );
                appendScalar( trace, prefix + ".meanOpeningSum", opening );
                std::printf( "%s x %d y %d previous %.9g current %.9g mean fraction %.9g mean opening %.9g\n",
                    prefix.c_str(), x, y, before.previousCellFluidFraction_[i], before.cellFluidFraction_[i],
                    fraction[i], opening );
                if( fraction[i] <= 0.0 || opening <= 0.0 )
                    throw std::runtime_error( prefix + " endpoint refinement did not recover open support" );
            }
        }

        const auto linearStart = timing ? benchmarkClock::now() : benchmarkClock::time_point{};
        if( timing ) timing->quadratureMs = elapsedMs( quadratureStart, linearStart );
        const std::span<const float> seed = previousForwardPressure.empty() ?
            std::span<const float>( baseline.pressure_ ) : previousForwardPressure;
        const fluidDetail::intervalPressureLinearOptions linearOptions{
            responseScale, volumeWeightedCompatibility, skipFixedBodyProducts, warmStart,
            relativeSquaredTolerance, closingVolumeRelativeTolerance };
        auto linear = before.solvePreparedIntervalPressure( preparedGeometry.integral, linearOptions, seed );
        auto& graph = linear.graph;
        auto& [edges, diagonal, rawDivergence, parent, count, active, regions, closedEndFaces, sourceSum] = graph;
        auto& rhs = linear.rhs;
        const auto& rhsResult = linear.rhsResult;
        const double scale = linear.scale;
        if( rhsResult.failedComponent >= 0 )
        {
            const int c = rhsResult.failedComponent;
            // A disappearing interval component has no endpoint
            // recipient. Preserve the failure and expose its geometry
            // and source/flux balance before choosing a remedy.
            std::fprintf( stderr,
                "%s endpointCapacity root %d cells %d rhs %.17g endpointFraction %.17g dt %.17g h %.17g\n",
                key.c_str(), c, count[c], rhsResult.componentRhs, rhsResult.componentVolume, dt, h );
            for( std::size_t j = 0; j < n; ++j )
            {
                if( parent[j] != c ) continue;
                const int x = static_cast<int>( j ) % before.width_;
                const int y = static_cast<int>( j ) / before.width_;
                const double previous = before.previousCellFluidFraction_[j];
                const double current = before.cellFluidFraction_[j];
                const double source = ( current - previous ) / dt;
                std::fprintf( stderr,
                    "%s endpointCapacity cell %zu x %d y %d previous %.17g current %.17g mean %.17g source %.17g fluxDivergence %.17g rhs %.17g meanOpen L/R/B/T %.17g %.17g %.17g %.17g\n",
                    key.c_str(), j, x, y, previous, current, fraction[j], source,
                    rawDivergence[j] - source, rhs[j],
                    meanU[before.uIndex( x, y )], meanU[before.uIndex( x + 1, y )],
                    meanV[before.vIndex( x, y )], meanV[before.vIndex( x, y + 1 )] );
            }
            throw std::runtime_error( key + " component lacks endpoint compatibility capacity" );
        }
        const double compatibilityL1 = rhsResult.compatibilityL1;
        if( !linear.solution )
            throw std::runtime_error( key + " missing interval pressure solution" );
        auto& force = linear.force;
        const fluidDetail::intervalPressureOperator pressureOperator( graph, coefficients, linear.bodies,
            force, linear.sums, responseScale, skipFixedBodyProducts );

        const auto dot = [&]( const std::vector<double>& a, const std::vector<double>& b )
        {
            double sum = 0.0;
            for( std::size_t i = 0; i < n; ++i ) sum += a[i] * b[i];
            return sum;
        };
        const auto& closingRows = linear.closingRows;
        auto& [p, ap, initial, seedSquared, trueSquared, unprojectedSquared, iterations] = *linear.solution;
        if( closingVolumeRelativeTolerance > 0.0 )
            std::printf( "%s R4am closing rows %zu relative tolerance %.9g PCG %d\n",
                key.c_str(), closingRows.size(), closingVolumeRelativeTolerance, iterations );
        if( timing ) timing->linearMs = elapsedMs( linearStart, benchmarkClock::now() );

        double peak = 0.0, endOpenPeak = 0.0, targetSpeed = 0.0, targetOpening = 0.0;
        double fluidQuadratic = 0.0, bodyQuadratic = 0.0, forceChange = 0.0, torqueChange = 0.0;
        const int targetIndex = before.vIndex( 17, 13 );
        for( const auto& e : edges )
        {
            const double difference = p[e.first] - p[e.second];
            const double velocity = fluidDetail::intervalFaceVelocity( e, p, h, dt );
            peak = std::max( peak, std::abs( velocity ) );
            const int x = e.index % ( before.width_ + ( e.isU ? 1 : 0 ) );
            const int y = e.index / ( before.width_ + ( e.isU ? 1 : 0 ) );
            if( ( e.isU ? before.cutCellUOpen( x, y ) : before.cutCellVOpen( x, y ) ) > 0.0f )
                endOpenPeak = std::max( endOpenPeak, std::abs( velocity ) );
            if( !e.isU && e.index == targetIndex ) { targetSpeed = velocity; targetOpening = e.a; }
            fluidQuadratic += e.a * difference * difference;
        }
        if( requireTarget && targetOpening <= 0.0 )
            throw std::runtime_error( key + " historical face absent from interval graph" );
        const auto emit = [&]( const std::string& name, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( key + name + " nonfinite prototype output" );
            appendScalar( trace, key + name, value );
        };
        for( std::size_t j = 0; j < owners; ++j )
        {
            const auto& f = force[j];
            bodyQuadratic += responseScale * ( static_cast<double>( finish[j].inverseMass ) * ( f.fx * f.fx + f.fy * f.fy ) +
                static_cast<double>( finish[j].inverseInertia ) * f.torque * f.torque );
            forceChange = std::max( forceChange, std::max(
                std::abs( f.fx - baselineFeedback[j].force.x ), std::abs( f.fy - baselineFeedback[j].force.y ) ) );
            torqueChange = std::max( torqueChange, std::abs( f.torque - baselineFeedback[j].torque ) );
            const auto k = ".body[" + std::to_string( j ) + "]";
            emit( k + ".force.x", f.fx ); emit( k + ".force.y", f.fy ); emit( k + ".torque", f.torque );
        }
        const double quadratic = dot( p, ap );
        const double adjointError = quadratic - fluidQuadratic - bodyQuadratic;
        if( std::abs( adjointError ) > 1.0e-10 * std::max( 1.0, std::abs( quadratic ) ) )
            throw std::runtime_error( key + " interval operator energy identity failed" );
        emit( ".active", active ); emit( ".regions", regions ); emit( ".closedEndFaces", closedEndFaces );
        emit( ".iterations", iterations ); emit( ".sourceSum", sourceSum );
        emit( ".compatibilityL1", compatibilityL1 ); emit( ".rhsSquared", initial );
        emit( ".projectedResidualSquared", trueSquared ); emit( ".unprojectedResidualSquared", unprojectedSquared );
        emit( ".peak", peak ); emit( ".endOpenPeak", endOpenPeak );
        emit( ".targetSpeed", targetSpeed ); emit( ".targetOpening", targetOpening );
        emit( ".baselineTargetSpeed", baseline.v_[targetIndex] );
        emit( ".fluidQuadratic", fluidQuadratic ); emit( ".bodyQuadratic", bodyQuadratic );
        emit( ".adjointError", adjointError ); emit( ".maxForceComponentChange", forceChange );
        emit( ".maxTorqueChange", torqueChange );
        std::printf( "%s interval peak %.9g m/s endpoint-open peak %.9g target %.9g opening %.9g "
            "PCG %d projected residual^2 %.9g unprojected residual^2 %.9g force delta %.9g torque delta %.9g "
            "compatibility L1 %.9g closed-end faces %d\n",
            key.c_str(), peak, endOpenPeak, targetSpeed, targetOpening, iterations,
            trueSquared, unprojectedSquared, forceChange, torqueChange, compatibilityL1, closedEndFaces );

        // R4b: independently reconstruct the integrated volume balance in
        // physical units (m^2 for unit depth), then compare two endpoint
        // representations. Neither representation is written to a field.
        // The original volume source and predicted body response are fixed.
        const double area = h * h;
        std::vector<double> volume( n ), correctedVolume( n ), velocityHandoff( n ), fluxHandoff( n );
        std::vector<double> bodyVolumeAudit( n ), removedVolumeAudit( n );
        double sourceVolume = 0.0, sourceL1 = 0.0, compatibilityVolume = 0.0;
        int closingCells = 0, openingCells = 0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double change = area * ( static_cast<double>( before.cellFluidFraction_[i] ) -
                before.previousCellFluidFraction_[i] );
            sourceVolume += change;
            sourceL1 += std::abs( change );
            closingCells += before.previousCellFluidFraction_[i] > 0.0f && before.cellFluidFraction_[i] == 0.0f;
            openingCells += before.previousCellFluidFraction_[i] == 0.0f && before.cellFluidFraction_[i] > 0.0f;
            double bodyVolume = 0.0;
            for( std::size_t j = 0; j < owners; ++j )
            {
                const auto& c = coefficients[i * owners + j];
                const auto& f = force[j];
                bodyVolume += responseScale * ( dt * dt * ( static_cast<double>( finish[j].inverseMass ) *
                    ( c.fx * f.fx + c.fy * f.fy ) +
                    static_cast<double>( finish[j].inverseInertia ) * c.torque * f.torque ) );
            }
            const double removed = dt * area * ( rawDivergence[i] + rhs[i] / scale );
            compatibilityVolume += std::abs( removed );
            bodyVolumeAudit[i] = bodyVolume;
            removedVolumeAudit[i] = removed;
            volume[i] = change + bodyVolume;
            correctedVolume[i] = volume[i] - removed;
            velocityHandoff[i] = correctedVolume[i];
            fluxHandoff[i] = correctedVolume[i];
        }

        const auto correctedSource = correctedVolume;
        std::vector<double> intervalUVelocity( before.u_.size() ), intervalVVelocity( before.v_.size() );
        intervalTransport transport{
            std::vector<double>( before.u_.size() ), std::vector<double>( before.v_.size() ),
            std::vector<double>( before.u_.size() ), std::vector<double>( before.v_.size() ) };
        double lostClosedTransfer = 0.0, velocityTransferError = 0.0, fluxTransferError = 0.0;
        double fluxHandoffPeak = 0.0, velocityHandoffPeak = 0.0;
        double targetFluxVelocity = 0.0, targetEndOpening = 0.0;
        double intervalTransferL1 = 0.0;
        int closedTransferFaces = 0;
        const auto addTransfer = []( std::vector<double>& v, const fluidDetail::intervalPressureEdge& e, double amount )
        {
            v[e.first] += amount; v[e.second] -= amount;
        };
        for( const auto& e : edges )
        {
            const double velocity = fluidDetail::intervalFaceVelocity( e, p, h, dt );
            const double transfer = fluidDetail::intervalFaceTransfer( e, velocity, h, dt );
            ( e.isU ? transport.uTransfer : transport.vTransfer )[e.index] = transfer;
            ( e.isU ? intervalUVelocity : intervalVVelocity )[e.index] = velocity;
            intervalTransferL1 += std::abs( transfer );
            addTransfer( volume, e, transfer );
            addTransfer( correctedVolume, e, transfer );
            const int stride = before.width_ + ( e.isU ? 1 : 0 );
            const int x = e.index % stride, y = e.index / stride;
            const double endOpen = e.isU ? before.cutCellUOpen( x, y ) : before.cutCellVOpen( x, y );
            ( e.isU ? transport.uEndpointFluid : transport.vEndpointFluid )[e.index] =
                endOpen > 0.0 ? velocity : 0.0;
            const double velocityTransfer = dt * h * endOpen * velocity;
            double fluxTransfer = 0.0;
            if( endOpen > 0.0 )
            {
                const double fluxVelocity = e.a * velocity / endOpen;
                if( !std::isfinite( fluxVelocity ) )
                    throw std::runtime_error( handoffKey + " nonfinite endpoint transfer velocity" );
                fluxTransfer = dt * h * endOpen * fluxVelocity;
                fluxHandoffPeak = std::max( fluxHandoffPeak, std::abs( fluxVelocity ) );
                velocityHandoffPeak = std::max( velocityHandoffPeak, std::abs( velocity ) );
                if( !e.isU && e.index == targetIndex )
                {
                    targetFluxVelocity = fluxVelocity; targetEndOpening = endOpen;
                }
            }
            else
            {
                lostClosedTransfer += std::abs( transfer );
                closedTransferFaces += transfer != 0.0;
            }
            velocityTransferError += std::abs( velocityTransfer - transfer );
            fluxTransferError += std::abs( fluxTransfer - transfer );
            addTransfer( velocityHandoff, e, velocityTransfer );
            addTransfer( fluxHandoff, e, fluxTransfer );
        }
        // Volume residual must agree with the independently evaluated,
        // unprojected matrix residual. Recompute it after ap was projected.
        pressureOperator.multiply( p, ap );
        double identityError = 0.0, rawL1 = 0.0, rawSum = 0.0, correctedL1 = 0.0;
        double correctedMax = 0.0, velocityL1 = 0.0, fluxL1 = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double expected = dt * area * ( ap[i] - rhs[i] ) / scale;
            identityError = std::max( identityError, std::abs( correctedVolume[i] - expected ) );
            rawSum += volume[i];
            rawL1 += std::abs( volume[i] );
            correctedL1 += std::abs( correctedVolume[i] );
            correctedMax = std::max( correctedMax, std::abs( correctedVolume[i] ) );
            velocityL1 += std::abs( velocityHandoff[i] );
            fluxL1 += std::abs( fluxHandoff[i] );
        }
        if( identityError > 1.0e-12 * std::max( 1.0, sourceL1 + intervalTransferL1 ) )
            throw std::runtime_error( handoffKey + " independent volume balance identity failed" );
        const auto out = [&]( const char* name, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( handoffKey + name + " nonfinite handoff output" );
            appendScalar( trace, handoffKey + name, value );
        };
        out( ".sourceVolume", sourceVolume );
        out( ".sourceL1", sourceL1 );
        out( ".compatibilityVolumeL1", compatibilityVolume );
        out( ".closingCells", closingCells );
        out( ".openingCells", openingCells );
        out( ".intervalTransferL1", intervalTransferL1 );
        out( ".rawVolumeSum", rawSum );
        out( ".rawVolumeL1", rawL1 );
        out( ".correctedVolumeL1", correctedL1 );
        out( ".correctedVolumeMax", correctedMax );
        out( ".identityError", identityError );
        out( ".closedTransferFaces", closedTransferFaces );
        out( ".lostClosedTransferL1", lostClosedTransfer );
        out( ".velocityTransferErrorL1", velocityTransferError );
        out( ".fluxTransferErrorL1", fluxTransferError );
        out( ".velocityHandoffVolumeL1", velocityL1 );
        out( ".fluxHandoffVolumeL1", fluxL1 );
        out( ".velocityHandoffPeak", velocityHandoffPeak );
        out( ".fluxHandoffPeak", fluxHandoffPeak );
        out( ".targetEndOpening", targetEndOpening );
        out( ".targetFluxVelocity", targetFluxVelocity );
        std::printf( "%s source %.9g m^2 compatibility L1 %.9g raw volume L1 %.9g corrected L1 %.9g "
            "identity error %.9g closing/opening cells %d/%d closed transfer %.9g "
            "velocity handoff L1 %.9g flux handoff L1 %.9g flux-matched peak %.9g m/s target %.9g m/s\n",
            handoffKey.c_str(), sourceVolume, compatibilityVolume, rawL1, correctedL1,
            identityError, closingCells, openingCells, lostClosedTransfer,
            velocityL1, fluxL1, fluxHandoffPeak, targetFluxVelocity );
        if( !continuation )
        {
            captureIntervalTransport( before, transport, correctedSource, correctedVolume, trace,
                "intervalTransport." + handoffKey );
            captureChronologicalTransport( before, transport, intervalUVelocity, intervalVVelocity,
                transportPanels, weightSum, trace, "chronologicalTracer." + handoffKey );
            captureGeometricConservation( before, transport, force, bodyVolumeAudit, removedVolumeAudit,
                correctedVolume, trace, "geometricConservation." + handoffKey, responseScale );
        }
        // R4g: defect correction on the fixed interval graph. The exact
        // binary32 endpoint rebuild replaces only the linearized volume
        // response in the residual, not the averaged L/B operator.
        // Each iterate is a counterfactual pose from the original endpoint.
        const std::string defectKey = "geometryDefectCorrection." + handoffKey;
        fluidField2 defectGeometry = before;
        defectGeometry.setPrePressureSnapshotObserver( {} );
        setStageObserver( defectGeometry, {} );
        const auto evaluateDefect = [&]( const std::vector<double>& pressure )
        {
            return before.evaluateIntervalPressureDefect( defectGeometry, pressureOperator,
                pressure, baselineFeedback, targetIndex );
        };
        // Solve P A P dp = -P(volume defect)/dt^2 with the unchanged
        // positive operator. The real endpoint derivative is only
        // approximated by the averaged B, so verify the actual new defect.
        std::vector<double> correctedPressure = p;
        auto current = evaluateDefect( correctedPressure );
        if( !current.analytic || current.unsupported != 0.0 )
            throw std::runtime_error( defectKey + " baseline trial lacks interval graph support" );
        double initialGap = current.rawL1;
        bool stalled = false;
        int correctionIterations = 0, defectEvaluations = 1;
        for( int pass = 0; pass <= correctionBudget; ++pass )
        {
            int steps = 0, trials = 0;
            double acceptedStep = 0.0, relativeSquared = 0.0;
            if( pass > 0 && !stalled )
            {
                const auto solvedCorrection = fluidDetail::solveIntervalPressureCorrection(
                    current.residual, pressureOperator, dt );
                const auto& correction = solvedCorrection.pressure;
                steps = solvedCorrection.iterations;
                relativeSquared = solvedCorrection.relativeSquared;
                correctionIterations += steps;
                // Fixed deterministic backtracking. Reject invalid geometry
                // or any step which fails to decrease both raw L1 and
                // projected squared defect. Do not discard unsupported rows.
                for( double alpha : { 1.0, 0.5, 0.25, 0.125 } )
                {
                    ++trials;
                    auto trialPressure = correctedPressure;
                    for( std::size_t i = 0; i < n; ++i ) trialPressure[i] += alpha * correction[i];
                    pressureOperator.project( trialPressure );
                    auto trial = evaluateDefect( trialPressure );
                    ++defectEvaluations;
                    if( trial.analytic && trial.unsupported == 0.0 &&
                        trial.rawL1 < current.rawL1 && trial.squared < current.squared )
                    {
                        correctedPressure = std::move( trialPressure );
                        current = std::move( trial );
                        acceptedStep = alpha;
                        break;
                    }
                }
                stalled = acceptedStep == 0.0;
            }
            const auto k = defectKey + ".pass" + std::to_string( pass );
            const auto write = [&]( const char* name, auto value )
            {
                if( !std::isfinite( static_cast<double>( value ) ) )
                    throw std::runtime_error( k + name + " nonfinite defect output" );
                appendScalar( trace, k + name, value );
            };
            write( ".rawL1", current.rawL1 ); write( ".projectedL1", current.projectedL1 );
            write( ".projectedSquared", current.squared ); write( ".compatibilityL1", current.compatibilityL1 );
            write( ".negativeFluxVolume", current.negativeFlux ); write( ".peak", current.peak );
            write( ".target", current.target ); write( ".unsupported", current.unsupported );
            write( ".forceChange", current.forceChange ); write( ".torqueChange", current.torqueChange );
            write( ".closed", current.closed ); write( ".opened", current.opened );
            write( ".acceptedStep", acceptedStep ); write( ".pcgIterations", steps );
            write( ".trials", trials ); write( ".linearRelativeResidualSquared", relativeSquared );
            write( ".stalled", static_cast<int>( stalled ) );
            std::printf( "%s raw gap %.9g m^2 projected L1 %.9g negative volume %.9g "
                "peak %.9g m/s force delta %.9g N torque delta %.9g Nm "
                "accepted alpha %.9g PCG %d trials %d stalled %d\n",
                k.c_str(), current.rawL1, current.projectedL1, current.negativeFlux,
                current.peak, current.forceChange, current.torqueChange,
                acceptedStep, steps, trials, static_cast<int>( stalled ) );
        }
        if( current.rawL1 > initialGap )
            throw std::runtime_error( defectKey + " accepted geometric defect increased" );
        // evaluateDefect/PCG may leave force scratch from a rejected trial.
        // Recompute the accepted pressure's feedback before World replay.
        pressureOperator.transpose( correctedPressure );
        if( !continuation )
            captureWorldPressureResponse( before, baselineFeedback, force, trace,
                "worldPressureResponse." + handoffKey );
        else
        {
            auto reconstructed = fluidDetail::reconstructIntervalPressure( before.width_, before.height_, h, dt,
                graph, correctedPressure, before.previousCellFluidFraction_, before.cellFluidFraction_,
                { meanU, before.u_, before.uOpenFraction_ }, { meanV, before.v_, before.vOpenFraction_ },
                captureEndpoint );
            continuation->u = std::move( reconstructed.u );
            continuation->v = std::move( reconstructed.v );
            continuation->fluxVolume = std::move( reconstructed.fluxVolume );
            continuation->uncoveredEndpointFaces = reconstructed.uncoveredEndpointFaces;
            continuation->pressure.resize( n );
            for( std::size_t i = 0; i < n; ++i )
                continuation->pressure[i] = static_cast<float>( correctedPressure[i] );
            continuation->feedback.resize( owners );
            for( std::size_t i = 0; i < owners; ++i )
            {
                continuation->feedback[i].force = { static_cast<float>( force[i].fx ), static_cast<float>( force[i].fy ) };
                continuation->feedback[i].torque = static_cast<float>( force[i].torque );
            }
            continuation->component = parent;
            if( captureEndpoint )
            {
                continuation->meanU = meanU; continuation->meanV = meanV;
                continuation->intervalU = std::move( reconstructed.intervalU );
                continuation->intervalV = std::move( reconstructed.intervalV );
                if( correctionBudget == 0 )
                {
                    continuation->bodyVolume = bodyVolumeAudit;
                    continuation->compatibilityVolume = removedVolumeAudit;
                    continuation->linearVolumeResidual = correctedVolume;
                }
            }
            continuation->trueResidualSquared = trueSquared;
            continuation->initialResidualSquared = initial;
            continuation->seedResidualSquared = seedSquared;
            continuation->initialIterations = iterations;
            continuation->correctionIterations = correctionIterations;
            continuation->defectEvaluations = defectEvaluations;
            continuation->missedMidpointCells = static_cast<int>( missed.size() );
            continuation->selectedEndpointDepth = endpointDepth;
            continuation->initialGap = initialGap;
            continuation->finalGap = current.rawL1;
            continuation->negativeVolume = current.negativeFlux;
            continuation->intervalPeak = current.peak;
            continuation->stalled = stalled;
            // Only the optimized debug audit consumes the existing panel geometry.
            if( refineEndpointFaces && !timing )
            {
                continuation->geometricPanels = std::move( transportPanels );
                continuation->geometricWeightSum = weightSum;
            }
        }

    }

    // R4t: compare endpoint representations on the actual loaded World
    // geometry, not the predicted geometry. Do not project or commit either.
    static void captureCorrectedEndpoint( const fluidField2& field,
        const std::vector<float>& startFraction, const continuationPressure& candidate,
        float timeStep, std::string& trace, int index, bool persistent = false )
    {
        const std::string key = std::string( persistent ? "forwardPersistentTransfer.interval" :
            "forwardCorrectedEndpoint.interval" ) + std::to_string( index );
        const double h = field.cellSize_, dt = timeStep, area = h * h;
        const auto n = startFraction.size();
        std::vector<double> velocityVolume( n ), fluxVolume( n );
        double velocityTransferL1 = 0.0, fluxTransferL1 = 0.0, lostTransfer = 0.0;
        double velocityPeak = 0.0, fluxPeak = 0.0, smallestOpen = 1.0;
        int unsupported = 0, closed = 0, openFaces = 0;
        for( std::size_t i = 0; i < n; ++i )
            velocityVolume[i] = fluxVolume[i] = area * field.cellFluidFraction_[i];
        const auto face = [&]( bool isU, int x, int y )
        {
            const int f = isU ? field.uIndex( x, y ) : field.vIndex( x, y );
            const double a = isU ? candidate.meanU[f] : candidate.meanV[f];
            const double v = isU ? candidate.intervalU[f] : candidate.intervalV[f];
            const double open = isU ? field.cutCellUOpen( x, y ) : field.cutCellVOpen( x, y );
            const int left = field.cellIndex( x - ( isU ? 1 : 0 ), y - ( isU ? 0 : 1 ) );
            const int right = field.cellIndex( x, y );
            const double transfer = dt * h * a * v;
            if( open <= 0.0 )
            {
                closed += transfer != 0.0;
                lostTransfer += std::abs( transfer );
                velocityTransferL1 += std::abs( transfer );
                fluxTransferL1 += std::abs( transfer );
                return;
            }
            ++openFaces;
            smallestOpen = std::min( smallestOpen, open );
            if( a <= 0.0 )
            {
                // No invented velocity or silent endpoint fallback.
                ++unsupported;
                return;
            }
            const double fluxVelocity = a * v / open;
            const double velocityTransfer = dt * h * open * v;
            const double fluxTransfer = dt * h * open * fluxVelocity;
            if( !std::isfinite( fluxVelocity ) || !std::isfinite( velocityTransfer ) ||
                !std::isfinite( fluxTransfer ) )
                throw std::runtime_error( key + " nonfinite corrected endpoint remap" );
            velocityPeak = std::max( velocityPeak, std::abs( v ) );
            fluxPeak = std::max( fluxPeak, std::abs( fluxVelocity ) );
            velocityTransferL1 += std::abs( velocityTransfer - transfer );
            fluxTransferL1 += std::abs( fluxTransfer - transfer );
            velocityVolume[left] += velocityTransfer; velocityVolume[right] -= velocityTransfer;
            fluxVolume[left] += fluxTransfer; fluxVolume[right] -= fluxTransfer;
        };
        for( int y = 0; y < field.height_; ++y )
            for( int x = 1; x < field.width_; ++x ) face( true, x, y );
        for( int y = 1; y < field.height_; ++y )
            for( int x = 0; x < field.width_; ++x ) face( false, x, y );
        double ledgerL1 = 0.0, velocityL1 = 0.0, fluxL1 = 0.0;
        double velocityMax = 0.0, fluxMax = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double start = area * startFraction[i];
            const double ledger = area * field.cellFluidFraction_[i] - candidate.fluxVolume[i];
            const double velocity = velocityVolume[i] - start;
            const double flux = fluxVolume[i] - start;
            ledgerL1 += std::abs( ledger );
            velocityL1 += std::abs( velocity ); fluxL1 += std::abs( flux );
            velocityMax = std::max( velocityMax, std::abs( velocity ) );
            fluxMax = std::max( fluxMax, std::abs( flux ) );
        }
        const auto out = [&]( const char* suffix, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( key + suffix + " nonfinite endpoint audit" );
            appendScalar( trace, key + suffix, value );
        };
        out( ".ledgerL1", ledgerL1 );
        out( ".velocityTransferL1", velocityTransferL1 ); out( ".fluxTransferL1", fluxTransferL1 );
        out( ".velocityVolumeL1", velocityL1 ); out( ".fluxVolumeL1", fluxL1 );
        out( ".velocityVolumeMax", velocityMax ); out( ".fluxVolumeMax", fluxMax );
        out( ".velocityPeak", velocityPeak ); out( ".fluxPeak", fluxPeak );
        out( ".lostClosedTransfer", lostTransfer ); out( ".closedTransferFaces", closed );
        out( ".unsupportedOpenFaces", unsupported ); out( ".openFaces", openFaces );
        out( ".smallestOpen", smallestOpen );
        std::printf( "%s ledger gap %.9g velocity gap %.9g flux gap %.9g m^2 "
            "velocity peak %.9g flux peak %.9g m/s closed transfer %.9g unsupported faces %d\n",
            key.c_str(), ledgerL1, velocityL1, fluxL1, velocityPeak, fluxPeak, lostTransfer, unsupported );
    }

    // R4u: instantaneous endpoint continuity with the loaded World held
    // fixed. The blocked fraction times actual solid velocity is the solid
    // flux; do not substitute the previous interval's volume difference.
    static void captureInstantaneousEndpoint( const fluidField2& field,
        const continuationPressure& candidate, std::string& trace, int index,
        std::vector<float>* committedU = nullptr, std::vector<float>* committedV = nullptr,
        bool staggered = false, bool optimized = false )
    {
        if( optimized && !staggered ) throw std::runtime_error( "R4ag invalid endpoint mode" );
        if( ( committedU == nullptr ) != ( committedV == nullptr ) )
            throw std::runtime_error( "R4v incomplete endpoint output" );
        const std::string key = std::string( optimized ? "forwardOptimizedEndpoint.interval" :
            staggered ? "forwardStaggeredEndpoint.interval" :
            committedU ? "forwardPersistentEndpoint.interval" :
            "forwardInstantaneousEndpoint.interval" ) + std::to_string( index );
        const std::size_t n = field.cellFluidFraction_.size();
        const double h = field.cellSize_;
        struct endpointEdge { int first, second; double aperture, velocity, matched; bool isU; int face; };
        std::vector<endpointEdge> edges;
        std::vector<int> parent( n, -1 ), count( n );
        std::vector<double> solid( n ), copied( n ), matched( n ), diagonal( n ), sums( n );
        int active = 0, unsupported = 0, maskedOpenFaces = 0;
        for( std::size_t i = 0; i < n; ++i )
            if( field.cellFluidFraction_[i] > 0.0f )
            {
                parent[i] = static_cast<int>( i );
                ++active;
            }
        const auto root = [&]( int i )
        {
            while( parent[i] != i ) i = parent[i];
            return i;
        };
        const auto face = [&]( bool isU, int x, int y )
        {
            const int f = isU ? field.uIndex( x, y ) : field.vIndex( x, y );
            const int left = field.cellIndex( x - ( isU ? 1 : 0 ), y - ( isU ? 0 : 1 ) );
            const int right = field.cellIndex( x, y );
            const double geometricOpen = isU ? field.uOpenFraction_[f] : field.vOpenFraction_[f];
            const double solidVelocity = isU ? field.uSolidVelocity_[f] : field.vSolidVelocity_[f];
            const double blockedFlux = ( 1.0 - geometricOpen ) * solidVelocity;
            solid[left] += blockedFlux; solid[right] -= blockedFlux;
            const double a = isU ? field.cutCellUOpen( x, y ) : field.cutCellVOpen( x, y );
            maskedOpenFaces += geometricOpen > 0.0 && a == 0.0;
            if( a <= 0.0 ) return;
            const double mean = isU ? candidate.meanU[f] : candidate.meanV[f];
            if( mean <= 0.0 )
            {
                ++unsupported;
                return;
            }
            const double velocity = isU ? candidate.intervalU[f] : candidate.intervalV[f];
            const double fluxMatched = mean * velocity / a;
            if( !std::isfinite( fluxMatched ) )
                throw std::runtime_error( key + " invalid endpoint seed" );
            const int l = root( left ), r = root( right );
            if( l != r ) parent[std::max( l, r )] = std::min( l, r );
            edges.push_back( { left, right, a, velocity, fluxMatched, isU, f } );
            diagonal[left] += a; diagonal[right] += a;
            copied[left] += a * velocity; copied[right] -= a * velocity;
            matched[left] += a * fluxMatched; matched[right] -= a * fluxMatched;
        };
        for( int y = 0; y < field.height_; ++y )
            for( int x = 1; x < field.width_; ++x ) face( true, x, y );
        for( int y = 1; y < field.height_; ++y )
            for( int x = 0; x < field.width_; ++x ) face( false, x, y );
        if( active == 0 || edges.empty() ) throw std::runtime_error( key + " empty endpoint graph" );
        int regions = 0;
        double inactiveSolidL1 = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            if( parent[i] >= 0 )
            {
                parent[i] = root( static_cast<int>( i ) );
                ++count[parent[i]];
                copied[i] += solid[i]; matched[i] += solid[i];
            }
            else inactiveSolidL1 += std::abs( solid[i] ) / h;
        }
        for( int c : count ) regions += c > 0;
        const auto zeroMean = [&]( std::vector<double>& v )
        {
            std::fill( sums.begin(), sums.end(), 0.0 );
            for( std::size_t i = 0; i < n; ++i )
                if( parent[i] >= 0 ) sums[parent[i]] += v[i];
            for( std::size_t i = 0; i < n; ++i )
                if( parent[i] >= 0 ) v[i] -= sums[parent[i]] / count[parent[i]];
        };
        const auto dot = [&]( const std::vector<double>& a, const std::vector<double>& b )
        {
            double sum = 0.0;
            for( std::size_t i = 0; i < n; ++i ) sum += a[i] * b[i];
            return sum;
        };
        const auto multiply = [&]( const std::vector<double>& p, std::vector<double>& out )
        {
            std::fill( out.begin(), out.end(), 0.0 );
            for( const auto& e : edges )
            {
                const double flux = e.aperture * ( p[e.first] - p[e.second] );
                out[e.first] += flux; out[e.second] -= flux;
            }
        };
        const auto out = [&]( const std::string& suffix, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( key + suffix + " nonfinite instantaneous audit" );
            appendScalar( trace, key + suffix, value );
        };
        out( ".active", active ); out( ".regions", regions );
        out( ".unsupportedOpenFaces", unsupported ); out( ".maskedOpenFaces", maskedOpenFaces );
        out( ".inactiveSolidL1", inactiveSolidL1 );
        // Unsupported open faces are a reported failed handoff, not an
        // invented zero-velocity boundary or an accepted projection.
        const bool supported = unsupported == 0;
        out( ".supported", static_cast<int>( supported ) );
        if( committedU )
        {
            if( !supported ) throw std::runtime_error( key + " cannot commit unsupported endpoint" );
            committedU->assign( field.u_.size(), 0.0f );
            committedV->assign( field.v_.size(), 0.0f );
        }
        for( bool preserveFlux : { false, true } )
        {
            if( staggered && preserveFlux ) break;
            const auto& before = preserveFlux ? matched : copied;
            const std::string prefix = preserveFlux ? ".flux" : ".copy";
            std::vector<double> rhs( n ), p( n ), r( n ), z( n ), d( n ), ap( n );
            for( std::size_t i = 0; i < n; ++i )
                if( parent[i] >= 0 ) rhs[i] = -before[i];
            zeroMean( rhs );
            double compatibilityL1 = 0.0;
            for( std::size_t i = 0; i < n; ++i )
                if( parent[i] >= 0 ) compatibilityL1 += std::abs( rhs[i] + before[i] ) / h;
            r = rhs;
            const auto precondition = [&]()
            {
                for( std::size_t i = 0; i < n; ++i )
                    z[i] = diagonal[i] > 0.0 ? r[i] / diagonal[i] : 0.0;
                zeroMean( z );
            };
            precondition(); d = z;
            double rz = dot( r, z ), squared = dot( r, r );
            const double initial = squared;
            const double tolerance = std::max( 1.0e-24, initial * 1.0e-10 );
            int iterations = 0;
            if( supported )
                for( ; iterations < 2000 && squared > tolerance; ++iterations )
                {
                    multiply( d, ap ); zeroMean( ap );
                    const double denominator = dot( d, ap );
                    if( !std::isfinite( denominator ) || denominator <= 0.0 ||
                        !std::isfinite( rz ) || rz <= 0.0 )
                        throw std::runtime_error( key + " endpoint PCG breakdown" );
                    const double alpha = rz / denominator;
                    for( std::size_t i = 0; i < n; ++i ) { p[i] += alpha * d[i]; r[i] -= alpha * ap[i]; }
                    squared = dot( r, r );
                    if( !std::isfinite( squared ) ) throw std::runtime_error( key + " endpoint residual nonfinite" );
                    if( squared <= tolerance ) { ++iterations; break; }
                    precondition();
                    const double next = dot( r, z ), beta = next / rz;
                    for( std::size_t i = 0; i < n; ++i ) d[i] = z[i] + beta * d[i];
                    rz = next;
                }
            zeroMean( p );
            multiply( p, ap ); zeroMean( ap );
            double trueSquared = 0.0;
            for( std::size_t i = 0; i < n; ++i )
            {
                const double residual = rhs[i] - ap[i];
                trueSquared += residual * residual;
            }
            if( supported && ( !std::isfinite( trueSquared ) || trueSquared > tolerance ) )
                throw std::runtime_error( key + " endpoint true residual failed" );
            auto after = solid;
            double seedPeak = 0.0, peak = 0.0, maxCorrection = 0.0, weightedCorrection = 0.0;
            double correctionMomentumX = 0.0, correctionMomentumY = 0.0;
            double seedEnergy = 0.0, reconstructedEnergy = 0.0;
            for( const auto& e : edges )
            {
                const double seed = preserveFlux ? e.matched : e.velocity;
                const double correction = p[e.first] - p[e.second];
                // Test the velocity representation that can actually be stored.
                const float velocity = static_cast<float>( seed + correction );
                if( committedU && !preserveFlux )
                {
                    ( e.isU ? *committedU : *committedV )[e.face] = velocity;
                    // Unit density/depth, aperture-weighted MAC face proxy.
                    // It measures the reconstruction alone, not total momentum.
                    const double weight = h * h * e.aperture;
                    ( e.isU ? correctionMomentumX : correctionMomentumY ) +=
                        weight * ( static_cast<double>( velocity ) - seed );
                    seedEnergy += 0.5 * weight * seed * seed;
                    reconstructedEnergy += 0.5 * weight * static_cast<double>( velocity ) * velocity;
                }
                const double flux = e.aperture * velocity;
                after[e.first] += flux; after[e.second] -= flux;
                seedPeak = std::max( seedPeak, std::abs( seed ) );
                peak = std::max( peak, std::abs( static_cast<double>( velocity ) ) );
                maxCorrection = std::max( maxCorrection, std::abs( correction ) );
                weightedCorrection += e.aperture * correction * correction;
            }
            double beforeSquared = 0.0, afterSquared = 0.0, afterMax = 0.0;
            for( std::size_t i = 0; i < n; ++i )
                if( parent[i] >= 0 )
                {
                    beforeSquared += before[i] * before[i];
                    afterSquared += after[i] * after[i];
                    afterMax = std::max( afterMax, std::abs( after[i] ) / h );
                }
            const double beforeRms = std::sqrt( beforeSquared / active ) / h;
            const double afterRms = std::sqrt( afterSquared / active ) / h;
            out( prefix + ".beforeRms", beforeRms ); out( prefix + ".afterRms", afterRms );
            out( prefix + ".afterMax", afterMax ); out( prefix + ".compatibilityL1", compatibilityL1 );
            out( prefix + ".seedPeak", seedPeak ); out( prefix + ".peak", peak );
            out( prefix + ".maxCorrection", maxCorrection ); out( prefix + ".weightedCorrectionSquared", weightedCorrection );
            out( prefix + ".iterations", iterations ); out( prefix + ".trueResidualSquared", trueSquared );
            if( committedU && !preserveFlux )
            {
                const std::string budgetKey = std::string( optimized ? "forwardOptimizedMomentum.interval" :
                    staggered ? "forwardStaggeredMomentum.interval" :
                    "forwardPersistentMomentum.interval" ) + std::to_string( index );
                const auto emit = [&]( const char* suffix, double value )
                {
                    if( !std::isfinite( value ) )
                        throw std::runtime_error( budgetKey + " nonfinite reconstruction budget" );
                    appendScalar( trace, budgetKey + suffix, value );
                };
                emit( ".correctionMomentumX", correctionMomentumX );
                emit( ".correctionMomentumY", correctionMomentumY );
                emit( ".seedEnergy", seedEnergy );
                emit( ".reconstructedEnergy", reconstructedEnergy );
                std::printf( "%s correction momentum proxy (%.9g, %.9g) energy proxy %.9g -> %.9g\n",
                    budgetKey.c_str(), correctionMomentumX, correctionMomentumY, seedEnergy, reconstructedEnergy );
            }
            std::printf( "%s%s supported %d instantaneous RMS %.9g -> %.9g /s peak %.9g -> %.9g m/s "
                "compatibility L1 %.9g PCG %d\n", key.c_str(), prefix.c_str(), static_cast<int>( supported ),
                beforeRms, afterRms, seedPeak, peak, compatibilityL1, iterations );
        }
    }

    static constexpr int optimizedContinuationIntervals = 240;
    struct persistentTracerAudit
    {
        std::vector<double> original, repaired, adaptive, hybrid;
        double initialMass = 0.0;
        int intervals = 0;
    };
    struct forwardEndpointState
    {
        fluidField2 field;
        std::vector<fluidShapeObstacle2> poses;
        std::vector<fluidObstacleForce2> feedback;
        persistentTracerAudit tracerAudit{};
    };

    // R4x: localize the minimum supported ledger volume without changing
    // the solver, the endpoint state or the deterministic traversal.
    static void captureMinimumVolumeCell( const fluidField2& input,
        const fluidField2& endpoint, const std::vector<float>& startFraction,
        const std::vector<float>& predictedFraction, const continuationPressure& candidate,
        float timeStep, std::string& trace, int index )
    {
        const std::string key = "forwardPersistentMinimumVolume.interval" + std::to_string( index );
        const double h = input.cellSize_, dt = timeStep, area = h * h;
        const std::size_t n = startFraction.size();
        if( candidate.bodyVolume.size() != n || candidate.compatibilityVolume.size() != n ||
            candidate.linearVolumeResidual.size() != n )
            throw std::runtime_error( key + " missing zero-correction decomposition" );
        int minimum = -1, negativeCount = 0;
        double negativeSum = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double volume = candidate.fluxVolume[i];
            if( volume < 0.0 )
            {
                ++negativeCount;
                negativeSum -= volume;
                if( candidate.component[i] < 0 )
                    throw std::runtime_error( key + " negative volume outside interval support" );
            }
            if( candidate.component[i] >= 0 &&
                ( minimum < 0 || volume < candidate.fluxVolume[minimum] ) )
                minimum = static_cast<int>( i );
        }
        if( minimum < 0 || negativeSum != candidate.negativeVolume )
            throw std::runtime_error( key + " inconsistent negative-volume ledger" );
        const auto out = [&]( const std::string& suffix, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( key + suffix + " nonfinite cell diagnostic" );
            appendScalar( trace, key + suffix, value );
        };
        const int x = minimum % input.width_, y = minimum / input.width_;
        const double start = area * startFraction[minimum];
        const double predicted = area * predictedFraction[minimum];
        const double actual = area * endpoint.cellFluidFraction_[minimum];
        const double ledger = candidate.fluxVolume[minimum];
        const double body = candidate.bodyVolume[minimum];
        const double compatibility = candidate.compatibilityVolume[minimum];
        const double linear = candidate.linearVolumeResidual[minimum];
        const double identity = predicted + body - compatibility - linear - ledger;
        double inflow = 0.0, outflow = 0.0;
        const auto face = [&]( const char* name, bool isU, int fx, int fy, double sign )
        {
            const int f = isU ? input.uIndex( fx, fy ) : input.vIndex( fx, fy );
            const double startOpen = isU ? input.cutCellUOpen( fx, fy ) : input.cutCellVOpen( fx, fy );
            const double mean = isU ? candidate.meanU[f] : candidate.meanV[f];
            const double actualOpen = isU ? endpoint.cutCellUOpen( fx, fy ) : endpoint.cutCellVOpen( fx, fy );
            const double velocity = isU ? candidate.intervalU[f] : candidate.intervalV[f];
            const double transfer = sign * ( dt * h * mean * velocity );
            inflow += std::max( 0.0, transfer ); outflow += std::max( 0.0, -transfer );
            const std::string prefix = std::string( "." ) + name;
            out( prefix + ".startOpen", startOpen ); out( prefix + ".meanOpen", mean );
            out( prefix + ".actualOpen", actualOpen ); out( prefix + ".velocity", velocity );
            out( prefix + ".inwardTransfer", transfer );
            if( negativeCount > 0 )
                std::printf( "%s%s aperture %.9g / %.9g / %.9g velocity %.9g inward %.9g m^2\n",
                    key.c_str(), prefix.c_str(), startOpen, mean, actualOpen, velocity, transfer );
        };
        face( "west", true, x, y, 1.0 ); face( "east", true, x + 1, y, -1.0 );
        face( "south", false, x, y, 1.0 ); face( "north", false, x, y + 1, -1.0 );
        if( std::abs( identity ) > 1.0e-12 ||
            std::abs( start + inflow - outflow - ledger ) > 1.0e-12 )
            throw std::runtime_error( key + " local volume identity failed" );
        out( ".cell", minimum ); out( ".negativeCount", negativeCount ); out( ".negativeSum", negativeSum );
        out( ".start", start ); out( ".predicted", predicted ); out( ".actual", actual );
        out( ".ledger", ledger ); out( ".bodyResponse", body ); out( ".compatibilityRemoved", compatibility );
        out( ".linearResidual", linear ); out( ".identityError", identity );
        out( ".inflow", inflow ); out( ".outflow", outflow );
        std::printf( "%s cell (%d,%d) negative cells %d start %.9g predicted %.9g actual %.9g "
            "ledger %.9g body %.9g compatibility %.9g linear %.9g inflow %.9g outflow %.9g m^2\n",
            key.c_str(), x, y, negativeCount, start, predicted, actual, ledger, body, compatibility,
            linear, inflow, outflow );
    }

    // R4y: replay the measured closing-cell interval using its actual loaded
    // endpoint, with body response disabled. Neither comparison is committed.
    static void captureActualGeometryLedger( const fluidField2& input, const fluidField2& endpoint,
        const std::vector<fluidShapeObstacle2>& start, const std::vector<fluidShapeObstacle2>& corrected,
        const continuationPressure& coupled, std::string& trace, int index )
    {
        fluidField2 fixed = endpoint;
        fixed.previousCellFluidFraction_ = input.cellFluidFraction_;
        fixed.previousObstacles_ = start;
        fixed.frozenPressureObstacles_ = corrected;
        fixed.volumeSourceValid_ = true;
        fixed.u_ = input.u_; fixed.v_ = input.v_;
        const double h = fixed.cellSize_, area = h * h;
        const int target = fixed.cellIndex( 17, 14 );
        if( coupled.fluxVolume[target] >= 0.0 || fixed.cellFluidFraction_[target] != 0.0f )
            throw std::runtime_error( "R4y historical closing-cell defect not reproduced" );
        for( bool weighted : { false, true } )
        {
            const std::string key = std::string( "forwardActualGeometry.interval" ) + std::to_string( index ) +
                ( weighted ? ".weighted" : ".uniform" );
            continuationPressure trial;
            std::string scratch;
            captureSpaceTimeProjection( fixed, endpoint, coupled.feedback, scratch,
                key + ".projection", 16, key + ".volume", false, 16, key + ".topology",
                0.0, &trial, 0, true, nullptr, 1.0e-16, false, {}, true, weighted );
            double negative = 0.0, minimum = 0.0, gap = 0.0, unsupportedVolume = 0.0;
            double totalLedger = 0.0, totalActual = 0.0, maxForceDelta = 0.0, maxTorqueDelta = 0.0;
            int negativeCells = 0, unsupportedCells = 0;
            for( std::size_t i = 0; i < trial.fluxVolume.size(); ++i )
            {
                const double volume = trial.fluxVolume[i];
                const double actual = area * fixed.cellFluidFraction_[i];
                const double error = actual - volume;
                totalLedger += volume; totalActual += actual;
                gap += std::abs( error );
                minimum = std::min( minimum, volume );
                if( volume < 0.0 ) { negative -= volume; ++negativeCells; }
                if( trial.component[i] < 0 )
                {
                    unsupportedCells += error != 0.0;
                    unsupportedVolume += std::abs( error );
                }
            }
            for( std::size_t j = 0; j < coupled.feedback.size(); ++j )
            {
                maxForceDelta = std::max( maxForceDelta, std::max(
                    std::abs( static_cast<double>( trial.feedback[j].force.x ) - coupled.feedback[j].force.x ),
                    std::abs( static_cast<double>( trial.feedback[j].force.y ) - coupled.feedback[j].force.y ) ) );
                maxTorqueDelta = std::max( maxTorqueDelta,
                    std::abs( static_cast<double>( trial.feedback[j].torque ) - coupled.feedback[j].torque ) );
            }
            const auto out = [&]( const char* suffix, auto value )
            {
                if( !std::isfinite( static_cast<double>( value ) ) )
                    throw std::runtime_error( key + suffix + " nonfinite actual geometry audit" );
                appendScalar( trace, key + suffix, value );
            };
            out( ".targetCoupledVolume", coupled.fluxVolume[target] );
            out( ".targetActualVolume", area * fixed.cellFluidFraction_[target] );
            out( ".targetLedgerVolume", trial.fluxVolume[target] );
            out( ".targetBodyResponse", trial.bodyVolume[target] );
            out( ".targetCompatibility", trial.compatibilityVolume[target] );
            out( ".targetLinearResidual", trial.linearVolumeResidual[target] );
            out( ".negativeVolume", negative ); out( ".negativeCells", negativeCells );
            out( ".minimumVolume", minimum ); out( ".actualGap", gap );
            out( ".unsupportedCells", unsupportedCells ); out( ".unsupportedVolume", unsupportedVolume );
            out( ".uncoveredEndpointFaces", trial.uncoveredEndpointFaces );
            out( ".totalLedgerMinusActual", totalLedger - totalActual );
            out( ".intervalPeak", trial.intervalPeak ); out( ".initialIterations", trial.initialIterations );
            out( ".trueResidualSquared", trial.trueResidualSquared );
            out( ".maxForceDelta", maxForceDelta ); out( ".maxTorqueDelta", maxTorqueDelta );
            std::printf( "%s target %.9g m^2 compatibility %.9g linear %.9g negative %.9g (%d cells) "
                "actual gap %.9g unsupported %d peak %.9g PCG %d unapplied force delta %.9g torque delta %.9g\n",
                key.c_str(), trial.fluxVolume[target], trial.compatibilityVolume[target],
                trial.linearVolumeResidual[target], negative, negativeCells, gap, unsupportedCells,
                trial.intervalPeak, trial.initialIterations, maxForceDelta, maxTorqueDelta );
        }
    }

    // R4ap: signed shared-face transport, separate from pressure and MAC.
    // Repair only small closing-cell deficits using existing outward
    // transfers to cells with positive endpoint volume. No partial result
    // escapes when the deterministic greedy repair cannot satisfy its gates.
    using transportRepairFace = fluidDetail::transportRepairFace;
    using transportRepairResult = fluidDetail::transportRepairResult;
    static void checkClosingTransportRepair()
    {
        constexpr double small = 0x1p-24;
        const std::vector<double> start{ 1.0, 1.0, 1.0 };
        const std::vector<double> endpoint{ 0.0, 0.0, 3.0 };
        const std::vector<transportRepairFace> shared{{ 0, 2, 1.0 + small },
            { 2, 1, -( 1.0 + small ) }};
        const auto result = fluidDetail::repairClosingTransport( start, endpoint, shared );
        if( !result || result->repaired != 2 || result->changedFaces != 2 )
            throw std::runtime_error( "R4ap multi-closing shared recipient failed" );
        // Force a deficit to be repaired over two outgoing faces by limiting
        // one recipient's available ledger (not merely its transfer size).
        const std::vector<double> splitStart{ 1.0, 0.0, 0.0, 1.0 };
        const std::vector<double> splitEnd{ 0.0, 1.0, 1.0, 1.0 };
        const std::vector<transportRepairFace> split{{ 0, 1, 0.5 }, { 0, 2, 0.5 + small },
            { 1, 3, 0.5 - small * 0.5 }};
        const auto splitResult = fluidDetail::repairClosingTransport( splitStart, splitEnd, split );
        if( !splitResult || splitResult->repaired != 1 || splitResult->changedFaces != 2 )
            throw std::runtime_error( "R4ap split capacity failed" );
        if( fluidDetail::repairClosingTransport( start, { 0.0, 0.0, 0.0 }, shared ) ||
            fluidDetail::repairClosingTransport( start, { 1.0, 0.0, 3.0 }, shared ) ||
            fluidDetail::repairClosingTransport( start, endpoint, {{ 0, 2, 1.01 }} ) )
            throw std::runtime_error( "R4ap unsafe repair accepted" );
        const auto clean = fluidDetail::repairClosingTransport( start, endpoint, {{ 0, 2, 1.0 }, { 1, 2, 1.0 }} );
        if( !clean || clean->repaired != 0 || clean->changedFaces != 0 )
            throw std::runtime_error( "R4ap clean transport changed" );
        std::puts( "R4ap synthetic multiple/split/no-capacity/nonclosing/large/clean cases passed" );

        const std::vector<transportRepairFace> scheduleFaces{{ 0, 1, 2.0 }, { 1, 2, -4.0 }};
        const std::vector<fluidDetail::transportFaceSample> mapping{{ true, 0, 0.5 }, { false, 0, 0.5 }};
        const auto first = fluidDetail::buildTransportPanelAmounts( scheduleFaces, mapping,
            { 0.25f }, { 0.75f }, 1.0, 2.0 );
        const auto second = fluidDetail::buildTransportPanelAmounts( scheduleFaces, mapping,
            { 0.75f }, { 0.25f }, 1.0, 2.0 );
        if( first != std::vector<double>{ 0.5, -3.0 } ||
            second != std::vector<double>{ 1.5, -1.0 } )
            throw std::runtime_error( "R4bf signed sampled schedule regression" );
        const auto closed = fluidDetail::buildTransportPanelAmounts( scheduleFaces, mapping,
            { 0.0f }, { 0.0f }, 1.0, 2.0 );
        if( closed[0] != 0.0 || closed[1] != 0.0 )
            throw std::runtime_error( "R4bf closed face received transfer" );
        const auto reject = [&]( const std::vector<fluidDetail::transportFaceSample>& map,
            const std::vector<float>& u, double weight, double sum )
        {
            bool threw = false;
            try { (void)fluidDetail::buildTransportPanelAmounts( scheduleFaces, map, u, { 0.5f }, weight, sum ); }
            catch( const std::invalid_argument& ) { threw = true; }
            if( !threw ) throw std::runtime_error( "R4bf invalid sampled schedule accepted" );
        };
        reject( {}, { 0.5f }, 1.0, 2.0 );
        reject( {{ true, 1, 0.5 }, { false, 0, 0.5 }}, { 0.5f }, 1.0, 2.0 );
        reject( {{ true, 0, 0.0 }, { false, 0, 0.5 }}, { 0.5f }, 1.0, 2.0 );
        reject( mapping, { 1.5f }, 1.0, 2.0 );
        reject( mapping, { std::numeric_limits<float>::quiet_NaN() }, 1.0, 2.0 );
        reject( mapping, { 0.5f }, 0.0, 2.0 );
        reject( mapping, { 0.5f }, 3.0, 2.0 );
        std::puts( "R4bf sampled schedule regressions passed" );

        // R4bg: library interval entry chooses its own path and never
        // changes the caller's scalar/volume input, even on rejection.
        const std::vector<double> chainStart{ 1.0, 0.0, 0.0 }, chainMass{ 2.0, 0.0, 0.0 };
        const auto chainRepair = fluidDetail::repairClosingTransport( chainStart,
            { 0.5, 0.0, 0.5 }, {{ 0, 1, 0.5 }, { 1, 2, 0.5 }} );
        if( !chainRepair ) throw std::runtime_error( "R4bg chain repair missing" );
        const std::vector<fluidDetail::transportFaceSample> chainMap{{ true, 0, 1.0 }, { true, 1, 1.0 }};
        const std::vector<transportPanel> chainPanels{{ 1.0, {}, { 1.0f, 1.0f }, {} }};
        const auto chainResult = fluidDetail::consumeHybridTransport( *chainRepair, chainStart,
            chainMass, 1.0, chainMap, chainPanels, 1.0 );
        if( !chainResult || chainResult->totalPasses != 2 || chainResult->unserved != 0.0 ||
            chainResult->volume != std::vector<double>{ 0.5, 0.0, 0.5 } ||
            chainResult->mass != std::vector<double>{ 1.0, 0.0, 1.0 } )
            throw std::runtime_error( "R4bg hybrid chain regression" );
        const auto simple = fluidDetail::repairClosingTransport( chainStart, { 0.5, 0.5, 0.0 },
            {{ 0, 1, 0.5 }} );
        if( !simple ) throw std::runtime_error( "R4bg fast path repair missing" );
        const auto fast = fluidDetail::consumeHybridTransport( *simple, chainStart, chainMass,
            1.0, {}, {}, 0.0 );
        if( !fast || fast->panels != 1 || fast->totalPasses != 1 || fast->unserved != 0.0 ||
            fast->volume != std::vector<double>{ 0.5, 0.5, 0.0 } ||
            fast->mass != std::vector<double>{ 1.0, 1.0, 0.0 } )
            throw std::runtime_error( "R4bg hybrid fast path regression" );
        const std::vector<double> empty{ 0.0, 0.0 };
        const auto cycle = fluidDetail::repairClosingTransport( empty, empty,
            {{ 0, 1, 0.5 }, { 1, 0, 0.5 }} );
        if( !cycle || fluidDetail::consumeHybridTransport( *cycle, empty, empty,
            1.0, chainMap, chainPanels, 1.0 ) )
            throw std::runtime_error( "R4bg unsupported interval was accepted" );
        const auto partial = fluidDetail::consumeHybridTransport( *cycle, empty, empty,
            1.0, chainMap, chainPanels, 1.0, fluidDetail::throughflowCommitMode::allowPartial );
        if( !partial || partial->unserved != 1.0 || partial->volume != empty || partial->mass != empty )
            throw std::runtime_error( "R4bg partial interval regression" );
        auto bad = *simple;
        bad.ledger[0] = 0.75;
        bool badLedgerRejected = false;
        try { (void)fluidDetail::consumeHybridTransport( bad, chainStart, chainMass, 1.0, {}, {}, 0.0 ); }
        catch( const std::invalid_argument& ) { badLedgerRejected = true; }
        if( !badLedgerRejected ) throw std::runtime_error( "R4bg inconsistent ledger accepted" );
        std::puts( "R4bg hybrid interval regressions passed" );

        // A 2x2 MAC fixture: internal U faces precede internal V faces.
        const std::vector<double> uMean{ 0.0, 0.5, 0.0, 0.0, 1.0, 0.0 };
        const std::vector<double> vMean{ 0.0, 0.0, 0.0, 0.25, 0.0, 0.0 };
        const auto mac = fluidDetail::buildMacTransportInput( 2, 2, 2.0, 0.5,
            uMean, vMean, { 0.0, 2.0, 0.0, 0.0, -3.0, 0.0 },
            { 0.0, 0.0, 99.0, 4.0, 0.0, 0.0 } );
        if( mac.faces.size() != 3 || mac.mapping.size() != 3 ||
            mac.faces[0].first != 0 || mac.faces[0].second != 1 || mac.faces[0].amount != 1.0 ||
            mac.faces[1].first != 2 || mac.faces[1].second != 3 || mac.faces[1].amount != -3.0 ||
            mac.faces[2].first != 1 || mac.faces[2].second != 3 || mac.faces[2].amount != 1.0 ||
            !mac.mapping[0].isU || mac.mapping[0].index != 1 ||
            !mac.mapping[1].isU || mac.mapping[1].index != 4 ||
            mac.mapping[2].isU || mac.mapping[2].index != 3 )
            throw std::runtime_error( "R4bh canonical MAC mapping regression" );
        bool invalidMacRejected = false;
        try { (void)fluidDetail::buildMacTransportInput( 2, 2, 1.0, 1.0, {}, vMean, {}, vMean ); }
        catch( const std::invalid_argument& ) { invalidMacRejected = true; }
        if( !invalidMacRejected ) throw std::runtime_error( "R4bh invalid MAC extent accepted" );
        std::puts( "R4bh canonical MAC transport regressions passed" );

        // The private field entry must commit only the scalar state and
        // its statistics. Rejected transport leaves even those unchanged.
        fluidField2 trial( 2, 2, 1.0f );
        const std::vector<double> emptyField( 4, 0.0 );
        const auto fieldCycle = fluidDetail::repairClosingTransport( emptyField, emptyField,
            {{ 0, 1, 0.5 }, { 1, 0, 0.5 }} );
        if( !fieldCycle ) throw std::runtime_error( "R4bi missing field cycle" );
        trial.cellFluidFraction_.assign( 4, 0.0f );
        trial.tracerEnabled_ = true;
        trial.tracerMass_ = emptyField;
        trial.tracerStats_.beforeMass = 3.0f;
        trial.tracerStats_.afterMass = 4.0f;
        trial.tracerStats_.substeps = 7;
        const auto beforeU = trial.u_, beforeV = trial.v_, beforePressure = trial.pressure_;
        if( trial.commitIntervalTracer( emptyField, *fieldCycle, chainMap, chainPanels, 1.0 ) ||
            trial.tracerMass_ != emptyField || trial.tracerStats_.beforeMass != 3.0f ||
            trial.tracerStats_.afterMass != 4.0f || trial.tracerStats_.substeps != 7 ||
            trial.u_ != beforeU || trial.v_ != beforeV || trial.pressure_ != beforePressure )
            throw std::runtime_error( "R4bi rejected field interval modified state" );
        const std::vector<double> simpleStart{ 1.0, 0.5, 1.0, 1.0 };
        trial.cellFluidFraction_ = { 0.5f, 1.0f, 1.0f, 1.0f };
        const auto fieldRepair = fluidDetail::repairClosingTransport( simpleStart,
            { 0.5, 1.0, 1.0, 1.0 }, {{ 0, 1, 0.5 }} );
        trial.tracerMass_ = { 2.0, 0.0, 0.0, 0.0 };
        trial.tracerEnabled_ = false;
        if( !fieldRepair || trial.commitIntervalTracer( simpleStart, *fieldRepair, {}, {}, 0.0 ) ||
            trial.tracerMass_ != std::vector<double>{ 2.0, 0.0, 0.0, 0.0 } ||
            trial.tracerStats_.substeps != 7 )
            throw std::runtime_error( "R4bi disabled field interval modified state" );
        trial.tracerEnabled_ = true;
        if( !trial.commitIntervalTracer( simpleStart, *fieldRepair, {}, {}, 0.0 ) ||
            trial.tracerMass_ != std::vector<double>{ 1.0, 1.0, 0.0, 0.0 } ||
            trial.tracerStats_.beforeMass != 2.0f || trial.tracerStats_.afterMass != 2.0f ||
            trial.tracerStats_.substeps != 1 ||
            trial.u_ != beforeU || trial.v_ != beforeV || trial.pressure_ != beforePressure )
            throw std::runtime_error( "R4bi field interval commit regression" );
        std::puts( "R4bi field interval commit regressions passed" );

        // Exact two-cell Neumann Laplacian: p = (+0.5, -0.5).
        fluidDetail::intervalPressureGraph exactGraph;
        exactGraph.edges = {{ 0, 1, 1.0, 0.0, true, 1 }};
        exactGraph.diagonal = { 1.0, 1.0 };
        exactGraph.parent = { 0, 0 }; exactGraph.count = { 2, 0 };
        std::vector<double> exactSums( 2 );
        const std::vector<fluidDetail::intervalPressureResponse> exactCoefficients{
            { 1.0, 2.0, 3.0 }, { -1.0, -2.0, -3.0 } };
        const std::vector<fluidDetail::intervalPressureBody> exactBodies{{ 0.0, 0.0 }};
        std::vector<fluidDetail::intervalPressureResponse> exactForce( 1 );
        const fluidDetail::intervalPressureOperator exactOperator( exactGraph, exactCoefficients,
            exactBodies, exactForce, exactSums, 0.0, true );
        const std::vector<double> pressureRhs{ 1.0, -1.0 };
        const std::vector<std::pair<std::size_t, double>> closingRows{{ 0, 0.0 }};
        const auto cold = fluidDetail::solveIntervalPressure( pressureRhs, exactOperator,
            {}, false, 1.0, 1.0e-16, closingRows );
        const std::vector<float> pressureSeed{ 1.5f, 0.5f };
        const auto warm = fluidDetail::solveIntervalPressure( pressureRhs, exactOperator,
            pressureSeed, true, 1.0, 1.0e-16, closingRows );
        if( cold.pressure != std::vector<double>{ 0.5, -0.5 } ||
            cold.projectedProduct != pressureRhs || cold.iterations != 1 ||
            warm.pressure != cold.pressure || warm.iterations != 0 ||
            cold.trueSquared != 0.0 || warm.trueSquared != 0.0 ||
            cold.unprojectedSquared != 0.0 || warm.unprojectedSquared != 0.0 )
            throw std::runtime_error( "R4bj projected pressure regression" );
        bool breakdownRejected = false;
        auto brokenGraph = exactGraph;
        brokenGraph.edges.clear();
        const fluidDetail::intervalPressureOperator zeroOperator( brokenGraph, {}, {}, {}, exactSums, 0.0, true );
        try
        {
            (void)fluidDetail::solveIntervalPressure( pressureRhs, zeroOperator,
                {}, false, 1.0, 1.0e-16, {} );
        }
        catch( const std::runtime_error& ) { breakdownRejected = true; }
        if( !breakdownRejected ) throw std::runtime_error( "R4bj invalid operator accepted" );
        if( exactForce[0].fx != 1.0 || exactForce[0].fy != 2.0 || exactForce[0].torque != 3.0 )
            throw std::runtime_error( "R4bo fixed geometry final traction regression" );
        auto coupledGraph = exactGraph;
        const std::vector<fluidDetail::intervalPressureResponse> coupledCoefficients{
            { 1.0, 0.0, 0.0 }, { -1.0, 0.0, 0.0 } };
        const std::vector<fluidDetail::intervalPressureBody> coupledBodies{{ 1.0, 0.0 }};
        const fluidDetail::intervalPressureOperator coupledOperator( coupledGraph, coupledCoefficients,
            coupledBodies, exactForce, exactSums, 1.0, false );
        coupledOperator.addDiagonal( coupledGraph.diagonal );
        const auto coupled = fluidDetail::solveIntervalPressure( pressureRhs, coupledOperator,
            {}, false, 1.0, 1.0e-16, closingRows );
        if( coupled.pressure[0] != 0.25 || coupled.pressure[1] != -0.25 ||
            coupled.projectedProduct != pressureRhs || coupled.iterations != 1 ||
            exactForce[0].fx != 0.5 || exactForce[0].fy != 0.0 || exactForce[0].torque != 0.0 )
            throw std::runtime_error( "R4bo coupled operator regression" );
        std::puts( "R4bj/R4bo projected pressure regressions passed" );

        // Disconnected components, a singleton and an inactive cell.
        const std::vector<int> graphParent{ 0, 0, 2, 2, 4, -1 };
        const std::vector<int> graphCount{ 2, 0, 2, 0, 1, 0 };
        std::vector<double> graphPressure{ 4.0, 2.0, -1.0, 3.0, 8.0, 7.0 };
        std::vector<double> graphSums( 6, 99.0 ), graphProduct( 6, 99.0 );
        const std::vector<fluidDetail::intervalPressureEdge> graphEdges{
            { 0, 1, 0.5, 0.0, true, 1 }, { 2, 3, 2.0, 0.0, false, 2 } };
        fluidDetail::projectIntervalPressureGauge( graphPressure, graphParent, graphCount, graphSums );
        if( graphPressure != std::vector<double>{ 1.0, -1.0, -2.0, 2.0, 0.0, 7.0 } )
            throw std::runtime_error( "R4bk disconnected graph gauge regression" );
        fluidDetail::multiplyIntervalPressureGraph( graphEdges, graphPressure, graphProduct );
        if( graphProduct != std::vector<double>{ 1.0, -1.0, -8.0, 8.0, 0.0, 0.0 } )
            throw std::runtime_error( "R4bk weighted graph multiply regression" );
        fluidDetail::projectIntervalPressureGauge( graphPressure, graphParent, graphCount, graphSums );
        if( graphPressure != std::vector<double>{ 1.0, -1.0, -2.0, 2.0, 0.0, 7.0 } )
            throw std::runtime_error( "R4bk gauge scratch reuse regression" );
        std::puts( "R4bk interval pressure graph regressions passed" );

        // Two cells and two owners; the fixed owner retains traction but
        // contributes no inverse-mass/inertia response to the Schur operator.
        const std::vector<fluidDetail::intervalPressureResponse> bodyCoefficients{
            { 1.0, 2.0, 3.0 }, { 2.0, 0.0, 1.0 },
            { -1.0, 1.0, 2.0 }, { 0.0, 3.0, -2.0 } };
        const std::vector<fluidDetail::intervalPressureBody> bodyMasses{{ 0.5, 0.25 }, { 0.0, 0.0 }};
        const std::vector<double> bodyPressure{ 2.0, -1.0 };
        std::vector<fluidDetail::intervalPressureResponse> bodyForce( 2, { 9.0, 9.0, 9.0 } );
        fluidDetail::transposeIntervalPressureBodies( bodyCoefficients, bodyPressure, bodyForce );
        if( bodyForce[0].fx != 3.0 || bodyForce[0].fy != 3.0 || bodyForce[0].torque != 4.0 ||
            bodyForce[1].fx != 4.0 || bodyForce[1].fy != -3.0 || bodyForce[1].torque != 4.0 )
            throw std::runtime_error( "R4bl body traction transpose regression" );
        std::vector<double> bodyProduct{ 1.0, 2.0 }, bodyDiagonal{ 1.0, 2.0 };
        fluidDetail::addIntervalPressureSchur( bodyCoefficients, bodyMasses, bodyForce, 0.5, bodyProduct );
        fluidDetail::addIntervalPressureSchurDiagonal( bodyCoefficients, bodyMasses, 0.5, bodyDiagonal );
        if( bodyProduct != std::vector<double>{ 4.75, 3.0 } ||
            bodyDiagonal != std::vector<double>{ 3.375, 3.0 } )
            throw std::runtime_error( "R4bl Schur product/diagonal regression" );
        fluidDetail::addIntervalPressureSchur( bodyCoefficients, bodyMasses, bodyForce, 0.0, bodyProduct );
        fluidDetail::addIntervalPressureSchurDiagonal( bodyCoefficients, bodyMasses, 0.0, bodyDiagonal );
        if( bodyProduct != std::vector<double>{ 4.75, 3.0 } ||
            bodyDiagonal != std::vector<double>{ 3.375, 3.0 } )
            throw std::runtime_error( "R4bl fixed geometry response regression" );
        std::puts( "R4bl interval pressure Schur regressions passed" );

        // 3x2 MAC graph: U(1,0), then V(0,1), with an active singleton.
        // A positive mean aperture may be closed at the endpoint.
        const std::vector<double> meanCells{ 1.0, 1.0, 0.0, 0.5, 0.0, 1.0 };
        const std::vector<float> previousCells{ 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f };
        const std::vector<float> endpointCells{ 0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
        std::vector<double> graphU( 8 ), graphV( 9 );
        std::vector<float> graphUVelocity( 8 ), graphVVelocity( 9 ),
            graphUEnd( 8, 1.0f ), graphVEnd( 9, 1.0f );
        graphU[1] = 0.5; graphV[3] = 0.25;
        graphUVelocity[1] = 2.0f; graphVVelocity[3] = 4.0f;
        const auto assembled = fluidDetail::buildIntervalPressureGraph( 3, 2, 2.0, 0.5,
            meanCells, previousCells, endpointCells,
            { graphU, graphUVelocity, graphUEnd }, { graphV, graphVVelocity, graphVEnd } );
        if( assembled.active != 4 || assembled.regions != 2 || assembled.closedEndFaces != 1 ||
            assembled.sourceSum != -1.5 || assembled.edges.size() != 2 ||
            assembled.parent != std::vector<int>{ 0, 0, -1, 0, -1, 5 } ||
            assembled.count != std::vector<int>{ 3, 0, 0, 0, 0, 1 } ||
            assembled.diagonal != std::vector<double>{ 0.75, 0.5, 0.0, 0.25, 0.0, 0.0 } ||
            assembled.rawDivergence != std::vector<double>{ 0.0, -0.5, 0.0, -2.5, 0.0, 0.0 } ||
            !assembled.edges[0].isU || assembled.edges[0].index != 1 ||
            assembled.edges[0].first != 0 || assembled.edges[0].second != 1 ||
            assembled.edges[1].isU || assembled.edges[1].index != 3 ||
            assembled.edges[1].first != 0 || assembled.edges[1].second != 3 )
            throw std::runtime_error( "R4bm canonical graph assembly regression" );
        bool unsupportedRejected = false, extentRejected = false;
        graphU[2] = 0.5;
        try
        {
            (void)fluidDetail::buildIntervalPressureGraph( 3, 2, 2.0, 0.5,
                meanCells, previousCells, endpointCells,
                { graphU, graphUVelocity, graphUEnd }, { graphV, graphVVelocity, graphVEnd } );
        }
        catch( const std::runtime_error& ) { unsupportedRejected = true; }
        try
        {
            (void)fluidDetail::buildIntervalPressureGraph( 3, 2, 2.0, 0.5,
                meanCells, previousCells, endpointCells,
                { graphU, {}, graphUEnd }, { graphV, graphVVelocity, graphVEnd } );
        }
        catch( const std::invalid_argument& ) { extentRejected = true; }
        if( !unsupportedRejected || !extentRejected )
            throw std::runtime_error( "R4bm invalid graph input accepted" );
        std::puts( "R4bm interval graph assembly regressions passed" );

        // Two components: the closing cell retains its physical RHS in
        // weighted mode while the endpoint-open cells absorb compatibility.
        fluidDetail::intervalPressureGraph rhsGraph;
        rhsGraph.rawDivergence = { -4.0, -4.0, 2.0, -2.0, 0.0 };
        rhsGraph.parent = { 0, 0, 0, 3, -1 };
        rhsGraph.count = { 3, 0, 0, 1, 0 };
        std::vector<float> rhsEndpoint{ 0.0f, 0.5f, 0.5f, 1.0f, 0.0f };
        std::vector<double> rhsOutput( 5 ), rhsScratch( 5, 99.0 );
        const auto rhsWeighted = fluidDetail::buildIntervalPressureRhs( rhsGraph, rhsEndpoint,
            1.0, true, rhsOutput, rhsScratch );
        if( rhsWeighted.failedComponent != -1 || rhsWeighted.compatibilityL1 != 8.0 ||
            rhsOutput != std::vector<double>{ 4.0, 1.0, -5.0, 0.0, 0.0 } )
            throw std::runtime_error( "R4bn weighted compatibility regression" );
        const auto rhsUniform = fluidDetail::buildIntervalPressureRhs( rhsGraph, rhsEndpoint,
            1.0, false, rhsOutput, rhsScratch );
        if( rhsUniform.failedComponent != -1 || rhsUniform.compatibilityL1 != 8.0 ||
            rhsOutput != std::vector<double>{ 2.0, 2.0, -4.0, 0.0, 0.0 } )
            throw std::runtime_error( "R4bn uniform compatibility regression" );
        rhsEndpoint[1] = rhsEndpoint[2] = 0.0f;
        const auto rhsFailure = fluidDetail::buildIntervalPressureRhs( rhsGraph, rhsEndpoint,
            1.0, true, rhsOutput, rhsScratch );
        if( rhsFailure.failedComponent != 0 || rhsFailure.componentRhs != 6.0 ||
            rhsFailure.componentVolume != 0.0 )
            throw std::runtime_error( "R4bn missing endpoint capacity regression" );
        rhsGraph.rawDivergence[1] = 2.0;
        const auto rhsZeroCapacity = fluidDetail::buildIntervalPressureRhs( rhsGraph, rhsEndpoint,
            1.0, true, rhsOutput, rhsScratch );
        if( rhsZeroCapacity.failedComponent != -1 || rhsZeroCapacity.compatibilityL1 != 2.0 ||
            rhsOutput != std::vector<double>{ 4.0, -2.0, -2.0, 0.0, 0.0 } )
            throw std::runtime_error( "R4bn compatible zero capacity regression" );
        bool rhsExtentRejected = false;
        try
        {
            (void)fluidDetail::buildIntervalPressureRhs( rhsGraph, rhsEndpoint,
                1.0, true, rhsOutput, {} );
        }
        catch( const std::invalid_argument& ) { rhsExtentRejected = true; }
        if( !rhsExtentRejected ) throw std::runtime_error( "R4bn invalid RHS extent accepted" );
        std::puts( "R4bn interval pressure RHS regressions passed" );

        // One mean-open face closes at the endpoint, another face is
        // endpoint-open but has no interval support. Do not lose the first
        // face's signed transfer or overwrite either frozen face velocity.
        fluidDetail::intervalPressureGraph reconstructionGraph;
        reconstructionGraph.parent = { 0, 0, 2 };
        reconstructionGraph.edges = {{ 0, 1, 0.5, 2.0, true, 1 }};
        const std::vector<double> reconstructionPressure{ 4.0, 0.0, 0.0 },
            reconstructionMeanU{ 0.0, 0.5, 0.0, 0.0 }, reconstructionMeanV( 6 );
        const std::vector<float> reconstructionPrevious{ 1.0f, 1.0f, 1.0f },
            reconstructionU{ 9.0f, 2.0f, 7.0f, 9.0f }, reconstructionV( 6, 3.0f ),
            reconstructionOpenU( 4, 1.0f ), reconstructionOpenV( 6, 1.0f );
        std::vector<float> reconstructionEnd{ 0.0f, 1.0f, 1.0f };
        const fluidDetail::intervalPressureFaceInput reconstructionFacesU{
            reconstructionMeanU, reconstructionU, reconstructionOpenU },
            reconstructionFacesV{ reconstructionMeanV, reconstructionV, reconstructionOpenV };
        const auto closedReconstruction = fluidDetail::reconstructIntervalPressure( 3, 1, 2.0, 0.5,
            reconstructionGraph, reconstructionPressure, reconstructionPrevious, reconstructionEnd,
            reconstructionFacesU, reconstructionFacesV, true );
        if( closedReconstruction.u != reconstructionU || closedReconstruction.v != reconstructionV ||
            closedReconstruction.fluxVolume != std::vector<double>{ 2.5, 5.5, 4.0 } ||
            closedReconstruction.intervalU != std::vector<double>{ 0.0, 3.0, 0.0, 0.0 } ||
            closedReconstruction.intervalV != reconstructionMeanV || closedReconstruction.uncoveredEndpointFaces != 1 )
            throw std::runtime_error( "R4bp closed endpoint reconstruction regression" );
        reconstructionEnd[0] = 1.0f;
        const auto openReconstruction = fluidDetail::reconstructIntervalPressure( 3, 1, 2.0, 0.5,
            reconstructionGraph, reconstructionPressure, reconstructionPrevious, reconstructionEnd,
            reconstructionFacesU, reconstructionFacesV, false );
        if( openReconstruction.u != std::vector<float>{ 9.0f, 3.0f, 7.0f, 9.0f } ||
            openReconstruction.v != reconstructionV || openReconstruction.fluxVolume != closedReconstruction.fluxVolume ||
            !openReconstruction.intervalU.empty() || !openReconstruction.intervalV.empty() ||
            openReconstruction.uncoveredEndpointFaces != 1 )
            throw std::runtime_error( "R4bp supported endpoint reconstruction regression" );
        bool reconstructionRejected = false;
        try
        {
            (void)fluidDetail::reconstructIntervalPressure( 3, 1, 2.0, 0.5,
                reconstructionGraph, {}, reconstructionPrevious, reconstructionEnd,
                reconstructionFacesU, reconstructionFacesV, true );
        }
        catch( const std::invalid_argument& ) { reconstructionRejected = true; }
        if( !reconstructionRejected ) throw std::runtime_error( "R4bp invalid reconstruction extent accepted" );
        std::puts( "R4bp interval pressure reconstruction regressions passed" );

        const auto uniformQuadrature = fluidDetail::buildIntervalPressureQuadrature( 4, 0 );
        if( uniformQuadrature.weightSum != 4.0 || uniformQuadrature.panels.size() != 4 )
            throw std::runtime_error( "R4bq uniform quadrature size regression" );
        for( std::size_t i = 0; i < uniformQuadrature.panels.size(); ++i )
        {
            const auto& panel = uniformQuadrature.panels[i];
            if( panel.alpha != ( static_cast<float>( i ) + 0.5f ) / 4.0f ||
                panel.weight != 1.0 || panel.lo != static_cast<double>( i ) / 4.0 )
                throw std::runtime_error( "R4bq uniform midpoint arithmetic regression" );
        }
        const auto refinedQuadrature = fluidDetail::buildIntervalPressureQuadrature( 4, 2 );
        const std::vector<double> expectedPanelWeights{ 0.0625, 0.0625, 0.125, 0.25,
            0.25, 0.125, 0.0625, 0.0625 };
        if( refinedQuadrature.weightSum != 1.0 || refinedQuadrature.panels.size() != expectedPanelWeights.size() )
            throw std::runtime_error( "R4bq refined quadrature size regression" );
        double panelStart = 0.0;
        for( std::size_t i = 0; i < expectedPanelWeights.size(); ++i )
        {
            const auto& panel = refinedQuadrature.panels[i];
            const double weight = expectedPanelWeights[i];
            if( panel.lo != panelStart || panel.weight != weight ||
                panel.alpha != static_cast<float>( panelStart + 0.5 * weight ) )
                throw std::runtime_error( "R4bq refined quadrature panel order regression" );
            panelStart += weight;
        }
        std::vector<double> quadratureU{ 2.0, 4.0 }, quadratureV{ 1.0 },
            quadratureFraction{ 3.0, 0.0 };
        std::vector<fluidDetail::intervalPressureResponse> quadratureCoefficients{{ 4.0, -2.0, 1.0 }};
        fluidDetail::normalizeIntervalPressureQuadrature( uniformQuadrature.weightSum,
            quadratureU, quadratureV, quadratureFraction, quadratureCoefficients );
        if( quadratureU != std::vector<double>{ 0.5, 1.0 } ||
            quadratureV != std::vector<double>{ 0.25 } ||
            quadratureFraction != std::vector<double>{ 0.75, 0.0 } ||
            quadratureCoefficients[0].fx != 1.0 || quadratureCoefficients[0].fy != -0.5 ||
            quadratureCoefficients[0].torque != 0.25 )
            throw std::runtime_error( "R4bq quadrature normalization regression" );
        bool quadratureDepthRejected = false, quadratureWeightRejected = false;
        try { (void)fluidDetail::buildIntervalPressureQuadrature( 4, 17 ); }
        catch( const std::invalid_argument& ) { quadratureDepthRejected = true; }
        try
        {
            fluidDetail::normalizeIntervalPressureQuadrature( 0.0, quadratureU, quadratureV,
                quadratureFraction, quadratureCoefficients );
        }
        catch( const std::invalid_argument& ) { quadratureWeightRejected = true; }
        if( !quadratureDepthRejected || !quadratureWeightRejected )
            throw std::runtime_error( "R4bq invalid quadrature input accepted" );
        std::puts( "R4bq interval quadrature regressions passed" );

        // 2x2 MAC grid: all endpoint cells open. One midpoint cell has
        // no support and its incident apertures must be masked.
        const std::vector<float> coverageEnd{ 1.0f, 1.0f, 1.0f, 1.0f },
            coveragePrevious{ 0.0f, 1.0f, 1.0f, 1.0f },
            coverageFirst{ 0.0f, 1.0f, 1.0f, 1.0f },
            coverageSecond{ 0.0f, 1.0f, 0.0f, 1.0f },
            coverageU( 6, 1.0f ), coverageV( 6, 1.0f );
        fluidDetail::intervalPressureCoverage coverage( 2, 2,
            { coverageEnd, coverageU, coverageV }, true );
        coverage.observe( { coverageFirst, coverageU, coverageV } );
        coverage.observe( { coverageSecond, coverageU, coverageV } );
        const auto& coverageMissedFaces = coverage.missingFaces();
        if( coverage.missingCells( coveragePrevious, coverageEnd ) != std::vector<int>{ 0 } ||
            coverageMissedFaces.size() != 2 ||
            !coverageMissedFaces[0].isU || coverageMissedFaces[0].x != 1 || coverageMissedFaces[0].y != 0 ||
            coverageMissedFaces[1].isU || coverageMissedFaces[1].x != 0 || coverageMissedFaces[1].y != 1 )
            throw std::runtime_error( "R4br midpoint support regression" );
        fluidDetail::intervalPressureCoverage cellsOnly( 2, 2,
            { coverageEnd, coverageU, coverageV }, false );
        cellsOnly.observe( { coverageFirst, coverageU, coverageV } );
        if( !cellsOnly.missingFaces().empty() ||
            cellsOnly.missingCells( coveragePrevious, coverageEnd ) != std::vector<int>{ 0 } )
            throw std::runtime_error( "R4br optional face coverage regression" );
        std::vector<double> coverageMeanCells( 4 ), coverageMeanU( 6 ), coverageMeanV( 6 );
        std::vector<float> coveragePanelU( 6, 9.0f ), coveragePanelV( 6, 9.0f );
        fluidDetail::accumulateIntervalPressureGeometry( 2, 2, { coverageFirst, coverageU, coverageV },
            0.25, coverageMeanCells, coverageMeanU, coverageMeanV, coveragePanelU, coveragePanelV );
        fluidDetail::accumulateIntervalPressureGeometry( 2, 2, { coverageSecond, coverageU, coverageV },
            0.75, coverageMeanCells, coverageMeanU, coverageMeanV, coveragePanelU, coveragePanelV );
        if( coverageMeanCells != std::vector<double>{ 0.0, 1.0, 0.25, 1.0 } ||
            coverageMeanU != std::vector<double>{ 0.0, 0.0, 0.0, 0.0, 0.25, 0.0 } ||
            coverageMeanV != std::vector<double>{ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 } ||
            coveragePanelU != std::vector<float>( 6 ) ||
            coveragePanelV != std::vector<float>{ 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f } )
            throw std::runtime_error( "R4br weighted geometry accumulation regression" );
        bool coverageExtentRejected = false;
        try { coverage.observe( { coverageFirst, {}, coverageV } ); }
        catch( const std::invalid_argument& ) { coverageExtentRejected = true; }
        if( !coverageExtentRejected ) throw std::runtime_error( "R4br invalid sample extent accepted" );
        std::puts( "R4br interval geometry coverage regressions passed" );

        // A rotation crossing the signed-pi branch must follow the short
        // arc; geometry metadata and endpoint velocities remain frozen.
        std::vector<fluidShapeObstacle2> poseStart( 2 ), poseFinish( 2 );
        poseStart[0].transform.position = { 0.0f, 2.0f };
        poseStart[0].center = { 1.0f, 3.0f };
        poseStart[0].transform.rotation = rot2::FromRadians( 3.0f );
        poseFinish[0].geometry = circle2{ {}, 0.25f };
        poseFinish[0].transform.position = { 4.0f, 6.0f };
        poseFinish[0].center = { 5.0f, 7.0f };
        poseFinish[0].transform.rotation = rot2::FromRadians( -3.0f );
        poseFinish[0].linearVelocity = { 2.0f, -1.0f };
        poseFinish[0].angularVelocity = 4.0f;
        poseFinish[0].inverseMass = 0.5f;
        poseFinish[0].inverseInertia = 0.25f;
        poseStart[1].transform.rotation = poseFinish[1].transform.rotation = rot2::FromRadians( 0.0f );
        constexpr float poseDt = 0.5f, poseAlpha = 0.25f;
        const auto poseMotion = fluidDetail::prepareIntervalPressurePoseMotion( poseStart, poseFinish, poseDt );
        auto poseSample = poseFinish;
        fluidDetail::sampleIntervalPressurePoses( poseStart, poseFinish, poseMotion, poseDt, poseAlpha, poseSample );
        const float referenceAngle = deterministicAtan2( poseStart[0].transform.rotation.s, poseStart[0].transform.rotation.c );
        const float referenceEnd = deterministicAtan2( poseFinish[0].transform.rotation.s, poseFinish[0].transform.rotation.c );
        const float referenceOmega = std::remainder( referenceEnd - referenceAngle,
            2.0f * std::numbers::pi_v<float> ) / poseDt;
        const auto referenceRotation = rot2::FromRadians( referenceAngle + referenceOmega * poseDt * poseAlpha );
        if( poseMotion[0].angle != referenceAngle || poseMotion[0].omega != referenceOmega ||
            !( poseMotion[0].omega > 0.0f && poseMotion[0].omega < 1.0f ) ||
            poseSample[0].transform.position.x != 1.0f || poseSample[0].transform.position.y != 3.0f ||
            poseSample[0].center.x != 2.0f || poseSample[0].center.y != 4.0f ||
            poseSample[0].transform.rotation.s != referenceRotation.s ||
            poseSample[0].transform.rotation.c != referenceRotation.c ||
            poseSample[0].linearVelocity.x != 2.0f || poseSample[0].linearVelocity.y != -1.0f ||
            poseSample[0].angularVelocity != 4.0f || poseSample[0].inverseMass != 0.5f ||
            poseSample[0].inverseInertia != 0.25f ||
            std::get<circle2>( poseSample[0].geometry ).radius != 0.25f || poseMotion[1].omega != 0.0f )
            throw std::runtime_error( "R4bs interval pose interpolation regression" );
        bool poseExtentRejected = false, poseAlphaRejected = false;
        try { (void)fluidDetail::prepareIntervalPressurePoseMotion( poseStart, {}, poseDt ); }
        catch( const std::invalid_argument& ) { poseExtentRejected = true; }
        try
        {
            fluidDetail::sampleIntervalPressurePoses( poseStart, poseFinish, poseMotion, poseDt, 1.5f, poseSample );
        }
        catch( const std::invalid_argument& ) { poseAlphaRejected = true; }
        if( !poseExtentRejected || !poseAlphaRejected )
            throw std::runtime_error( "R4bs invalid pose history accepted" );
        std::puts( "R4bs interval pose regressions passed" );

        // A newly constructed field has no frozen interval history.
        // Reject preparation before creating any scratch geometry.
        const fluidField2 noIntervalHistory( 2, 2, 1.0f );
        bool geometryHistoryRejected = false, geometryScheduleRejected = false;
        try { (void)noIntervalHistory.prepareIntervalPressureGeometry( 4, 0, false, false ); }
        catch( const std::invalid_argument& ) { geometryHistoryRejected = true; }
        try { (void)noIntervalHistory.prepareIntervalPressureGeometry( 0, 17, true, true ); }
        catch( const std::invalid_argument& ) { geometryScheduleRejected = true; }
        if( !geometryHistoryRejected || !geometryScheduleRejected )
            throw std::runtime_error( "R4bu invalid geometry preparation accepted" );
        std::puts( "R4bu interval geometry preparation regressions passed" );

        bool linearHistoryRejected = false;
        try
        {
            (void)noIntervalHistory.solvePreparedIntervalPressure( {}, {}, {} );
        }
        catch( const std::invalid_argument& ) { linearHistoryRejected = true; }
        if( !linearHistoryRejected )
            throw std::runtime_error( "R4bv invalid frozen linear history accepted" );
        std::puts( "R4bv interval linear solve regressions passed" );

        const std::vector<double> correctionError{ -0.25, 0.25 };
        const auto exactCorrection = fluidDetail::solveIntervalPressureCorrection(
            correctionError, exactOperator, 0.5 );
        if( exactCorrection.pressure != std::vector<double>{ 0.5, -0.5 } ||
            exactCorrection.iterations != 1 || exactCorrection.relativeSquared != 0.0 )
            throw std::runtime_error( "R4bw exact correction regression" );
        const std::vector<double> compatibleCorrectionError{ 1.0, 1.0 };
        const auto zeroCorrection = fluidDetail::solveIntervalPressureCorrection(
            compatibleCorrectionError, exactOperator, 0.5 );
        if( zeroCorrection.pressure != std::vector<double>{ 0.0, 0.0 } ||
            zeroCorrection.iterations != 0 || zeroCorrection.relativeSquared != 0.0 )
            throw std::runtime_error( "R4bw projected zero correction regression" );
        bool correctionBreakdownRejected = false, correctionExtentRejected = false;
        try { (void)fluidDetail::solveIntervalPressureCorrection( correctionError, zeroOperator, 0.5 ); }
        catch( const std::runtime_error& ) { correctionBreakdownRejected = true; }
        try { (void)fluidDetail::solveIntervalPressureCorrection( {}, exactOperator, 0.5 ); }
        catch( const std::invalid_argument& ) { correctionExtentRejected = true; }
        if( !correctionBreakdownRejected || !correctionExtentRejected )
            throw std::runtime_error( "R4bw invalid correction accepted" );
        std::puts( "R4bw interval defect correction regressions passed" );

        fluidField2 defectScratch = noIntervalHistory;
        bool defectHistoryRejected = false, defectAliasRejected = false;
        try
        {
            (void)noIntervalHistory.evaluateIntervalPressureDefect( defectScratch, exactOperator,
                exactCorrection.pressure, {}, -1 );
        }
        catch( const std::invalid_argument& ) { defectHistoryRejected = true; }
        try
        {
            (void)defectScratch.evaluateIntervalPressureDefect( defectScratch, exactOperator,
                exactCorrection.pressure, {}, -1 );
        }
        catch( const std::invalid_argument& ) { defectAliasRejected = true; }
        if( !defectHistoryRejected || !defectAliasRejected )
            throw std::runtime_error( "R4bx invalid defect evaluation accepted" );
        std::puts( "R4bx interval defect evaluation regressions passed" );
    }
    // R4aw: reuse exactly the matched R4av scalar update for a carried
    // adaptive history. Pressure, geometry and signed transfers are fixed.
    using tracerPanelResult = fluidDetail::tracerPanelResult;
    // R4ax: compare the same repaired signed transfers redistributed
    // according to their sampled aperture-time weights, with actual
    // geometric panel-start capacity instead of linear model capacity.
    // R4ba: consume the original signed transfer schedule in bounded
    // simultaneous passes. Incoming volume/mass becomes available only
    // in the next pass; no face is executed beyond its requested amount.
    static void auditThroughflowSynthetic()
    {
        const std::vector<transportRepairFace> chain{{ 0, 1, 0.5 }, { 1, 2, 0.5 }};
        const std::vector<double> requested{ 0.5, 0.5 };
        std::vector<double> volume{ 1.0, 0.0, 0.0 }, uniform = volume;
        std::vector<double> mass{ 2.0, 0.0, 0.0 };
        const auto chainResult = fluidDetail::consumeThroughflowPanel( chain, requested, volume, mass, uniform );
        if( !chainResult.committed || chainResult.passes != 2 || chainResult.unserved != 0.0 ||
            volume != std::vector<double>{ 0.5, 0.0, 0.5 } || uniform != volume ||
            mass != std::vector<double>{ 1.0, 0.0, 1.0 } )
            throw std::runtime_error( "R4ba zero-start donor chain regression" );
        const std::vector<transportRepairFace> cycle{{ 0, 1, 0.5 }, { 1, 0, 0.5 }};
        volume = { 0.0, 0.0 }; uniform = volume; mass = volume;
        const auto cycleResult = fluidDetail::consumeThroughflowPanel( cycle, requested, volume, mass, uniform );
        if( cycleResult.committed || cycleResult.passes != 1 || cycleResult.unserved != 1.0 ||
            volume != std::vector<double>{ 0.0, 0.0 } || uniform != volume || mass != volume )
            throw std::runtime_error( "R4ba unsupported empty cycle regression" );

        const std::vector<transportRepairFace> single{{ 0, 1, 2.0 }};
        const std::vector<double> excessive{ 2.0 };
        volume = { 1.0, 0.0 }; uniform = volume; mass = { 3.0, 0.0 };
        const auto rejected = fluidDetail::consumeThroughflowPanel( single, excessive, volume, mass, uniform );
        if( rejected.committed || rejected.unserved != 1.0 ||
            volume != std::vector<double>{ 1.0, 0.0 } || uniform != volume ||
            mass != std::vector<double>{ 3.0, 0.0 } )
            throw std::runtime_error( "R4be incomplete transaction changed state" );
        const auto partial = fluidDetail::consumeThroughflowPanel( single, excessive, volume, mass, uniform,
            fluidDetail::throughflowCommitMode::allowPartial );
        if( !partial.committed || partial.unserved != 1.0 ||
            volume != std::vector<double>{ 0.0, 1.0 } || uniform != volume ||
            mass != std::vector<double>{ 0.0, 3.0 } )
            throw std::runtime_error( "R4be explicit partial transaction differs" );

        const auto rejectInvalid = [&]( const std::vector<transportRepairFace>& faces,
            const std::vector<double>& amounts, std::vector<double> v,
            std::vector<double> m, std::vector<double> u )
        {
            const auto originalV = v, originalM = m, originalU = u;
            const auto identical = []( const std::vector<double>& a, const std::vector<double>& b )
            {
                if( a.size() != b.size() ) return false;
                for( std::size_t i = 0; i < a.size(); ++i )
                    if( std::bit_cast<std::uint64_t>( a[i] ) != std::bit_cast<std::uint64_t>( b[i] ) )
                        return false;
                return true;
            };
            bool threw = false;
            try { (void)fluidDetail::consumeThroughflowPanel( faces, amounts, v, m, u ); }
            catch( const std::exception& ) { threw = true; }
            if( !threw || !identical( v, originalV ) || !identical( m, originalM ) ||
                !identical( u, originalU ) )
                throw std::runtime_error( "R4be invalid transaction was not atomic" );
        };
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double largest = std::numeric_limits<double>::max();
        rejectInvalid( single, {}, { 1.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 0.0 } );
        rejectInvalid( single, { 0.5 }, { 1.0, 0.0 }, { 1.0 }, { 1.0, 0.0 } );
        rejectInvalid( {{ -1, 1, 0.0 }}, { 0.0 }, { 1.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 0.0 } );
        rejectInvalid( {{ 0, 2, 0.0 }}, { 0.0 }, { 1.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 0.0 } );
        rejectInvalid( {{ 0, 0, 0.0 }}, { 0.0 }, { 1.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 0.0 } );
        rejectInvalid( single, { nan }, { 1.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 0.0 } );
        rejectInvalid( {}, {}, { 1.0, 0.0 }, { nan, 0.0 }, { 1.0, 0.0 } );
        rejectInvalid( {}, {}, { -1.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 0.0 } );
        rejectInvalid( single, { largest }, { largest, largest }, { 1.0, 0.0 }, { 1.0, 0.0 } );
        rejectInvalid( {{ 0, 1, largest }, { 0, 2, largest }}, { largest, largest },
            { 1.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 } );
        volume = { 1.0, 0.0 }; mass = volume;
        bool aliasRejected = false;
        try { (void)fluidDetail::consumeThroughflowPanel( single, excessive, volume, mass, volume ); }
        catch( const std::invalid_argument& ) { aliasRejected = true; }
        if( !aliasRejected || volume != std::vector<double>{ 1.0, 0.0 } || mass != volume )
            throw std::runtime_error( "R4be aliased output accepted" );
        std::puts( "R4be transactional transport regressions passed" );
    }

    static void auditGeometricTracerPanels( const fluidField2& field,
        const continuationPressure& candidate, const transportRepairResult& repair,
        const std::vector<fluidDetail::transportFaceSample>& mapping,
        const std::vector<double>& seed, const tracerPanelResult& equalPanels, int index,
        std::string& trace )
    {
        const std::size_t n = seed.size();
        const double h = field.cellSize_, area = h * h;
        if( candidate.geometricPanels.empty() || candidate.geometricWeightSum <= 0.0 )
            throw std::runtime_error( "R4ax missing sampled panel geometry" );
        if( mapping.size() != repair.faces.size() )
            throw std::runtime_error( "R4bh geometric mapping size differs" );
        auto mass = seed;
        // R4ay: a unit-concentration scalar must reproduce geometric
        // volume if every panel obeys discrete geometric conservation
        // and donor requests are supported. Audit those conditions separately.
        std::vector<double> uniform( n ), ledger( n ), panelVolume( n ), incoming( n );
        for( std::size_t i = 0; i < n; ++i )
            uniform[i] = ledger[i] = area * candidate.geometricPanels.front().startFraction[i];
        // R4az: matched alternative consumes the same face schedule using
        // the carried modeled volume, not a reset to geometric capacity.
        auto modeledMass = seed, modeledUniform = uniform;
        auto throughVolume = ledger, throughMass = seed, throughUniform = uniform;
        int throughMaxPasses = 0, throughTotalPasses = 0;
        double throughUnserved = 0.0, throughMaxUniformError = 0.0;
        if( index == 113 ) auditThroughflowSynthetic();
        std::vector<double> modeledCapacity( n );
        double modeledUnserved = 0.0, modeledZeroCapacityOutgoing = 0.0;
        double maxModeledUniformError = 0.0, maxModeledGeometryError = 0.0;
        int modeledLimited = 0;
        double uniformInitialMass = 0.0;
        for( double value : uniform ) uniformInitialMass += value;
        double panelGeometryL1 = 0.0, maxPanelGeometryError = 0.0;
        double maxIntermediateNegative = 0.0, maxUniformGeometryError = 0.0;
        double maxUniformConcentration = 0.0;
        std::vector<double> sums( mapping.size() ), amounts( mapping.size() ), outgoing( n );
        double unserved = 0.0, zeroCapacityOutgoing = 0.0;
        int limitedDonors = 0;
        for( std::size_t step = 0; step < candidate.geometricPanels.size(); ++step )
        {
            const auto& panel = candidate.geometricPanels[step];
            const auto& endFraction = step + 1 < candidate.geometricPanels.size() ?
                candidate.geometricPanels[step + 1].startFraction : field.cellFluidFraction_;
            if( panel.startFraction.size() != n || panel.weight <= 0.0 )
                throw std::runtime_error( "R4ax invalid sampled panel" );
            std::fill( outgoing.begin(), outgoing.end(), 0.0 );
            std::fill( incoming.begin(), incoming.end(), 0.0 );
            modeledCapacity = ledger;
            for( std::size_t i = 0; i < n; ++i ) panelVolume[i] = area * panel.startFraction[i];
            amounts = fluidDetail::buildTransportPanelAmounts( repair.faces, mapping,
                panel.uOpen, panel.vOpen, panel.weight, candidate.geometricWeightSum );
            for( std::size_t j = 0; j < mapping.size(); ++j )
            {
                const auto& edge = repair.faces[j];
                const double amount = amounts[j];
                sums[j] += amount;
                outgoing[amount >= 0.0 ? edge.first : edge.second] += std::abs( amount );
                incoming[amount >= 0.0 ? edge.second : edge.first] += std::abs( amount );
                panelVolume[edge.first] -= amount; panelVolume[edge.second] += amount;
                ledger[edge.first] -= amount; ledger[edge.second] += amount;
            }
            const auto through = fluidDetail::consumeThroughflowPanel( repair.faces, amounts,
                throughVolume, throughMass, throughUniform, fluidDetail::throughflowCommitMode::allowPartial );
            throughMaxPasses = std::max( throughMaxPasses, through.passes );
            throughTotalPasses += through.passes;
            throughUnserved += through.unserved;
            for( std::size_t i = 0; i < n; ++i )
                throughMaxUniformError = std::max( throughMaxUniformError,
                    std::abs( throughUniform[i] - throughVolume[i] ) );
            std::vector<double> remaining = mass, received( n );
            std::vector<double> uniformRemaining = uniform, uniformReceived( n );
            std::vector<double> modeledRemaining = modeledMass, modeledReceived( n );
            std::vector<double> modeledUniformRemaining = modeledUniform, modeledUniformReceived( n );
            for( std::size_t i = 0; i < n; ++i )
            {
                const double capacity = area * panel.startFraction[i];
                if( capacity < 0.0 || capacity > area || !std::isfinite( capacity ) )
                    throw std::runtime_error( "R4ax invalid geometric capacity" );
                if( !std::isfinite( modeledCapacity[i] ) || modeledCapacity[i] < 0.0 )
                    throw std::runtime_error( "R4az negative or invalid carried start volume" );
                if( outgoing[i] > modeledCapacity[i] )
                {
                    ++modeledLimited;
                    modeledUnserved += outgoing[i] - modeledCapacity[i];
                    if( modeledCapacity[i] == 0.0 ) modeledZeroCapacityOutgoing += outgoing[i];
                }
                if( outgoing[i] > capacity )
                {
                    ++limitedDonors;
                    unserved += outgoing[i] - capacity;
                    if( capacity == 0.0 ) zeroCapacityOutgoing += outgoing[i];
                }
            }
            for( std::size_t j = 0; j < mapping.size(); ++j )
            {
                const auto& edge = repair.faces[j];
                const double amount = amounts[j];
                if( amount == 0.0 ) continue;
                const int donor = amount > 0.0 ? edge.first : edge.second;
                const int recipient = amount > 0.0 ? edge.second : edge.first;
                const double modelCapacity = modeledCapacity[donor];
                if( modelCapacity > 0.0 )
                {
                    const double ratio = std::abs( amount ) /
                        std::max( modelCapacity, outgoing[donor] );
                    const double movedModel = std::min( modeledRemaining[donor], modeledMass[donor] * ratio );
                    modeledRemaining[donor] -= movedModel; modeledReceived[recipient] += movedModel;
                    const double movedUniform = std::min( modeledUniformRemaining[donor],
                        modeledUniform[donor] * ratio );
                    modeledUniformRemaining[donor] -= movedUniform;
                    modeledUniformReceived[recipient] += movedUniform;
                }
                const double capacity = area * panel.startFraction[donor];
                if( capacity <= 0.0 ) continue;
                const double request = mass[donor] * ( std::abs( amount ) /
                    std::max( capacity, outgoing[donor] ) );
                const double moved = std::min( remaining[donor], request );
                remaining[donor] -= moved; received[recipient] += moved;
                const double uniformRequest = uniform[donor] * ( std::abs( amount ) /
                    std::max( capacity, outgoing[donor] ) );
                const double uniformMoved = std::min( uniformRemaining[donor], uniformRequest );
                uniformRemaining[donor] -= uniformMoved;
                uniformReceived[recipient] += uniformMoved;
            }
            for( std::size_t i = 0; i < n; ++i )
            {
                mass[i] = remaining[i] + received[i];
                if( !std::isfinite( mass[i] ) || mass[i] < 0.0 )
                    throw std::runtime_error( "R4ax invalid geometric scalar mass" );
                uniform[i] = uniformRemaining[i] + uniformReceived[i];
                const double endVolume = area * endFraction[i];
                modeledMass[i] = modeledRemaining[i] + modeledReceived[i];
                modeledUniform[i] = modeledUniformRemaining[i] + modeledUniformReceived[i];
                if( !std::isfinite( modeledMass[i] ) || modeledMass[i] < 0.0 ||
                    !std::isfinite( modeledUniform[i] ) || modeledUniform[i] < 0.0 ||
                    !std::isfinite( ledger[i] ) || ledger[i] < 0.0 )
                    throw std::runtime_error( "R4az invalid modeled mass or intermediate volume" );
                maxModeledUniformError = std::max( maxModeledUniformError,
                    std::abs( modeledUniform[i] - ledger[i] ) );
                maxModeledGeometryError = std::max( maxModeledGeometryError,
                    std::abs( modeledUniform[i] - endVolume ) );
                const double localError = std::abs( panelVolume[i] - endVolume );
                panelGeometryL1 += localError;
                maxPanelGeometryError = std::max( maxPanelGeometryError, localError );
                maxIntermediateNegative = std::max( maxIntermediateNegative, -ledger[i] );
                maxUniformGeometryError = std::max( maxUniformGeometryError,
                    std::abs( uniform[i] - endVolume ) );
                if( !std::isfinite( uniform[i] ) || uniform[i] < 0.0 ||
                    !std::isfinite( ledger[i] ) || !std::isfinite( localError ) )
                    throw std::runtime_error( "R4ay invalid panel volume or uniform mass" );
                if( endVolume > 0.0 )
                    maxUniformConcentration = std::max( maxUniformConcentration, uniform[i] / endVolume );
                // A final closed cell can retain mass even when its donor
                // capacity never limited a request; measure its last-panel
                // incoming/outgoing and geometric residual without clamping.
                if( step + 1 == candidate.geometricPanels.size() && endVolume == 0.0 &&
                    ( uniform[i] > 0.0 || mass[i] > seed[i] || panel.startFraction[i] > 0.0f ) )
                    std::printf( "R4ay interval%d closingCell %zu startVolume %.17g incoming %.17g "
                        "outgoing %.17g localVolume %.17g integratedVolume %.17g uniformRemaining %.17g "
                        "scalarRemaining %.17g\n", index, i, area * panel.startFraction[i],
                        incoming[i], outgoing[i], panelVolume[i], ledger[i], uniform[i], mass[i] );
            }
        }
        double transferError = 0.0, transferL1 = 0.0;
        for( std::size_t j = 0; j < mapping.size(); ++j )
        {
            transferError += std::abs( sums[j] - repair.faces[j].amount );
            transferL1 += std::abs( repair.faces[j].amount );
        }
        if( transferError > 1.0e-12 * std::max( transferL1, 1.0e-12 ) )
            throw std::runtime_error( "R4ax sampled transfer sum differs from repaired interval" );
        double initialMass = 0.0, finalMass = 0.0, peak = 0.0, trapped = 0.0, deltaL1 = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            initialMass += seed[i]; finalMass += mass[i];
            deltaL1 += std::abs( mass[i] - equalPanels.mass[i] );
            const double volume = area * field.cellFluidFraction_[i];
            if( volume > 0.0 ) peak = std::max( peak, mass[i] / volume );
            else trapped += mass[i];
        }
        double uniformTotal = 0.0, uniformTrapped = 0.0, uniformEndL1 = 0.0, ledgerError = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double volume = area * field.cellFluidFraction_[i];
            uniformTotal += uniform[i];
            uniformEndL1 += std::abs( uniform[i] - volume );
            ledgerError += std::abs( ledger[i] - repair.ledger[i] );
            if( volume == 0.0 ) uniformTrapped += uniform[i];
        }
        double modelMassTotal = 0.0, modelUniformTotal = 0.0, modelTrapped = 0.0;
        double modelUniformTrapped = 0.0, modelUniformEndL1 = 0.0, modelPeak = 0.0, modelDeltaL1 = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double volume = area * field.cellFluidFraction_[i];
            modelMassTotal += modeledMass[i]; modelUniformTotal += modeledUniform[i];
            modelUniformEndL1 += std::abs( modeledUniform[i] - volume );
            modelDeltaL1 += std::abs( modeledMass[i] - mass[i] );
            if( volume == 0.0 )
            {
                modelTrapped += modeledMass[i]; modelUniformTrapped += modeledUniform[i];
            }
            else modelPeak = std::max( modelPeak, modeledMass[i] / volume );
        }
        double throughMassTotal = 0.0, throughUniformTotal = 0.0, throughVolumeTotal = 0.0;
        double throughTrapped = 0.0, throughUniformTrapped = 0.0, throughUniformEndL1 = 0.0;
        double throughPeak = 0.0, throughDeltaL1 = 0.0, throughModelGap = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double volume = area * field.cellFluidFraction_[i];
            throughMassTotal += throughMass[i]; throughUniformTotal += throughUniform[i];
            throughVolumeTotal += throughVolume[i];
            throughUniformEndL1 += std::abs( throughUniform[i] - volume );
            throughDeltaL1 += std::abs( throughMass[i] - modeledMass[i] );
            throughModelGap += std::abs( throughVolume[i] - ledger[i] );
            if( volume == 0.0 )
            {
                throughTrapped += throughMass[i]; throughUniformTrapped += throughUniform[i];
            }
            else throughPeak = std::max( throughPeak, throughMass[i] / volume );
        }
        const double throughMassError = throughMassTotal - initialMass;
        const double throughUniformMassError = throughUniformTotal - uniformInitialMass;
        const double throughTolerance = 1.0e-12 * std::max( 1.0, uniformInitialMass );
        if( std::abs( throughMassError ) > 1.0e-12 * std::max( 1.0, initialMass ) ||
            std::abs( throughUniformMassError ) > throughTolerance ||
            std::abs( throughVolumeTotal - uniformInitialMass ) > throughTolerance ||
            throughModelGap > 2.0 * throughUnserved + throughTolerance ||
            throughMaxUniformError > 1.0e-12 * area )
            throw std::runtime_error( "R4ba throughflow conservation or uniform identity failed" );
        const double modelMassError = modelMassTotal - initialMass;
        const double modelUniformMassError = modelUniformTotal - uniformInitialMass;
        if( std::abs( modelMassError ) > 1.0e-12 * std::max( 1.0, initialMass ) ||
            std::abs( modelUniformMassError ) > 1.0e-12 * std::max( 1.0, uniformInitialMass ) )
            throw std::runtime_error( "R4az modeled capacity mass conservation failed" );
        // With no donor limiting, a uniform scalar should reproduce the
        // integrated volume, not necessarily the independently sampled geometry.
        if( modeledLimited == 0 && maxModeledUniformError > 1.0e-12 * area )
            throw std::runtime_error( "R4az uniform scalar failed modeled volume identity" );
        const double uniformMassError = uniformTotal - uniformInitialMass;
        if( std::abs( uniformMassError ) > 1.0e-12 * std::max( 1.0, uniformInitialMass ) ||
            ledgerError > 1.0e-12 * std::max( 1.0, uniformInitialMass ) )
            throw std::runtime_error( "R4ay uniform conservation or integrated ledger audit failed" );
        const double massError = finalMass - initialMass;
        if( std::abs( massError ) > 1.0e-12 * std::max( 1.0, initialMass ) )
            throw std::runtime_error( "R4ax geometric tracer mass conservation failed" );
        const std::string key = "forwardGeometricTracer.interval" + std::to_string( index );
        const auto emit = [&]( const char* suffix, double value )
        {
            if( !std::isfinite( value ) ) throw std::runtime_error( key + suffix + " nonfinite comparison" );
            appendScalar( trace, key + suffix, value );
        };
        emit( ".panels", static_cast<double>( candidate.geometricPanels.size() ) );
        emit( ".limitedDonors", limitedDonors ); emit( ".unserved", unserved );
        emit( ".zeroCapacityOutgoing", zeroCapacityOutgoing ); emit( ".transferError", transferError );
        emit( ".massError", massError ); emit( ".peakConcentration", peak );
        emit( ".trapped", trapped ); emit( ".deltaL1", deltaL1 );
        emit( ".panelGeometryL1", panelGeometryL1 );
        emit( ".maxPanelGeometryError", maxPanelGeometryError );
        emit( ".maxIntermediateNegative", maxIntermediateNegative );
        emit( ".maxUniformGeometryError", maxUniformGeometryError );
        emit( ".maxUniformConcentration", maxUniformConcentration );
        emit( ".uniformTrapped", uniformTrapped ); emit( ".uniformEndL1", uniformEndL1 );
        emit( ".uniformMassError", uniformMassError ); emit( ".ledgerError", ledgerError );
        emit( ".modeledLimited", modeledLimited );
        emit( ".modeledUnserved", modeledUnserved );
        emit( ".modeledZeroCapacityOutgoing", modeledZeroCapacityOutgoing );
        emit( ".modeledUniformError", maxModeledUniformError );
        emit( ".modeledGeometryError", maxModeledGeometryError );
        emit( ".modeledUniformTrapped", modelUniformTrapped );
        emit( ".modeledTrapped", modelTrapped );
        emit( ".modeledUniformEndL1", modelUniformEndL1 );
        emit( ".modeledMassError", modelMassError );
        emit( ".modeledUniformMassError", modelUniformMassError );
        emit( ".modeledPeak", modelPeak ); emit( ".modeledDeltaL1", modelDeltaL1 );
        emit( ".throughMaxPasses", throughMaxPasses ); emit( ".throughTotalPasses", throughTotalPasses );
        emit( ".throughUnserved", throughUnserved ); emit( ".throughModelGap", throughModelGap );
        emit( ".throughUniformError", throughMaxUniformError );
        emit( ".throughUniformTrapped", throughUniformTrapped ); emit( ".throughTrapped", throughTrapped );
        emit( ".throughUniformEndL1", throughUniformEndL1 );
        emit( ".throughMassError", throughMassError ); emit( ".throughUniformMassError", throughUniformMassError );
        emit( ".throughPeak", throughPeak ); emit( ".throughDeltaL1", throughDeltaL1 );
        std::printf( "R4ba interval%d maxPasses %d totalPasses %d unserved %.17g modelGap %.17g "
            "uniformError %.17g uniformTrapped %.17g trapped %.17g uniformEndL1 %.17g "
            "massError %.17g uniformMassError %.17g peak %.17g deltaL1 %.17g\n", index,
            throughMaxPasses, throughTotalPasses, throughUnserved, throughModelGap, throughMaxUniformError,
            throughUniformTrapped, throughTrapped, throughUniformEndL1, throughMassError,
            throughUniformMassError, throughPeak, throughDeltaL1 );
        std::printf( "R4az interval%d limited %d unserved %.17g zeroCapacityOutgoing %.17g "
            "uniformModelError %.17g uniformGeometryError %.17g uniformTrapped %.17g "
            "trapped %.17g uniformEndL1 %.17g massError %.17g uniformMassError %.17g "
            "peak %.17g deltaL1 %.17g\n", index, modeledLimited, modeledUnserved,
            modeledZeroCapacityOutgoing, maxModeledUniformError, maxModeledGeometryError,
            modelUniformTrapped, modelTrapped, modelUniformEndL1, modelMassError,
            modelUniformMassError, modelPeak, modelDeltaL1 );
        std::printf( "R4ay interval%d panelGeometryL1 %.17g maxPanelError %.17g intermediateNegative %.17g "
            "maxUniformError %.17g maxUniformConcentration %.17g uniformTrapped %.17g "
            "uniformEndL1 %.17g uniformMassError %.17g ledgerError %.17g\n", index, panelGeometryL1,
            maxPanelGeometryError, maxIntermediateNegative, maxUniformGeometryError, maxUniformConcentration,
            uniformTrapped, uniformEndL1, uniformMassError, ledgerError );
        std::printf( "R4ax interval%d geometricPanels %zu limitedDonors %d unserved %.17g "
            "zeroCapacityOutgoing %.17g transferError %.17g massError %.17g peak %.17g "
            "trapped %.17g deltaL1 %.17g equalUnserved %.17g\n",
            index, candidate.geometricPanels.size(), limitedDonors, unserved, zeroCapacityOutgoing,
            transferError, massError, peak, trapped, deltaL1, equalPanels.unserved );
    }

    // R4bb: carry a fourth, hybrid scalar history. Use one interval
    // transfer when the donor bound holds; otherwise reuse the pressure
    // quadrature schedule with bounded throughflow. Reset modeled volume
    // to actual start geometry each interval, not scalar mass.
    using hybridTracerResult = fluidDetail::hybridTracerResult;

    static hybridTracerResult consumeHybridTracer( const fluidField2& field,
        const continuationPressure& candidate, const transportRepairResult& repair,
        const std::vector<fluidDetail::transportFaceSample>& mapping,
        const std::vector<double>& startVolume, const std::vector<double>& outgoing,
        int adaptivePanels, const std::vector<double>& seed )
    {
        const double h = field.cellSize_, area = h * h;
        // Independent engine-side path selection must agree with the
        // existing equal-panel donor bound for the one-panel case.
        auto result = fluidDetail::consumeHybridTransport( repair, startVolume, seed, area,
            mapping, candidate.geometricPanels, candidate.geometricWeightSum,
            fluidDetail::throughflowCommitMode::allowPartial );
        bool single = true;
        for( std::size_t i = 0; i < startVolume.size(); ++i )
            if( outgoing[i] > startVolume[i] ) { single = false; break; }
        if( !result || single != ( adaptivePanels == 1 ) ||
            ( single && result->panels != 1 ) )
            throw std::runtime_error( "R4bg hybrid path selection differs" );
        return std::move( *result );
    }

    static void auditHybridTracer( const fluidField2& field,
        const continuationPressure& candidate, const transportRepairResult& repair,
        const std::vector<fluidDetail::transportFaceSample>& mapping,
        const std::vector<double>& startVolume, const std::vector<double>& outgoing,
        int adaptivePanels, const tracerPanelResult& adaptiveResult, persistentTracerAudit& history,
        int index, std::string& trace )
    {
        const std::size_t n = startVolume.size();
        const double h = field.cellSize_, area = h * h;
        if( history.intervals == 0 ) history.hybrid = history.repaired;
        if( history.hybrid.size() != n )
            throw std::runtime_error( "R4bb missing hybrid tracer history" );
        auto computed = consumeHybridTracer( field, candidate, repair, mapping, startVolume, outgoing,
            adaptivePanels, history.hybrid );
        auto& [mass, volume, panels, maxPasses, totalPasses, unserved, transferError] = computed;

        // The accepted pressure candidate now reaches actual field-owned
        // scalar storage through the private complete-only entry point.
        // Preserve the surrounding research trajectory: this copy is not
        // committed to World or the live pressure/geometry state.
        fluidField2 accepted = field;
        accepted.tracerEnabled_ = true;
        accepted.tracerMass_ = history.hybrid;
        const bool committed = accepted.commitIntervalTracer( startVolume, repair, mapping,
            candidate.geometricPanels, candidate.geometricWeightSum );
        if( committed != ( unserved == 0.0 ) ||
            accepted.tracerMass_ != ( committed ? mass : history.hybrid ) ||
            accepted.u_ != field.u_ || accepted.v_ != field.v_ ||
            accepted.pressure_ != field.pressure_ ||
            accepted.cellFluidFraction_ != field.cellFluidFraction_ ||
            accepted.previousCellFluidFraction_ != field.previousCellFluidFraction_ ||
            accepted.uOpenFraction_ != field.uOpenFraction_ || accepted.vOpenFraction_ != field.vOpenFraction_ )
            throw std::runtime_error( "R4bi field interval handoff differs" );
        if( committed && accepted.tracerStats_.substeps != panels )
            throw std::runtime_error( "R4bi field panel count differs" );
        if( !committed )
            throw std::runtime_error( "R4bi historical accepted interval not fully served" );
        if( adaptivePanels == 1 && history.hybrid == history.adaptive && mass != adaptiveResult.mass )
            throw std::runtime_error( "R4bb identical fast-path input differs" );

        // R4bc: identical seed, face ledger and panel-selection decision.
        // Time only the consumers, alternating order. No clock values in
        // the deterministic trace, and no timed trial commits history.
        if( benchmarkTracer )
        {
            const auto reference = fluidDetail::consumeTracerPanels( history.hybrid, startVolume,
                repair.ledger, outgoing, repair.faces, adaptivePanels );
            for( int repeat = 0; repeat < 7; ++repeat )
            {
                double adaptiveMs = 0.0, hybridMs = 0.0;
                const auto measureAdaptive = [&]()
                {
                    const auto begin = benchmarkClock::now();
                    const auto trial = fluidDetail::consumeTracerPanels( history.hybrid, startVolume,
                        repair.ledger, outgoing, repair.faces, adaptivePanels );
                    adaptiveMs = elapsedMs( begin, benchmarkClock::now() );
                    if( trial.mass != reference.mass || trial.unserved != reference.unserved )
                        throw std::runtime_error( "R4bc adaptive timing replay differs" );
                };
                const auto measureHybrid = [&]()
                {
                    const auto begin = benchmarkClock::now();
                    const auto trial = consumeHybridTracer( field, candidate, repair, mapping, startVolume,
                        outgoing, adaptivePanels, history.hybrid );
                    hybridMs = elapsedMs( begin, benchmarkClock::now() );
                    if( trial.mass != mass || trial.volume != volume || trial.panels != panels ||
                        trial.maxPasses != maxPasses || trial.totalPasses != totalPasses ||
                        trial.unserved != unserved || trial.transferError != transferError )
                        throw std::runtime_error( "R4bc hybrid timing replay differs" );
                };
                if( repeat % 2 == 0 ) { measureAdaptive(); measureHybrid(); }
                else { measureHybrid(); measureAdaptive(); }
                std::printf( "R4bc timing interval=%d repeat=%d adaptive_panels=%d hybrid_panels=%d "
                    "hybrid_passes=%d adaptive_ms=%.9f hybrid_ms=%.9f\n", index, repeat,
                    adaptivePanels, panels, totalPasses, adaptiveMs, hybridMs );
            }
        }
        double total = 0.0, peak = 0.0, trapped = 0.0, deltaL1 = 0.0, modelGap = 0.0;
        double totalVolume = 0.0, initialVolume = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            if( !std::isfinite( mass[i] ) || mass[i] < 0.0 ||
                !std::isfinite( volume[i] ) || volume[i] < 0.0 )
                throw std::runtime_error( "R4bb invalid hybrid state" );
            total += mass[i]; totalVolume += volume[i]; initialVolume += startVolume[i];
            modelGap += std::abs( volume[i] - repair.ledger[i] );
            deltaL1 += std::abs( mass[i] - adaptiveResult.mass[i] );
            const double actual = area * field.cellFluidFraction_[i];
            if( actual > 0.0 ) peak = std::max( peak, mass[i] / actual );
            else trapped += mass[i];
        }
        const double massError = total - history.initialMass;
        const double volumeError = totalVolume - initialVolume;
        const double volumeTolerance = 1.0e-12 * std::max( 1.0, initialVolume );
        if( std::abs( massError ) > 1.0e-12 * std::max( 1.0, history.initialMass ) ||
            std::abs( volumeError ) > volumeTolerance ||
            modelGap > 2.0 * unserved + volumeTolerance )
            throw std::runtime_error( "R4bb hybrid conservation audit failed" );
        const std::string key = "forwardHybridTracer.interval" + std::to_string( index );
        const auto emit = [&]( const char* suffix, double value )
        {
            if( !std::isfinite( value ) ) throw std::runtime_error( key + suffix + " nonfinite audit" );
            appendScalar( trace, key + suffix, value );
        };
        emit( ".panels", panels ); emit( ".maxPasses", maxPasses ); emit( ".totalPasses", totalPasses );
        emit( ".unserved", unserved ); emit( ".transferError", transferError );
        emit( ".modelGap", modelGap ); emit( ".volumeError", volumeError ); emit( ".massError", massError );
        emit( ".peakConcentration", peak ); emit( ".trapped", trapped ); emit( ".deltaL1", deltaL1 );
        history.hybrid = std::move( accepted.tracerMass_ );
        if( history.intervals + 1 == optimizedContinuationIntervals )
            appendVector( trace, key + ".mass", history.hybrid );
        std::printf( "R4bb interval%d panels %d maxPasses %d totalPasses %d unserved %.17g "
            "modelGap %.17g massError %.17g peak %.17g trapped %.17g deltaL1 %.17g\n", index,
            panels, maxPasses, totalPasses, unserved, modelGap, massError, peak, trapped, deltaL1 );
    }

    static void auditAdaptiveTracerPanels( const fluidField2& field,
        const std::vector<float>& startFraction, const continuationPressure& candidate,
        const transportRepairResult& repair, const std::vector<fluidDetail::transportFaceSample>& mapping,
        const std::vector<double>& nextRepaired, persistentTracerAudit& history, int index,
        std::string& trace )
    {
        const double area = static_cast<double>( field.cellSize_ ) * field.cellSize_;
        const std::size_t n = startFraction.size();
        std::vector<double> initialVolume( n ), outgoing( n );
        for( std::size_t i = 0; i < n; ++i ) initialVolume[i] = area * startFraction[i];
        for( const auto& f : repair.faces )
            outgoing[f.amount >= 0.0 ? f.first : f.second] += std::abs( f.amount );
        if( history.intervals == 0 ) history.adaptive = history.repaired;
        if( history.adaptive.size() != n )
            throw std::runtime_error( "R4aw missing adaptive scalar history" );

        // For linearly evolving volume, minimum panel-start capacity
        // occurs at the first or last panel. Choose the smallest power of
        // two with no predicted donor shortage, within a fixed work cap.
        constexpr int maxPanels = 64;
        const auto fits = [&]( int panels )
        {
            const double alpha = static_cast<double>( panels - 1 ) / panels;
            for( std::size_t i = 0; i < n; ++i )
            {
                const double last = ( 1.0 - alpha ) * initialVolume[i] + alpha * repair.ledger[i];
                if( outgoing[i] / panels > std::min( initialVolume[i], last ) ) return false;
            }
            return true;
        };
        int panels = 1;
        while( panels < maxPanels && !fits( panels ) ) panels *= 2;
        const bool limited = !fits( panels );
        auto result = fluidDetail::consumeTracerPanels( history.adaptive, initialVolume,
            repair.ledger, outgoing, repair.faces, panels );
        if( !limited && result.unserved != 0.0 )
            throw std::runtime_error( "R4aw donor bound did not hold" );
        if( panels == 1 && history.adaptive == history.repaired && result.mass != nextRepaired )
            throw std::runtime_error( "R4aw one-panel reference differs" );
        double total = 0.0, peak = 0.0, trapped = 0.0, deltaL1 = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            total += result.mass[i];
            deltaL1 += std::abs( result.mass[i] - nextRepaired[i] );
            const double volume = area * field.cellFluidFraction_[i];
            if( volume > 0.0 ) peak = std::max( peak, result.mass[i] / volume );
            else trapped += result.mass[i];
        }
        const double massError = total - history.initialMass;
        if( std::abs( massError ) > 1.0e-12 * std::max( 1.0, history.initialMass ) )
            throw std::runtime_error( "R4aw cumulative scalar mass conservation failed" );
        const std::string key = "forwardAdaptiveTracer.interval" + std::to_string( index );
        const auto emit = [&]( const char* suffix, double value )
        {
            if( !std::isfinite( value ) ) throw std::runtime_error( key + suffix + " nonfinite audit" );
            appendScalar( trace, key + suffix, value );
        };
        emit( ".panels", panels ); emit( ".limited", limited ); emit( ".unserved", result.unserved );
        emit( ".peakConcentration", peak ); emit( ".trapped", trapped );
        emit( ".massError", massError ); emit( ".deltaL1", deltaL1 );
        if( panels > 1 )
            auditGeometricTracerPanels( field, candidate, repair, mapping, history.adaptive, result, index, trace );
        auditHybridTracer( field, candidate, repair, mapping, initialVolume, outgoing, panels,
            result, history, index, trace );
        history.adaptive = std::move( result.mass );
        if( history.intervals + 1 == optimizedContinuationIntervals )
            appendVector( trace, key + ".mass", history.adaptive );
        std::printf( "R4aw interval%d panels %d limited %d unserved %.17g peak %.17g trapped %.17g "
            "massError %.17g deltaL1 %.17g\n", index, panels, static_cast<int>( limited ),
            result.unserved, peak, trapped, massError, deltaL1 );
    }

    // R4aq: exercise the existing donor-limited tracer consumer on the
    // repaired transport ledger without feeding it back into pressure/MAC.
    static void auditRepairedTracerTransport( const fluidField2& field,
        const std::vector<float>& startFraction, const continuationPressure& candidate,
        const std::vector<transportRepairFace>& original, const transportRepairResult& repair,
        const std::vector<fluidDetail::transportFaceSample>& mapping, int index,
        persistentTracerAudit& history, std::string& trace )
    {
        const double area = static_cast<double>( field.cellSize_ ) * field.cellSize_;
        const std::size_t n = startFraction.size();
        fluidField2 copy = field;
        copy.previousCellFluidFraction_ = startFraction;
        copy.tracerMass_.resize( n );
        std::vector<double> source( n );
        for( std::size_t i = 0; i < n; ++i )
        {
            // A reproducible nonuniform concentration, with no initial mass
            // in solid cells. This is not the live tracer history.
            copy.tracerMass_[i] = area * startFraction[i] * ( 1.0 + ( i % 7 ) / 8.0 );
            source[i] = area * ( static_cast<double>( field.cellFluidFraction_[i] ) - startFraction[i] );
        }
        const auto consume = [&]( const std::vector<transportRepairFace>& faces, const char* suffix,
            const std::vector<double>* seed = nullptr )
        {
            if( seed ) copy.tracerMass_ = *seed;
            intervalTransport state{
                std::vector<double>( field.u_.size() ), std::vector<double>( field.v_.size() ),
                std::vector<double>( field.u_.size() ), std::vector<double>( field.v_.size() ) };
            std::vector<double> expected = source;
            std::size_t cursor = 0;
            const auto collect = [&]( bool isU, int x, int y )
            {
                const int f = isU ? field.uIndex( x, y ) : field.vIndex( x, y );
                const double aperture = ( isU ? candidate.meanU : candidate.meanV )[f];
                const double endpointOpen = isU ? field.cutCellUOpen( x, y ) : field.cutCellVOpen( x, y );
                if( endpointOpen > 0.0 )
                    ( isU ? state.uEndpointFluid : state.vEndpointFluid )[f] =
                        ( isU ? candidate.intervalU : candidate.intervalV )[f];
                if( aperture <= 0.0 ) return;
                const int a = field.cellIndex( x - ( isU ? 1 : 0 ), y - ( isU ? 0 : 1 ) );
                const int b = field.cellIndex( x, y );
                if( cursor >= faces.size() || faces[cursor].first != a || faces[cursor].second != b )
                    throw std::runtime_error( "R4aq transport face mapping differs" );
                const double amount = faces[cursor++].amount;
                ( isU ? state.uTransfer : state.vTransfer )[f] = amount;
                if( amount != 0.0 ) { expected[a] += amount; expected[b] -= amount; }
            };
            for( int y = 0; y < field.height_; ++y )
                for( int x = 1; x < field.width_; ++x ) collect( true, x, y );
            for( int y = 1; y < field.height_; ++y )
                for( int x = 0; x < field.width_; ++x ) collect( false, x, y );
            if( cursor != faces.size() ) throw std::runtime_error( "R4aq unused transfer face" );
            std::array<std::vector<double>, 2> mass;
            std::string scratch;
            captureIntervalTransport( copy, state, source, expected, scratch,
                "R4aq.interval" + std::to_string( index ) + suffix, &mass );
            return mass;
        };
        const auto originalMass = consume( original, ".original" );
        const auto repairedMass = consume( repair.faces, ".repaired" );
        if( repair.changedFaces == 0 && originalMass != repairedMass )
            throw std::runtime_error( "R4aq no-op transport changed tracer" );
        for( std::size_t mode = 0; mode < repairedMass.size(); ++mode )
        {
            double deltaL1 = 0.0, closedBefore = 0.0, closedAfter = 0.0;
            double modelGap = 0.0, geometryGap = 0.0;
            for( std::size_t i = 0; i < n; ++i )
            {
                const double oldMass = originalMass[mode][i], newMass = repairedMass[mode][i];
                deltaL1 += std::abs( newMass - oldMass );
                if( field.cellFluidFraction_[i] == 0.0f )
                {
                    closedBefore += oldMass; closedAfter += newMass;
                }
                if( mode == 0 )
                {
                    modelGap += std::abs( newMass - repair.ledger[i] );
                    geometryGap += std::abs( newMass - area * field.cellFluidFraction_[i] );
                }
            }
            if( !std::isfinite( deltaL1 ) || !std::isfinite( closedAfter ) )
                throw std::runtime_error( "R4aq nonfinite tracer comparison" );
            std::printf( "R4aq interval%d mode %zu deltaL1 %.17g closedBefore %.17g closedAfter %.17g "
                "uniformModelGap %.17g uniformGeometryGap %.17g\n",
                index, mode, deltaL1, closedBefore, closedAfter, modelGap, geometryGap );
        }
        // R4ar: seed once, then carry both scalar histories independently
        // through the accepted common pressure/World trajectory.
        if( history.intervals == 0 )
        {
            history.original = copy.tracerMass_;
            history.repaired = history.original;
            for( double mass : history.original ) history.initialMass += mass;
        }
        if( index != 43 + history.intervals || history.original.size() != n ||
            history.repaired.size() != n )
            throw std::runtime_error( "R4ar missing persistent tracer history" );
        auto nextOriginal = consume( original, ".persistentOriginal", &history.original )[1];
        auto nextRepaired = consume( repair.faces, ".persistentRepaired", &history.repaired )[1];
        if( repair.changedFaces == 0 && history.original == history.repaired &&
            nextOriginal != nextRepaired )
            throw std::runtime_error( "R4ar identical input transport diverged" );
        // R4as: distinguish retained mass released by reopening geometry
        // from a concentration peak caused by a small active cell.
        const std::string topologyKey = "forwardTracerReopening.interval" + std::to_string( index );
        int reopeningCells = 0;
        for( std::size_t i = 0; i < n; ++i )
            reopeningCells += startFraction[i] == 0.0f && field.cellFluidFraction_[i] > 0.0f;
        appendScalar( trace, topologyKey + ".reopeningCells", reopeningCells );
        const auto auditTopology = [&]( const std::vector<double>& previous,
            const std::vector<double>& next, const char* mode )
        {
            int peakCell = -1;
            double peak = 0.0, reopeningMass = 0.0, reopeningPeak = 0.0, retainedPeak = 0.0;
            for( std::size_t i = 0; i < n; ++i )
            {
                const double fraction = field.cellFluidFraction_[i];
                if( startFraction[i] == 0.0f && next[i] < previous[i] )
                    throw std::runtime_error( topologyKey + " closed-start tracer mass lost" );
                if( fraction <= 0.0 ) continue;
                const double volume = area * fraction;
                const double concentration = next[i] / volume;
                if( !std::isfinite( concentration ) || concentration < 0.0 )
                    throw std::runtime_error( topologyKey + " invalid concentration" );
                if( peakCell < 0 || concentration > peak )
                {
                    peakCell = static_cast<int>( i );
                    peak = concentration;
                }
                if( startFraction[i] == 0.0f )
                {
                    reopeningMass += previous[i];
                    reopeningPeak = std::max( reopeningPeak, concentration );
                    retainedPeak = std::max( retainedPeak, previous[i] / volume );
                }
            }
            if( peakCell < 0 || !std::isfinite( reopeningMass ) || !std::isfinite( retainedPeak ) )
                throw std::runtime_error( topologyKey + " invalid reopening audit" );
            const std::string key = topologyKey + mode;
            appendScalar( trace, key + ".peakCell", peakCell );
            appendScalar( trace, key + ".peakStartFraction", startFraction[peakCell] );
            appendScalar( trace, key + ".peakEndFraction", field.cellFluidFraction_[peakCell] );
            appendScalar( trace, key + ".reopeningMassBefore", reopeningMass );
            appendScalar( trace, key + ".reopeningPeakConcentration", reopeningPeak );
            appendScalar( trace, key + ".retainedPeakConcentration", retainedPeak );
            std::printf( "R4as interval%d %s reopening %d retainedMass %.17g reopeningPeak %.17g "
                "retainedPeak %.17g peakCell %d start/endFraction %.9g/%.9g peak %.17g\n",
                index, mode, reopeningCells, reopeningMass, reopeningPeak, retainedPeak,
                peakCell, startFraction[peakCell], field.cellFluidFraction_[peakCell], peak );
        };
        auditTopology( history.original, nextOriginal, ".original" );
        auditTopology( history.repaired, nextRepaired, ".repaired" );

        // R4av: matched scalar-only experiment on the concentration spike.
        // Divide the same signed interval transfers into equal panels and
        // use linearly evolving modeled volume for donor capacity. This is
        // not a reconstruction of actual time-varying geometric apertures.
        if( index == 163 )
        {
            constexpr std::size_t target = 643;
            if( target >= n || startFraction[target] <= 0.0f ||
                field.cellFluidFraction_[target] <= 0.0f )
                throw std::runtime_error( "R4av historical shrinking cell missing" );
            const std::string key = "forwardTracerPanelComparison.interval163";
            std::vector<double> initialVolume( n ), outgoing( n );
            double targetIncoming = 0.0;
            for( std::size_t i = 0; i < n; ++i )
                initialVolume[i] = area * startFraction[i];
            for( const auto& f : repair.faces )
            {
                const int donor = f.amount >= 0.0 ? f.first : f.second;
                const int recipient = f.amount >= 0.0 ? f.second : f.first;
                const double amount = std::abs( f.amount );
                outgoing[donor] += amount;
                if( static_cast<std::size_t>( recipient ) == target ) targetIncoming += amount;
            }
            const double actualVolume = area * field.cellFluidFraction_[target];
            const double startMass = history.repaired[target];
            const double startConcentration = startMass / initialVolume[target];
            double totalInitialMass = 0.0;
            for( double mass : history.repaired ) totalInitialMass += mass;
            const auto emit = [&]( const std::string& suffix, double value )
            {
                if( !std::isfinite( value ) )
                    throw std::runtime_error( key + suffix + " nonfinite comparison" );
                appendScalar( trace, key + suffix, value );
            };
            emit( ".startVolume", initialVolume[target] );
            emit( ".actualEndVolume", actualVolume );
            emit( ".modeledEndVolume", repair.ledger[target] );
            emit( ".outgoing", outgoing[target] );
            emit( ".incoming", targetIncoming );
            emit( ".startMass", startMass );
            emit( ".startConcentration", startConcentration );
            std::printf( "R4av cell643 start %.17g modeledEnd %.17g actualEnd %.17g incoming %.17g "
                "outgoing %.17g startConcentration %.17g\n", initialVolume[target], repair.ledger[target],
                actualVolume, targetIncoming, outgoing[target], startConcentration );
            for( int panels : { 1, 2, 4, 8, 16 } )
            {
                auto result = fluidDetail::consumeTracerPanels( history.repaired, initialVolume,
                    repair.ledger, outgoing, repair.faces, panels );
                const auto& mass = result.mass;
                const double unserved = result.unserved;
                if( panels == 1 && mass != nextRepaired )
                    throw std::runtime_error( "R4av one-panel reference differs" );
                double total = 0.0, peak = 0.0, trapped = 0.0, deltaL1 = 0.0;
                for( std::size_t i = 0; i < n; ++i )
                {
                    total += mass[i];
                    deltaL1 += std::abs( mass[i] - nextRepaired[i] );
                    const double volume = area * field.cellFluidFraction_[i];
                    if( volume > 0.0 ) peak = std::max( peak, mass[i] / volume );
                    else trapped += mass[i];
                }
                const double massError = total - totalInitialMass;
                if( std::abs( massError ) > 1.0e-12 * std::max( 1.0, totalInitialMass ) )
                    throw std::runtime_error( "R4av panel mass conservation failed" );
                const std::string prefix = ".panels" + std::to_string( panels );
                const double concentration = mass[target] / actualVolume;
                emit( prefix + ".targetConcentration", concentration );
                emit( prefix + ".peakConcentration", peak );
                emit( prefix + ".massError", massError );
                emit( prefix + ".trapped", trapped );
                emit( prefix + ".unserved", unserved );
                emit( prefix + ".deltaL1", deltaL1 );
                std::printf( "R4av panels %d targetConcentration %.17g peak %.17g massError %.17g "
                    "trapped %.17g unserved %.17g deltaL1 %.17g\n", panels, concentration, peak,
                    massError, trapped, unserved, deltaL1 );
            }
        }

        auditAdaptiveTracerPanels( field, startFraction, candidate, repair, mapping, nextRepaired, history, index, trace );
        history.original = std::move( nextOriginal );
        history.repaired = std::move( nextRepaired );
        ++history.intervals;
        double massOriginal = 0.0, massRepaired = 0.0, trappedOriginal = 0.0, trappedRepaired = 0.0;
        double peakOriginal = 0.0, peakRepaired = 0.0, deltaL1 = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double a = history.original[i], b = history.repaired[i];
            massOriginal += a; massRepaired += b;
            deltaL1 += std::abs( a - b );
            const double volume = area * field.cellFluidFraction_[i];
            if( volume == 0.0 ) { trappedOriginal += a; trappedRepaired += b; }
            else
            {
                peakOriginal = std::max( peakOriginal, a / volume );
                peakRepaired = std::max( peakRepaired, b / volume );
            }
        }
        const double errorOriginal = massOriginal - history.initialMass;
        const double errorRepaired = massRepaired - history.initialMass;
        if( std::abs( errorOriginal ) > 1.0e-12 * std::max( 1.0, history.initialMass ) ||
            std::abs( errorRepaired ) > 1.0e-12 * std::max( 1.0, history.initialMass ) )
            throw std::runtime_error( "R4ar cumulative tracer mass conservation failed" );
        const std::string key = "forwardPersistentTracer.interval" + std::to_string( index );
        const auto emit = [&]( const char* suffix, double value )
        {
            if( !std::isfinite( value ) ) throw std::runtime_error( key + suffix + " nonfinite tracer audit" );
            appendScalar( trace, key + suffix, value );
        };
        emit( ".deltaL1", deltaL1 ); emit( ".originalMassError", errorOriginal );
        emit( ".repairedMassError", errorRepaired ); emit( ".originalTrapped", trappedOriginal );
        emit( ".repairedTrapped", trappedRepaired ); emit( ".originalPeakConcentration", peakOriginal );
        emit( ".repairedPeakConcentration", peakRepaired );
        if( history.intervals == 64 || history.intervals == optimizedContinuationIntervals )
        {
            appendVector( trace, key + ".originalMass", history.original );
            appendVector( trace, key + ".repairedMass", history.repaired );
        }
        std::printf( "R4ar interval%d deltaL1 %.17g massErrors %.17g/%.17g trapped %.17g/%.17g "
            "peakConcentration %.17g/%.17g\n", index, deltaL1, errorOriginal, errorRepaired,
            trappedOriginal, trappedRepaired, peakOriginal, peakRepaired );
    }

    static void auditClosingTransportRepair( const fluidField2& field,
        const std::vector<float>& startFraction, const continuationPressure& candidate,
        float dt, int index, persistentTracerAudit& history, std::string& trace )
    {
        const double h = field.cellSize_, area = h * h;
        std::vector<double> start( startFraction.size() ), endpoint( start.size() );
        for( std::size_t i = 0; i < start.size(); ++i )
        {
            start[i] = area * startFraction[i];
            endpoint[i] = area * field.cellFluidFraction_[i];
        }
        const auto input = fluidDetail::buildMacTransportInput( field.width_, field.height_,
            h, static_cast<double>( dt ), candidate.meanU, candidate.meanV,
            candidate.intervalU, candidate.intervalV );
        const auto& faces = input.faces;
        if( fluidDetail::rebuildTransportLedger( start, faces ) != candidate.fluxVolume )
            throw std::runtime_error( "R4ap baseline transport ledger differs" );
        const auto result = fluidDetail::repairClosingTransport( start, endpoint, faces );
        if( !result ) throw std::runtime_error( "R4ap unsupported closing transport repair" );
        if( index == 64 && result->repaired != 1 )
            throw std::runtime_error( "R4ap historical deficit not repaired" );
        auditRepairedTracerTransport( field, startFraction, candidate, faces, *result, input.mapping, index, history, trace );
        std::printf( "R4ap interval%d repaired %d changedFaces %d reduction %.17g conservationError %.17g\n",
            index, result->repaired, result->changedFaces, result->reduction, result->conservationError );
    }

    // R4k: independent forward pressure trial at a sampled R4j start.
    // Freeze the incoming MAC trial velocity: no forward advection or
    // persistent candidate commit is implied by this geometry-only test.
    static void captureForwardPredictor( const fluidField2& input,
        const fluidField2& laggedField, const std::vector<fluidShapeObstacle2>& start,
        const std::vector<fluidObstacleForce2>& laggedLoad, std::string& trace,
        int index, double responseScale = 1.0, int correctionBudget = 3, int samples = 64,
        bool adaptiveEndpoint = false, forwardTiming* timing = nullptr,
        double relativeSquaredTolerance = 1.0e-16, bool warmStart = false,
        std::vector<float>* forwardPressureHistory = nullptr, bool auditEndpoint = false,
        forwardEndpointState* committed = nullptr, bool staggered = false, bool optimized = false )
    {
        const auto totalStart = timing ? benchmarkClock::now() : benchmarkClock::time_point{};
        using namespace sandbox;
        if( responseScale != 1.0 && responseScale != 0.75 )
            throw std::runtime_error( "R4l unsupported forward response scale" );
        if( samples != 64 && ( ( samples != 16 && samples != 32 ) ||
            responseScale != 0.75 || correctionBudget != 0 ) )
            throw std::runtime_error( "R4n unsupported quadrature budget" );
        if( adaptiveEndpoint && ( samples != 16 || responseScale != 0.75 || correctionBudget != 0 ) )
            throw std::runtime_error( "R4o invalid adaptive endpoint configuration" );
        const bool experimentalResidual = relativeSquaredTolerance != 1.0e-16;
        if( experimentalResidual && ( !adaptiveEndpoint ||
            ( relativeSquaredTolerance != 1.0e-12 && relativeSquaredTolerance != 1.0e-10 ) ) )
            throw std::runtime_error( "R4q invalid forward residual configuration" );
        if( warmStart && ( !adaptiveEndpoint || relativeSquaredTolerance != 1.0e-10 ) )
            throw std::runtime_error( "R4r invalid warm-start configuration" );
        if( forwardPressureHistory && !warmStart )
            throw std::runtime_error( "R4s pressure history requires warm start" );
        const bool usedPreviousForward = forwardPressureHistory && !forwardPressureHistory->empty();
        if( committed && ( !auditEndpoint || !warmStart || forwardPressureHistory ) )
            throw std::runtime_error( "R4v invalid endpoint commit configuration" );
        const std::string name = committed ? "forwardPersistent" :
            forwardPressureHistory ? "forwardPressureHistory" :
            warmStart ? "forwardWarmStart" : experimentalResidual ?
            ( relativeSquaredTolerance == 1.0e-12 ? "forwardResidual12" : "forwardResidual10" ) :
            adaptiveEndpoint ? "forwardAdaptiveEndpoint" :
            samples != 64 ? "forwardQuadrature" + std::to_string( samples ) :
            responseScale == 1.0 ? "forwardPredictor" :
            correctionBudget == 3 ? "forwardCadence" :
            "forwardBudget" + std::to_string( correctionBudget );
        const std::string key = name + ".interval" + std::to_string( index );
        const float dt = input.frozenPressureTimeStep_;
        const double area = static_cast<double>( input.cellSize_ ) * input.cellSize_;
        const auto run = [&]( const std::vector<fluidObstacleForce2>& loads )
        {
            rigidBodyDemo scene{ demoKind::windFlow };
            auto& world = scene.getWorld();
            if( scene.getShapes().size() != start.size() || loads.size() != start.size() )
                throw std::runtime_error( key + " invalid forward fixture" );
            auto result = start;
            for( std::size_t j = 0; j < start.size(); ++j )
            {
                const auto id = scene.getShapes()[j].bodyHandle;
                if( 1.0f / world.GetBodyMass( id ) != start[j].inverseMass ||
                    1.0f / world.GetBodyRotationalInertia( id ) != start[j].inverseInertia ||
                    world.GetBodyLinearDamping( id ) != 0.0f || world.GetBodyAngularDamping( id ) != 0.0f )
                    throw std::runtime_error( key + " forward fixture mass/damping differs" );
                world.SetBodyTransform( id, start[j].transform );
                world.SetBodyLinearVelocity( id, start[j].linearVelocity );
                world.SetBodyAngularVelocity( id, start[j].angularVelocity );
                const vec2 center = TransformPoint( world.GetBodyTransform( id ), world.GetBodyLocalCenter( id ) );
                if( center.x != start[j].center.x || center.y != start[j].center.y ||
                    !IsFinite( loads[j].force ) || !std::isfinite( loads[j].torque ) )
                    throw std::runtime_error( key + " invalid start COM or pressure load" );
                world.ApplyForceToCenter( id, loads[j].force );
                world.ApplyTorque( id, loads[j].torque );
            }
            world.Step( dt, 2 );
            for( std::size_t j = 0; j < start.size(); ++j )
            {
                const auto id = scene.getShapes()[j].bodyHandle;
                if( world.GetBody( id ).contactCount != 0 || world.GetBody( id ).jointCount != 0 )
                    throw std::runtime_error( key + " unexpected forward constraint" );
                result[j].transform = world.GetBodyTransform( id );
                result[j].center = TransformPoint( result[j].transform, world.GetBodyLocalCenter( id ) );
                result[j].linearVelocity = world.GetBodyLinearVelocity( id );
                result[j].angularVelocity = world.GetBodyAngularVelocity( id );
                if( !IsFinite( result[j].center ) || !IsFinite( result[j].linearVelocity ) ||
                    !std::isfinite( result[j].angularVelocity ) )
                    throw std::runtime_error( key + " invalid forward World result" );
            }
            return result;
        };

        if( optimized && !staggered ) throw std::runtime_error( "R4ag invalid forward mode" );
        if( staggered && ( !committed || !input.preciseDisjointCapsuleEnabled_ ) )
            throw std::runtime_error( "R4ad invalid staggered fixture" );
        const auto predicted = run( staggered ? laggedLoad : std::vector<fluidObstacleForce2>( start.size() ) );
        fluidField2 forward = input;
        forward.setPrePressureSnapshotObserver( {} );
        setStageObserver( forward, {} );
        forward.tracerEnabled_ = false;
        const auto rebuild = [&]( const std::vector<fluidShapeObstacle2>& poses )
        {
            forward.updateObstacles( poses );
            forward.updateCutCellGeometry( poses );
            if( !forward.cutCellTransition_.usedDisjointAnalyticGeometry )
                throw std::runtime_error( key + " forward geometry not analytic" );
        };
        rebuild( start );
        const auto startFraction = forward.cellFluidFraction_;
        if( startFraction != input.cellFluidFraction_ )
            throw std::runtime_error( key + " start geometry differs from frozen input" );
        rebuild( predicted );
        const auto predictedFraction = forward.cellFluidFraction_;
        forward.previousCellFluidFraction_ = startFraction;
        forward.previousObstacles_ = start;
        forward.frozenPressureObstacles_ = predicted;
        forward.volumeSourceValid_ = true;
        forward.u_ = input.u_;
        forward.v_ = input.v_;
        continuationPressure candidate;
        std::string scratch;
        if( staggered )
        {
            // R4ad: one explicit body step using the previously computed
            // pressure, followed by a fixed-geometry interval solve. The
            // newly computed load is applied only in the next interval.
            const std::string splitKey = std::string( optimized ? "forwardOptimized.interval" :
                "forwardStaggered.interval" ) + std::to_string( index );
            const auto pressureStart = timing ? benchmarkClock::now() : benchmarkClock::time_point{};
            if( timing ) timing->setupMs = elapsedMs( totalStart, pressureStart );
            captureSpaceTimeProjection( forward, laggedField, laggedLoad, scratch,
                splitKey + ".projection", 16, splitKey + ".volume", false,
                16, splitKey + ".topology", 0.0, &candidate, 0, true,
                timing, optimized ? 1.0e-10 : 1.0e-16, optimized, {}, true, true, optimized,
                0.0, optimized );
            const auto pressureEnd = timing ? benchmarkClock::now() : benchmarkClock::time_point{};
            if( timing ) timing->pressureMs = elapsedMs( pressureStart, pressureEnd );
            if( optimized && !timing )
                auditClosingTransportRepair( forward, startFraction, candidate, dt, index,
                    committed->tracerAudit, trace );
            // R4al: isolate the negative modeled volume on the exact
            // accepted optimized input. Probe strict solves without
            // committing either alternative or changing trace coverage.
            if( optimized && !timing && index == 64 )
            {
                std::string diagnostic;
                captureMinimumVolumeCell( input, forward, startFraction, predictedFraction,
                    candidate, dt, diagnostic, index );
                const auto minimum = std::min_element( candidate.fluxVolume.begin(), candidate.fluxVolume.end() );
                if( minimum == candidate.fluxVolume.end() || *minimum >= 0.0 )
                    throw std::runtime_error( splitKey + " R4al historical negative volume not reproduced" );
                const std::size_t target = static_cast<std::size_t>( minimum - candidate.fluxVolume.begin() );
                const auto report = [&]( const char* name, const continuationPressure& trial )
                {
                    if( trial.component != candidate.component || trial.meanU != candidate.meanU ||
                        trial.meanV != candidate.meanV ||
                        trial.linearVolumeResidual.size() != startFraction.size() ||
                        trial.compatibilityVolume.size() != startFraction.size() )
                        throw std::runtime_error( splitKey + " R4al matched geometry differs" );
                    int negatives = 0;
                    double negativeSum = 0.0, gap = 0.0;
                    for( std::size_t j = 0; j < trial.fluxVolume.size(); ++j )
                    {
                        const double volume = trial.fluxVolume[j];
                        if( !std::isfinite( volume ) )
                            throw std::runtime_error( splitKey + " R4al nonfinite volume" );
                        if( volume < 0.0 ) { negativeSum -= volume; ++negatives; }
                        const double error = std::abs( area * predictedFraction[j] - volume );
                        if( trial.component[j] < 0 && error != 0.0 )
                            throw std::runtime_error( splitKey + " R4al unsupported trial volume" );
                        gap += error;
                    }
                    std::printf( "R4al interval64 %s cell %zu start %.17g endpoint %.17g ledger %.17g "
                        "compatibility %.17g linear %.17g negative %.17g cells %d gap %.17g "
                        "PCG %d residualSquared %.17g\n", name, target, area * startFraction[target],
                        area * predictedFraction[target], trial.fluxVolume[target],
                        trial.compatibilityVolume[target], trial.linearVolumeResidual[target],
                        negativeSum, negatives, gap, trial.initialIterations, trial.trueResidualSquared );
                };
                report( "warmResidual10", candidate );
                // R4an: a matched, uncommitted conservative transport probe.
                // Reduce one shared outgoing face transfer rather than clamp
                // the negative cell ledger. Pressure and traction stay fixed.
                if( predictedFraction[target] != 0.0f || startFraction[target] <= 0.0f )
                    throw std::runtime_error( splitKey + " R4an expected a closing cell" );
                const double h = forward.cellSize_;
                auto trialU = candidate.intervalU, trialV = candidate.intervalV;
                struct transferFace
                {
                    bool isU = false;
                    int face = -1;
                    int neighbor = -1;
                    double outward = 0.0;
                    double factor = 0.0;
                };
                transferFace chosen;
                const int x = static_cast<int>( target ) % forward.width_;
                const int y = static_cast<int>( target ) / forward.width_;
                const auto consider = [&]( bool isU, int fx, int fy, int neighbor, double sign )
                {
                    if( neighbor < 0 || candidate.component[neighbor] != candidate.component[target] )
                        return;
                    const int face = isU ? forward.uIndex( fx, fy ) : forward.vIndex( fx, fy );
                    const double aperture = ( isU ? candidate.meanU : candidate.meanV )[face];
                    const double factor = static_cast<double>( dt ) * h * aperture;
                    const double outward = sign * factor * ( isU ? trialU : trialV )[face];
                    if( predictedFraction[neighbor] > 0.0f && outward > chosen.outward )
                        chosen = { isU, face, neighbor, outward, factor };
                };
                if( x > 0 ) consider( true, x, y, forward.cellIndex( x - 1, y ), -1.0 );
                if( x + 1 < forward.width_ ) consider( true, x + 1, y, forward.cellIndex( x + 1, y ), 1.0 );
                if( y > 0 ) consider( false, x, y, forward.cellIndex( x, y - 1 ), -1.0 );
                if( y + 1 < forward.height_ ) consider( false, x, y + 1, forward.cellIndex( x, y + 1 ), 1.0 );
                const double deficit = -*minimum;
                // Account for subtraction roundoff, not a physical volume floor.
                const double guard = 16.0 * std::numeric_limits<double>::epsilon() *
                    std::max( area * startFraction[target], chosen.outward );
                if( chosen.face < 0 || !( chosen.outward > deficit + guard ) ||
                    chosen.factor <= 0.0 || candidate.fluxVolume[chosen.neighbor] <= deficit + guard )
                    throw std::runtime_error( splitKey + " R4an no safe shared outgoing transfer" );
                auto& changedVelocity = ( chosen.isU ? trialU : trialV )[chosen.face];
                const double oldVelocity = changedVelocity;
                const double oldTransfer = chosen.factor * oldVelocity;
                changedVelocity *= 1.0 - ( deficit + guard ) / chosen.outward;
                const double transferReduction = std::abs( oldTransfer ) -
                    std::abs( chosen.factor * changedVelocity );
                const auto rebuildLedger = [&]( const std::vector<double>& u, const std::vector<double>& v )
                {
                    std::vector<double> ledger( startFraction.size() );
                    for( std::size_t i = 0; i < ledger.size(); ++i )
                        ledger[i] = area * startFraction[i];
                    const auto transfer = [&]( bool isU, int fx, int fy )
                    {
                        const int face = isU ? forward.uIndex( fx, fy ) : forward.vIndex( fx, fy );
                        const double aperture = ( isU ? candidate.meanU : candidate.meanV )[face];
                        if( aperture <= 0.0 ) return;
                        const int a = forward.cellIndex( fx - ( isU ? 1 : 0 ), fy - ( isU ? 0 : 1 ) );
                        const int b = forward.cellIndex( fx, fy );
                        const double amount = static_cast<double>( dt ) * h * aperture * ( isU ? u : v )[face];
                        ledger[a] -= amount; ledger[b] += amount;
                    };
                    for( int fy = 0; fy < forward.height_; ++fy )
                        for( int fx = 1; fx < forward.width_; ++fx ) transfer( true, fx, fy );
                    for( int fy = 1; fy < forward.height_; ++fy )
                        for( int fx = 0; fx < forward.width_; ++fx ) transfer( false, fx, fy );
                    return ledger;
                };
                if( rebuildLedger( candidate.intervalU, candidate.intervalV ) != candidate.fluxVolume )
                    throw std::runtime_error( splitKey + " R4an baseline ledger differs" );
                const auto corrected = rebuildLedger( trialU, trialV );
                double sumDelta = 0.0, gapBefore = 0.0, gapAfter = 0.0, maxDelta = 0.0;
                int changedCells = 0;
                for( std::size_t i = 0; i < corrected.size(); ++i )
                {
                    const double delta = corrected[i] - candidate.fluxVolume[i];
                    sumDelta += delta;
                    maxDelta = std::max( maxDelta, std::abs( delta ) );
                    changedCells += delta != 0.0;
                    gapBefore += std::abs( area * predictedFraction[i] - candidate.fluxVolume[i] );
                    gapAfter += std::abs( area * predictedFraction[i] - corrected[i] );
                    if( !std::isfinite( corrected[i] ) || corrected[i] < 0.0 ||
                        ( i != target && i != static_cast<std::size_t>( chosen.neighbor ) && delta != 0.0 ) )
                        throw std::runtime_error( splitKey + " R4an invalid corrected shared-face ledger" );
                }
                const double conservationError = std::abs( sumDelta );
                const double conservationBound = 32.0 * std::numeric_limits<double>::epsilon() *
                    ( area * startFraction[target] + std::abs( candidate.fluxVolume[chosen.neighbor] ) +
                        std::abs( oldTransfer ) );
                if( !( transferReduction > 0.0 ) || corrected[target] > 2.0 * guard ||
                    conservationError > conservationBound || changedCells != 2 )
                    throw std::runtime_error( splitKey + " R4an transfer conservation audit failed" );
                std::printf( "R4an interval64 target %zu neighbor %d face %c%d deficit %.17g reduction %.17g "
                    "targetAfter %.17g neighborBefore %.17g neighborAfter %.17g conservationError %.17g "
                    "gapBefore %.17g gapAfter %.17g maxDelta %.17g velocityDelta %.17g changedCells %d PCG %d\n",
                    target, chosen.neighbor, chosen.isU ? 'U' : 'V', chosen.face, deficit, transferReduction,
                    corrected[target], candidate.fluxVolume[chosen.neighbor], corrected[chosen.neighbor],
                    conservationError, gapBefore, gapAfter, maxDelta, changedVelocity - oldVelocity,
                    changedCells, candidate.initialIterations );

                // R4ao: distinguish a transport-only correction from an
                // interval pressure update. A nonzero circulation around an
                // alternate graph path cannot come from a scalar pressure.
                std::vector<std::vector<int>> adjacency( startFraction.size() );
                const auto connect = [&]( bool isU, int fx, int fy )
                {
                    const int f = isU ? forward.uIndex( fx, fy ) : forward.vIndex( fx, fy );
                    if( ( isU == chosen.isU && f == chosen.face ) ||
                        ( isU ? candidate.meanU : candidate.meanV )[f] <= 0.0 ) return;
                    const int a = forward.cellIndex( fx - ( isU ? 1 : 0 ), fy - ( isU ? 0 : 1 ) );
                    const int b = forward.cellIndex( fx, fy );
                    adjacency[a].push_back( b );
                    adjacency[b].push_back( a );
                };
                for( int fy = 0; fy < forward.height_; ++fy )
                    for( int fx = 1; fx < forward.width_; ++fx ) connect( true, fx, fy );
                for( int fy = 1; fy < forward.height_; ++fy )
                    for( int fx = 0; fx < forward.width_; ++fx ) connect( false, fx, fy );
                std::vector<int> distance( startFraction.size(), -1 );
                std::vector<int> pending{ static_cast<int>( target ) };
                distance[target] = 0;
                for( std::size_t i = 0; i < pending.size(); ++i )
                    for( int neighbor : adjacency[pending[i]] )
                        if( distance[neighbor] < 0 )
                        {
                            distance[neighbor] = distance[pending[i]] + 1;
                            pending.push_back( neighbor );
                        }
                const int alternativePath = distance[chosen.neighbor];
                const double velocityDelta = changedVelocity - oldVelocity;
                const double cycleCirculation = alternativePath >= 0 ? h * std::abs( velocityDelta ) : 0.0;
                const double weight = h * h *
                    ( chosen.isU ? candidate.meanU : candidate.meanV )[chosen.face];
                const double momentumProxy = weight * velocityDelta;
                const double energyProxy = 0.5 * weight * velocityDelta * ( changedVelocity + oldVelocity );
                const int stride = forward.width_ + ( chosen.isU ? 1 : 0 );
                const int fx = chosen.face % stride, fy = chosen.face / stride;
                const double endpointOpening = chosen.isU ? forward.cutCellUOpen( fx, fy ) :
                    forward.cutCellVOpen( fx, fy );
                if( endpointOpening != 0.0 )
                    throw std::runtime_error( splitKey + " R4ao corrected face still open at endpoint" );

                // The corrected face disappears from the endpoint MAC graph.
                // Independently reconstruct both outputs to verify that the
                // correction is transport-only for this particular interval.
                continuationPressure correctedTransport = candidate;
                correctedTransport.intervalU = trialU; correctedTransport.intervalV = trialV;
                correctedTransport.fluxVolume = corrected;
                std::vector<float> originalU, originalV, correctedU, correctedV;
                std::string originalEndpoint, correctedEndpoint;
                captureInstantaneousEndpoint( forward, candidate, originalEndpoint, index,
                    &originalU, &originalV, true, true );
                captureInstantaneousEndpoint( forward, correctedTransport, correctedEndpoint, index,
                    &correctedU, &correctedV, true, true );
                if( originalU != correctedU || originalV != correctedV ||
                    originalEndpoint != correctedEndpoint )
                    throw std::runtime_error( splitKey + " R4ao transport correction changed endpoint" );
                if( !std::isfinite( cycleCirculation ) || !std::isfinite( momentumProxy ) ||
                    !std::isfinite( energyProxy ) )
                    throw std::runtime_error( splitKey + " R4ao nonfinite transport audit" );
                std::printf( "R4ao interval64 alternativePath %d cycleCirculation %.17g "
                    "momentumProxy %.17g energyProxy %.17g endpointOpening %.17g "
                    "endpointIdentical 1 pressureUnchanged 1\n",
                    alternativePath, cycleCirculation, momentumProxy, energyProxy, endpointOpening );
                for( bool warm : { false, true } )
                {
                    const std::string trialKey = splitKey + ( warm ? ".R4alWarmStrict" : ".R4alColdStrict" );
                    continuationPressure trial;
                    std::string trialScratch;
                    captureSpaceTimeProjection( forward, laggedField, laggedLoad, trialScratch,
                        trialKey + ".projection", 16, trialKey + ".volume", false,
                        16, trialKey + ".topology", 0.0, &trial, 0, true,
                        nullptr, 1.0e-16, warm, {}, true, true, true );
                    report( warm ? "warmStrict" : "coldStrict", trial );
                }
                for( double localTolerance : { 1.0e-8, 1.0e-10 } )
                {
                    const char* name = localTolerance == 1.0e-8 ? "closingRelative8" : "closingRelative10";
                    const std::string trialKey = splitKey + ".R4am." + name;
                    continuationPressure trial;
                    std::string trialScratch;
                    captureSpaceTimeProjection( forward, laggedField, laggedLoad, trialScratch,
                        trialKey + ".projection", 16, trialKey + ".volume", false,
                        16, trialKey + ".topology", 0.0, &trial, 0, true,
                        nullptr, 1.0e-10, true, {}, true, true, true, localTolerance );
                    report( name, trial );
                }
            }
            // R4af: matched-state pressure-only trials. Do not replace the
            // accepted candidate or include trial time in the R4ae total.
            // Benchmark probes are timed separately in Release.
            for( int variant = 0; !optimized && variant < 3; ++variant )
            {
                const bool warm = variant != 0;
                const double tolerance = variant == 2 ? 1.0e-10 : 1.0e-16;
                const std::string trialKey = "forwardFixedPressureCost.interval" +
                    std::to_string( index ) + ".variant" + std::to_string( variant );
                continuationPressure alternative;
                std::string alternativeScratch;
                forwardTiming trialTiming{};
                const auto trialStart = timing ? benchmarkClock::now() : benchmarkClock::time_point{};
                captureSpaceTimeProjection( forward, laggedField, laggedLoad, alternativeScratch,
                    trialKey + ".projection", 16, trialKey + ".volume", false,
                    16, trialKey + ".topology", 0.0, &alternative, 0, true,
                    timing ? &trialTiming : nullptr, tolerance, warm, {}, true, true, true );
                const auto trialEnd = timing ? benchmarkClock::now() : benchmarkClock::time_point{};
                double gap = 0.0, negative = 0.0, pressureDelta = 0.0, velocityDelta = 0.0;
                double forceDelta = 0.0, torqueDelta = 0.0, ledgerDelta = 0.0;
                int negativeCells = 0, unsupportedCells = 0;
                for( std::size_t i = 0; i < alternative.fluxVolume.size(); ++i )
                {
                    const double volume = alternative.fluxVolume[i];
                    const double error = std::abs( area * predictedFraction[i] - volume );
                    gap += error;
                    ledgerDelta = std::max( ledgerDelta, std::abs( volume - candidate.fluxVolume[i] ) );
                    if( volume < 0.0 ) { negative -= volume; ++negativeCells; }
                    unsupportedCells += alternative.component[i] < 0 && error != 0.0;
                    pressureDelta = std::max( pressureDelta, std::abs(
                        static_cast<double>( alternative.pressure[i] ) - candidate.pressure[i] ) );
                }
                const auto compareVelocity = [&]( const std::vector<double>& a, const std::vector<double>& b )
                {
                    for( std::size_t i = 0; i < a.size(); ++i )
                        velocityDelta = std::max( velocityDelta, std::abs( a[i] - b[i] ) );
                };
                compareVelocity( alternative.intervalU, candidate.intervalU );
                compareVelocity( alternative.intervalV, candidate.intervalV );
                for( std::size_t j = 0; j < alternative.feedback.size(); ++j )
                {
                    forceDelta = std::max( forceDelta, std::max(
                        std::abs( static_cast<double>( alternative.feedback[j].force.x ) - candidate.feedback[j].force.x ),
                        std::abs( static_cast<double>( alternative.feedback[j].force.y ) - candidate.feedback[j].force.y ) ) );
                    torqueDelta = std::max( torqueDelta, std::abs(
                        static_cast<double>( alternative.feedback[j].torque ) - candidate.feedback[j].torque ) );
                }
                if( variant == 0 && ( pressureDelta != 0.0 || velocityDelta != 0.0 ||
                    forceDelta != 0.0 || torqueDelta != 0.0 || ledgerDelta != 0.0 ||
                    alternative.initialIterations != candidate.initialIterations ||
                    alternative.trueResidualSquared != candidate.trueResidualSquared ) )
                    throw std::runtime_error( trialKey + " fixed operator changed numerical result" );
                const auto emitTrial = [&]( const char* suffix, auto value )
                {
                    if( !std::isfinite( static_cast<double>( value ) ) )
                        throw std::runtime_error( trialKey + suffix + " nonfinite fixed pressure trial" );
                    appendScalar( trace, trialKey + suffix, value );
                };
                emitTrial( ".iterations", alternative.initialIterations );
                emitTrial( ".relativeSquaredResidual", alternative.initialResidualSquared > 0.0 ?
                    alternative.trueResidualSquared / alternative.initialResidualSquared : 0.0 );
                emitTrial( ".seedRelativeSquared", alternative.initialResidualSquared > 0.0 ?
                    alternative.seedResidualSquared / alternative.initialResidualSquared : 0.0 );
                emitTrial( ".negativeVolume", negative ); emitTrial( ".negativeCells", negativeCells );
                emitTrial( ".unsupportedCells", unsupportedCells ); emitTrial( ".actualGap", gap );
                emitTrial( ".pressureDelta", pressureDelta ); emitTrial( ".velocityDelta", velocityDelta );
                emitTrial( ".forceDelta", forceDelta ); emitTrial( ".torqueDelta", torqueDelta );
                emitTrial( ".ledgerDelta", ledgerDelta ); emitTrial( ".peak", alternative.intervalPeak );
                std::printf( "%s PCG %d negative %.9g (%d cells) gap %.9g pressure delta %.9g "
                    "velocity delta %.9g force delta %.9g torque delta %.9g ledger delta %.9g\n",
                    trialKey.c_str(), alternative.initialIterations, negative, negativeCells, gap,
                    pressureDelta, velocityDelta, forceDelta, torqueDelta, ledgerDelta );
                if( timing )
                    std::printf( "R4af timing interval=%d variant=%d linear_ms=%.6f pressure_ms=%.6f\n",
                        index, variant, trialTiming.linearMs, elapsedMs( trialStart, trialEnd ) );
            }
            const auto trialsEnd = timing ? benchmarkClock::now() : benchmarkClock::time_point{};
            const double trialsMs = timing ? elapsedMs( pressureEnd, trialsEnd ) : 0.0;
            double gap = 0.0, negative = 0.0, unsupported = 0.0;
            int negativeCells = 0, unsupportedCells = 0;
            for( std::size_t i = 0; i < startFraction.size(); ++i )
            {
                const double volume = candidate.fluxVolume[i];
                const double error = std::abs( area * predictedFraction[i] - volume );
                gap += error;
                if( volume < 0.0 ) { negative -= volume; ++negativeCells; }
                if( candidate.component[i] < 0 )
                {
                    unsupported += error;
                    unsupportedCells += error != 0.0;
                }
            }
            if( unsupportedCells != 0 )
                throw std::runtime_error( splitKey + " unsupported volume" );
            const auto endpointStart = timing ? benchmarkClock::now() : benchmarkClock::time_point{};
            std::vector<float> endpointU, endpointV;
            captureInstantaneousEndpoint( forward, candidate, trace, index, &endpointU, &endpointV, true, optimized );
            forward.u_ = std::move( endpointU ); forward.v_ = std::move( endpointV );
            forward.pressure_ = candidate.pressure;
            forward.applyCutCellBoundaries();
            forward.previousObstacles_ = predicted;
            forward.frozenPressureObstacles_ = predicted;
            forward.previousCellFluidFraction_ = forward.cellFluidFraction_;
            if( timing ) timing->endpointMs = elapsedMs( endpointStart, benchmarkClock::now() );
            const auto emit = [&]( const std::string& suffix, auto value )
            {
                if( !std::isfinite( static_cast<double>( value ) ) )
                    throw std::runtime_error( splitKey + suffix + " nonfinite staggered state" );
                appendScalar( trace, splitKey + suffix, value );
            };
            emit( ".actualGap", gap ); emit( ".negativeVolume", negative );
            emit( ".negativeCells", negativeCells ); emit( ".unsupportedVolume", unsupported );
            emit( ".unsupportedCells", unsupportedCells );
            emit( ".peak", candidate.intervalPeak ); emit( ".iterations", candidate.initialIterations );
            emit( ".trueResidualSquared", candidate.trueResidualSquared );
            if( optimized )
            {
                emit( ".relativeSquaredResidual", candidate.initialResidualSquared > 0.0 ?
                    candidate.trueResidualSquared / candidate.initialResidualSquared : 0.0 );
                emit( ".seedRelativeSquared", candidate.initialResidualSquared > 0.0 ?
                    candidate.seedResidualSquared / candidate.initialResidualSquared : 0.0 );
            }
            double maxForceDelta = 0.0, maxTorqueDelta = 0.0;
            for( std::size_t j = 0; j < predicted.size(); ++j )
            {
                const auto k = ".body[" + std::to_string( j ) + "]";
                emit( k + ".position.x", predicted[j].transform.position.x );
                emit( k + ".position.y", predicted[j].transform.position.y );
                emit( k + ".rotation.c", predicted[j].transform.rotation.c );
                emit( k + ".rotation.s", predicted[j].transform.rotation.s );
                emit( k + ".velocity.x", predicted[j].linearVelocity.x );
                emit( k + ".velocity.y", predicted[j].linearVelocity.y );
                emit( k + ".omega", predicted[j].angularVelocity );
                emit( k + ".appliedForce.x", laggedLoad[j].force.x );
                emit( k + ".appliedForce.y", laggedLoad[j].force.y );
                emit( k + ".appliedTorque", laggedLoad[j].torque );
                emit( k + ".nextForce.x", candidate.feedback[j].force.x );
                emit( k + ".nextForce.y", candidate.feedback[j].force.y );
                emit( k + ".nextTorque", candidate.feedback[j].torque );
                maxForceDelta = std::max( maxForceDelta, std::max(
                    std::abs( static_cast<double>( candidate.feedback[j].force.x ) - laggedLoad[j].force.x ),
                    std::abs( static_cast<double>( candidate.feedback[j].force.y ) - laggedLoad[j].force.y ) ) );
                maxTorqueDelta = std::max( maxTorqueDelta, std::abs(
                    static_cast<double>( candidate.feedback[j].torque ) - laggedLoad[j].torque ) );
            }
            emit( ".maxForceDelta", maxForceDelta ); emit( ".maxTorqueDelta", maxTorqueDelta );
            appendVector( trace, splitKey + ".endpointU", forward.u_ );
            appendVector( trace, splitKey + ".endpointV", forward.v_ );
            appendVector( trace, splitKey + ".endpointPressure", forward.pressure_ );
            appendVector( trace, splitKey + ".endpointFraction", forward.cellFluidFraction_ );
            std::printf( "%s gap %.9g negative %.9g (%d cells) peak %.9g PCG %d "
                "next force delta %.9g torque delta %.9g\n", splitKey.c_str(), gap, negative,
                negativeCells, candidate.intervalPeak, candidate.initialIterations, maxForceDelta, maxTorqueDelta );
            committed->field = std::move( forward );
            committed->poses = predicted;
            committed->feedback = std::move( candidate.feedback );
            if( timing ) timing->totalMs = elapsedMs( totalStart, benchmarkClock::now() ) - trialsMs;
            return;
        }
        // Use the lagged load only as the reference for diagnostic deltas.
        // Neither the lagged nor the candidate load is applied twice.
        captureSpaceTimeProjection( forward, laggedField, laggedLoad, scratch,
            key + ".projection", samples, key + ".volume", false, 16,
            key + ".topology", responseScale, &candidate, correctionBudget, adaptiveEndpoint, timing,
            relativeSquaredTolerance, warmStart, usedPreviousForward ?
            std::span<const float>( *forwardPressureHistory ) : std::span<const float>{}, auditEndpoint );
        const auto corrected = run( candidate.feedback );
        rebuild( corrected );
        if( auditEndpoint )
        {
            captureCorrectedEndpoint( forward, startFraction, candidate, dt, trace, index, committed != nullptr );
            std::vector<float> endpointU, endpointV;
            captureInstantaneousEndpoint( forward, candidate, trace, index,
                committed ? &endpointU : nullptr, committed ? &endpointV : nullptr );
            if( committed )
            {
                forward.u_ = std::move( endpointU ); forward.v_ = std::move( endpointV );
                forward.pressure_ = candidate.pressure;
                forward.applyCutCellBoundaries();
                forward.previousObstacles_ = corrected;
                forward.frozenPressureObstacles_ = corrected;
                forward.previousCellFluidFraction_ = forward.cellFluidFraction_;
            }
        }

        const std::size_t n = startFraction.size();
        std::vector<double> residual( n ), sums( n );
        std::vector<int> counts( n );
        double rawL1 = 0.0, maxResidual = 0.0, unsupportedL1 = 0.0;
        double startVolume = 0.0, correctedVolume = 0.0, fluxVolume = 0.0, predictedGap = 0.0;
        int unsupportedCells = 0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const double fraction = forward.cellFluidFraction_[i];
            const double modeled = candidate.fluxVolume[i];
            if( !std::isfinite( fraction ) || fraction < 0.0 || fraction > 1.0 ||
                !std::isfinite( modeled ) )
                throw std::runtime_error( key + " invalid measured endpoint volume" );
            residual[i] = area * fraction - modeled;
            rawL1 += std::abs( residual[i] );
            maxResidual = std::max( maxResidual, std::abs( residual[i] ) );
            predictedGap += std::abs( area * predictedFraction[i] - modeled );
            startVolume += area * startFraction[i];
            correctedVolume += area * fraction;
            fluxVolume += modeled;
            const int component = candidate.component[i];
            if( component >= 0 )
            {
                sums[component] += residual[i];
                ++counts[component];
            }
            else
            {
                unsupportedL1 += std::abs( residual[i] );
                unsupportedCells += residual[i] != 0.0;
            }
        }
        double projectedL1 = 0.0, projectedSquared = 0.0, compatibility = 0.0;
        for( std::size_t i = 0; i < n; ++i )
        {
            const int component = candidate.component[i];
            if( component < 0 ) continue;
            const double mean = sums[component] / counts[component];
            const double projected = residual[i] - mean;
            compatibility += std::abs( mean );
            projectedL1 += std::abs( projected );
            projectedSquared += projected * projected;
        }
        const auto out = [&]( const std::string& suffix, auto value )
        {
            if( !std::isfinite( static_cast<double>( value ) ) )
                throw std::runtime_error( key + suffix + " nonfinite forward audit" );
            appendScalar( trace, key + suffix, value );
        };
        out( ".rawL1", rawL1 ); out( ".maxResidual", maxResidual );
        out( ".projectedL1", projectedL1 ); out( ".projectedSquared", projectedSquared );
        out( ".compatibilityL1", compatibility ); out( ".negativeModelVolume", candidate.negativeVolume );
        out( ".unsupportedL1", unsupportedL1 ); out( ".unsupportedCells", unsupportedCells );
        out( ".intervalPeak", candidate.intervalPeak ); out( ".initialDefect", candidate.initialGap );
        out( ".modelFinalDefect", candidate.finalGap ); out( ".uncoveredPredictedFaces", candidate.uncoveredEndpointFaces );
        out( ".stalled", static_cast<int>( candidate.stalled ) ); out( ".startVolume", startVolume );
        out( ".correctedVolume", correctedVolume ); out( ".fluxVolume", fluxVolume );
        out( ".predictedGap", predictedGap ); out( ".linearResidualSquared", candidate.trueResidualSquared );
        out( ".initialIterations", candidate.initialIterations );
        for( std::size_t j = 0; j < start.size(); ++j )
        {
            const auto k = ".body[" + std::to_string( j ) + "]";
            out( k + ".force.x", candidate.feedback[j].force.x );
            out( k + ".force.y", candidate.feedback[j].force.y );
            out( k + ".torque", candidate.feedback[j].torque );
            out( k + ".predicted.center.x", predicted[j].center.x );
            out( k + ".predicted.center.y", predicted[j].center.y );
            out( k + ".corrected.center.x", corrected[j].center.x );
            out( k + ".corrected.center.y", corrected[j].center.y );
            out( k + ".corrected.omega", corrected[j].angularVelocity );
        }
        if( responseScale == 0.75 && samples == 64 )
        {
            const std::string costKey = "forwardBudgetCost.pass" + std::to_string( correctionBudget ) +
                ".interval" + std::to_string( index );
            appendScalar( trace, costKey + ".initialIterations", candidate.initialIterations );
            appendScalar( trace, costKey + ".correctionIterations", candidate.correctionIterations );
            appendScalar( trace, costKey + ".defectEvaluations", candidate.defectEvaluations );
            std::printf( "%s initial PCG %d correction PCG %d defect evaluations %d\n",
                costKey.c_str(), candidate.initialIterations, candidate.correctionIterations,
                candidate.defectEvaluations );
        }
        if( responseScale == 0.75 && correctionBudget == 0 && !adaptiveEndpoint )
        {
            const std::string costKey = "forwardQuadratureCost.samples" + std::to_string( samples ) +
                ".interval" + std::to_string( index );
            appendScalar( trace, costKey + ".midpointCoverageChecks", samples );
            appendScalar( trace, costKey + ".weightedPanels", samples + 32 );
            appendScalar( trace, costKey + ".initialIterations", candidate.initialIterations );
            std::printf( "%s midpoint checks %d weighted panels %d initial PCG %d\n",
                costKey.c_str(), samples, samples + 32, candidate.initialIterations );
        }
        if( adaptiveEndpoint )
        {
            out( ".midpointCoverageChecks", samples );
            out( ".weightedPanels", samples + 2 * candidate.selectedEndpointDepth );
            out( ".missedMidpointCells", candidate.missedMidpointCells );
            out( ".selectedEndpointDepth", candidate.selectedEndpointDepth );
            std::printf( "%s midpoint checks %d weighted panels %d missed cells %d selected depth %d\n",
                key.c_str(), samples, samples + 2 * candidate.selectedEndpointDepth,
                candidate.missedMidpointCells, candidate.selectedEndpointDepth );
        }
        if( experimentalResidual )
        {
            const double relativeSquared = candidate.initialResidualSquared > 0.0 ?
                candidate.trueResidualSquared / candidate.initialResidualSquared : 0.0;
            out( ".relativeSquaredTolerance", relativeSquaredTolerance );
            out( ".trueRelativeSquaredResidual", relativeSquared );
            std::printf( "%s initial PCG %d true relative squared residual %.9g target %.9g\n",
                key.c_str(), candidate.initialIterations, relativeSquared, relativeSquaredTolerance );
        }
        if( warmStart )
        {
            const double seedRelativeSquared = candidate.initialResidualSquared > 0.0 ?
                candidate.seedResidualSquared / candidate.initialResidualSquared : 0.0;
            out( ".seedRelativeSquaredResidual", seedRelativeSquared );
            std::printf( "%s seed relative squared residual %.9g\n",
                key.c_str(), seedRelativeSquared );
        }
        std::printf( "%s predicted gap %.9g actual corrected gap %.9g m^2 modeled gap %.9g "
            "unsupported cells %d unsupported volume %.9g interval peak %.9g m/s negative volume %.9g\n",
            key.c_str(), predictedGap, rawL1, candidate.finalGap, unsupportedCells, unsupportedL1,
            candidate.intervalPeak, candidate.negativeVolume );
        if( forwardPressureHistory )
        {
            out( ".usedPreviousForward", static_cast<int>( usedPreviousForward ) );
            // Carry only the binary32 pressure seed, not the candidate World
            // or field. This is an independent matched-state experiment.
            *forwardPressureHistory = std::move( candidate.pressure );
        }
        if( committed )
        {
            if( unsupportedCells != 0 ) throw std::runtime_error( key + " unsupported corrected volume" );
            captureMinimumVolumeCell( input, forward, startFraction, predictedFraction, candidate, dt, trace, index );
            if( index == 57 )
            {
                captureActualGeometryLedger( input, forward, start, corrected, candidate, trace, index );
                // R4z: bounded fixed-point experiment. Every pressure load is
                // replayed from the original World state, never accumulated
                // on a previous trial's endpoint. Keep the accepted trajectory.
                // Compare deterministic load relaxation without treating the
                // partially applied load as the full pressure solution.
                for( float relaxation : { 1.0f, 0.5f, 0.25f } )
                {
                    auto estimate = corrected;
                    auto estimateLoad = candidate.feedback;
                    for( int pass = 0; pass < ( relaxation == 1.0f ? 3 : 6 ); ++pass )
                    {
                        const std::string iterationKey = std::string( relaxation == 1.0f ?
                            "forwardGeometryFeedback.interval" : "forwardRelaxedGeometryFeedback.interval" ) +
                            std::to_string( index ) + ( relaxation == 1.0f ? "" :
                                relaxation == 0.5f ? ".half" : ".quarter" ) +
                            ".pass" + std::to_string( pass + 1 );
                        fluidField2 fixed = forward;
                        fixed.updateObstacles( estimate );
                        fixed.updateCutCellGeometry( estimate );
                        if( !fixed.cutCellTransition_.usedDisjointAnalyticGeometry )
                            throw std::runtime_error( iterationKey + " estimated geometry not analytic" );
                        fixed.previousCellFluidFraction_ = startFraction;
                        fixed.previousObstacles_ = start;
                        fixed.frozenPressureObstacles_ = estimate;
                        fixed.volumeSourceValid_ = true;
                        fixed.u_ = input.u_; fixed.v_ = input.v_;
                        continuationPressure trial;
                        std::string trialScratch;
                        captureSpaceTimeProjection( fixed, forward, estimateLoad, trialScratch,
                            iterationKey + ".projection", 16, iterationKey + ".volume", false,
                            16, iterationKey + ".topology", 0.0, &trial, 0, true,
                            nullptr, 1.0e-16, false, {}, true, true );
                        auto appliedLoad = trial.feedback;
                        if( relaxation != 1.0f )
                            for( std::size_t j = 0; j < appliedLoad.size(); ++j )
                            {
                                appliedLoad[j].force = estimateLoad[j].force +
                                    relaxation * ( trial.feedback[j].force - estimateLoad[j].force );
                                appliedLoad[j].torque = estimateLoad[j].torque +
                                    relaxation * ( trial.feedback[j].torque - estimateLoad[j].torque );
                            }
                        const auto nextEstimate = run( appliedLoad );
                        fluidField2 actual = fixed;
                        actual.updateObstacles( nextEstimate );
                        actual.updateCutCellGeometry( nextEstimate );
                        if( !actual.cutCellTransition_.usedDisjointAnalyticGeometry )
                            throw std::runtime_error( iterationKey + " loaded geometry not analytic" );
                        double negative = 0.0, minimum = 0.0, guessedGap = 0.0, actualGap = 0.0;
                        double maxResidual = 0.0, geometryDelta = 0.0, unsupportedVolume = 0.0;
                        int negativeCells = 0, unsupportedCells = 0, uncoveredFaces = 0;
                        for( std::size_t i = 0; i < trial.fluxVolume.size(); ++i )
                        {
                            const double volume = trial.fluxVolume[i];
                            const double guessed = area * fixed.cellFluidFraction_[i];
                            const double measured = area * actual.cellFluidFraction_[i];
                            const double error = std::abs( measured - volume );
                            if( volume < 0.0 ) { negative -= volume; ++negativeCells; }
                            minimum = std::min( minimum, volume );
                            guessedGap += std::abs( guessed - volume );
                            actualGap += error;
                            maxResidual = std::max( maxResidual, error );
                            geometryDelta += std::abs( measured - guessed );
                            if( trial.component[i] < 0 )
                            {
                                unsupportedCells += error != 0.0;
                                unsupportedVolume += error;
                            }
                        }
                        for( int y = 0; y < actual.height_; ++y )
                            for( int x = 1; x < actual.width_; ++x )
                                uncoveredFaces += actual.cutCellUOpen( x, y ) > 0.0f &&
                                    trial.meanU[actual.uIndex( x, y )] <= 0.0;
                        for( int y = 1; y < actual.height_; ++y )
                            for( int x = 0; x < actual.width_; ++x )
                                uncoveredFaces += actual.cutCellVOpen( x, y ) > 0.0f &&
                                    trial.meanV[actual.vIndex( x, y )] <= 0.0;
                        double maxForceDelta = 0.0, maxTorqueDelta = 0.0;
                        double maxCenterDelta = 0.0, maxRotationDelta = 0.0;
                        for( std::size_t j = 0; j < estimate.size(); ++j )
                        {
                            maxForceDelta = std::max( maxForceDelta, std::max(
                                std::abs( static_cast<double>( trial.feedback[j].force.x ) - estimateLoad[j].force.x ),
                                std::abs( static_cast<double>( trial.feedback[j].force.y ) - estimateLoad[j].force.y ) ) );
                            maxTorqueDelta = std::max( maxTorqueDelta, std::abs(
                                static_cast<double>( trial.feedback[j].torque ) - estimateLoad[j].torque ) );
                            maxCenterDelta = std::max( maxCenterDelta, std::max(
                                std::abs( static_cast<double>( nextEstimate[j].center.x ) - estimate[j].center.x ),
                                std::abs( static_cast<double>( nextEstimate[j].center.y ) - estimate[j].center.y ) ) );
                            maxRotationDelta = std::max( maxRotationDelta, std::max(
                                std::abs( static_cast<double>( nextEstimate[j].transform.rotation.c ) -
                                    estimate[j].transform.rotation.c ),
                                std::abs( static_cast<double>( nextEstimate[j].transform.rotation.s ) -
                                    estimate[j].transform.rotation.s ) ) );
                        }
                        const auto emit = [&]( const char* suffix, auto value )
                        {
                            if( !std::isfinite( static_cast<double>( value ) ) )
                                throw std::runtime_error( iterationKey + suffix + " nonfinite geometry feedback" );
                            appendScalar( trace, iterationKey + suffix, value );
                        };
                        const int target = actual.cellIndex( 17, 14 );
                        emit( ".targetLedger", trial.fluxVolume[target] );
                        emit( ".targetGuess", area * fixed.cellFluidFraction_[target] );
                        emit( ".targetActual", area * actual.cellFluidFraction_[target] );
                        emit( ".negativeVolume", negative ); emit( ".negativeCells", negativeCells );
                        emit( ".minimumVolume", minimum ); emit( ".guessedGap", guessedGap );
                        emit( ".actualGap", actualGap ); emit( ".maxActualResidual", maxResidual );
                        emit( ".geometryDeltaL1", geometryDelta ); emit( ".unsupportedCells", unsupportedCells );
                        emit( ".unsupportedVolume", unsupportedVolume ); emit( ".uncoveredActualFaces", uncoveredFaces );
                        emit( ".intervalPeak", trial.intervalPeak ); emit( ".initialIterations", trial.initialIterations );
                        emit( ".trueResidualSquared", trial.trueResidualSquared );
                        emit( ".maxForceDelta", maxForceDelta ); emit( ".maxTorqueDelta", maxTorqueDelta );
                        emit( ".maxCenterComponentDelta", maxCenterDelta );
                        emit( ".maxRotationComponentDelta", maxRotationDelta );
                        std::printf( "%s ledger %.9g negative %.9g (%d cells) guessed gap %.9g actual gap %.9g "
                            "geometry delta %.9g unsupported %d uncovered faces %d peak %.9g "
                            "PCG %d force delta %.9g torque delta %.9g center delta %.9g rotation delta %.9g\n",
                            iterationKey.c_str(), trial.fluxVolume[target], negative, negativeCells,
                            guessedGap, actualGap, geometryDelta, unsupportedCells, uncoveredFaces,
                            trial.intervalPeak, trial.initialIterations, maxForceDelta, maxTorqueDelta,
                            maxCenterDelta, maxRotationDelta );
                        if( relaxation != 1.0f )
                        {
                            double appliedForceDelta = 0.0, appliedTorqueDelta = 0.0;
                            double unappliedForce = 0.0, unappliedTorque = 0.0;
                            for( std::size_t j = 0; j < appliedLoad.size(); ++j )
                            {
                                appliedForceDelta = std::max( appliedForceDelta, std::max(
                                    std::abs( static_cast<double>( appliedLoad[j].force.x ) - estimateLoad[j].force.x ),
                                    std::abs( static_cast<double>( appliedLoad[j].force.y ) - estimateLoad[j].force.y ) ) );
                                appliedTorqueDelta = std::max( appliedTorqueDelta, std::abs(
                                    static_cast<double>( appliedLoad[j].torque ) - estimateLoad[j].torque ) );
                                unappliedForce = std::max( unappliedForce, std::max(
                                    std::abs( static_cast<double>( trial.feedback[j].force.x ) - appliedLoad[j].force.x ),
                                    std::abs( static_cast<double>( trial.feedback[j].force.y ) - appliedLoad[j].force.y ) ) );
                                unappliedTorque = std::max( unappliedTorque, std::abs(
                                    static_cast<double>( trial.feedback[j].torque ) - appliedLoad[j].torque ) );
                            }
                            emit( ".relaxation", relaxation );
                            emit( ".appliedForceDelta", appliedForceDelta );
                            emit( ".appliedTorqueDelta", appliedTorqueDelta );
                            emit( ".unappliedForce", unappliedForce );
                            emit( ".unappliedTorque", unappliedTorque );
                            std::printf( "%s applied force delta %.9g torque delta %.9g "
                                "unapplied force %.9g torque %.9g\n", iterationKey.c_str(),
                                appliedForceDelta, appliedTorqueDelta, unappliedForce, unappliedTorque );
                        }
                        // R4ac: replay the same two endpoint estimates with
                        // precise capsule construction, including the common start.
                        if( relaxation == 0.5f && pass == 3 )
                        {
                            fluidField2 preciseStart = input;
                            preciseStart.preciseDisjointCapsuleEnabled_ = true;
                            preciseStart.updateObstacles( start );
                            preciseStart.updateCutCellGeometry( start );
                            if( !preciseStart.cutCellTransition_.usedDisjointAnalyticGeometry )
                                throw std::runtime_error( "R4ac precise start geometry not analytic" );
                            std::array<continuationPressure, 2> results;
                            std::array<std::vector<float>, 2> fractions;
                            for( int k = 0; k < 2; ++k )
                            {
                                const auto& poses = k == 0 ? estimate : nextEstimate;
                                fluidField2 geometry = preciseStart;
                                geometry.updateObstacles( poses );
                                geometry.updateCutCellGeometry( poses );
                                if( !geometry.cutCellTransition_.usedDisjointAnalyticGeometry )
                                    throw std::runtime_error( "R4ac precise endpoint not analytic" );
                                fractions[k] = geometry.cellFluidFraction_;
                                geometry.previousCellFluidFraction_ = preciseStart.cellFluidFraction_;
                                geometry.previousObstacles_ = start;
                                geometry.frozenPressureObstacles_ = poses;
                                geometry.volumeSourceValid_ = true;
                                geometry.u_ = input.u_; geometry.v_ = input.v_;
                                const std::string probeKey = "forwardPreciseCapsule.interval" +
                                    std::to_string( index ) + ".estimate" + std::to_string( k );
                                std::string preciseScratch;
                                captureSpaceTimeProjection( geometry, forward, trial.feedback, preciseScratch,
                                    probeKey + ".projection", 16, probeKey + ".volume", false, 16,
                                    probeKey + ".topology", 0.0, &results[k], 0, true,
                                    nullptr, 1.0e-16, false, {}, true, true );
                                double signedChange = 0.0, gap = 0.0;
                                for( std::size_t i = 0; i < fractions[k].size(); ++i )
                                {
                                    signedChange += area * ( static_cast<double>( fractions[k][i] ) -
                                        preciseStart.cellFluidFraction_[i] );
                                    gap += std::abs( area * fractions[k][i] - results[k].fluxVolume[i] );
                                }
                                appendScalar( trace, probeKey + ".signedVolumeChange", signedChange );
                                appendScalar( trace, probeKey + ".geometryGap", gap );
                                appendScalar( trace, probeKey + ".negativeVolume", results[k].negativeVolume );
                                appendScalar( trace, probeKey + ".peak", results[k].intervalPeak );
                                appendScalar( trace, probeKey + ".iterations", results[k].initialIterations );
                                appendScalar( trace, probeKey + ".trueResidualSquared", results[k].trueResidualSquared );
                                std::printf( "%s signed volume %.9g gap %.9g negative %.9g peak %.9g PCG %d\n",
                                    probeKey.c_str(), signedChange, gap, results[k].negativeVolume,
                                    results[k].intervalPeak, results[k].initialIterations );
                            }
                            double volumeL1 = 0.0, signedVolume = 0.0, compatibilityL1 = 0.0;
                            double forceDelta = 0.0, torqueDelta = 0.0, pressureDelta = 0.0;
                            int topologyChanges = 0;
                            for( std::size_t i = 0; i < fractions[0].size(); ++i )
                            {
                                const double dv = area * ( static_cast<double>( fractions[1][i] ) - fractions[0][i] );
                                volumeL1 += std::abs( dv ); signedVolume += dv;
                                topologyChanges += ( fractions[0][i] > 0.0f ) != ( fractions[1][i] > 0.0f );
                                compatibilityL1 += std::abs( results[1].compatibilityVolume[i] -
                                    results[0].compatibilityVolume[i] );
                                pressureDelta = std::max( pressureDelta, std::abs(
                                    static_cast<double>( results[1].pressure[i] ) - results[0].pressure[i] ) );
                            }
                            for( std::size_t j = 0; j < results[0].feedback.size(); ++j )
                            {
                                forceDelta = std::max( forceDelta, std::max(
                                    std::abs( static_cast<double>( results[1].feedback[j].force.x ) - results[0].feedback[j].force.x ),
                                    std::abs( static_cast<double>( results[1].feedback[j].force.y ) - results[0].feedback[j].force.y ) ) );
                                torqueDelta = std::max( torqueDelta, std::abs(
                                    static_cast<double>( results[1].feedback[j].torque ) - results[0].feedback[j].torque ) );
                            }
                            const std::string probeKey = "forwardPreciseCapsule.interval" + std::to_string( index ) + ".delta";
                            appendScalar( trace, probeKey + ".volumeL1", volumeL1 );
                            appendScalar( trace, probeKey + ".signedVolume", signedVolume );
                            appendScalar( trace, probeKey + ".topologyChanges", topologyChanges );
                            appendScalar( trace, probeKey + ".compatibilityL1", compatibilityL1 );
                            appendScalar( trace, probeKey + ".pressureMaxDelta", pressureDelta );
                            appendScalar( trace, probeKey + ".forceMaxDelta", forceDelta );
                            appendScalar( trace, probeKey + ".torqueMaxDelta", torqueDelta );
                            std::printf( "%s volume L1 %.9g signed %.9g compatibility %.9g topology %d "
                                "pressure delta %.9g force delta %.9g torque delta %.9g\n",
                                probeKey.c_str(), volumeL1, signedVolume, compatibilityL1, topologyChanges,
                                pressureDelta, forceDelta, torqueDelta );
                        }
                        // R4ab: isolate the actual half-relaxation pass4 -> pass5
                        // geometry jump. Replace one body's endpoint at a time,
                        // then all endpoints. None of the probes is applied.
                        if( relaxation == 0.5f && pass == 3 )
                        {
                            for( int changed = 0; changed <= static_cast<int>( estimate.size() ); ++changed )
                            {
                                const std::string probeKey = "forwardGeometrySensitivity.interval" +
                                    std::to_string( index ) + ".body" + std::to_string( changed );
                                auto probePoses = estimate;
                                if( changed == static_cast<int>( estimate.size() ) )
                                    probePoses = nextEstimate;
                                else
                                    probePoses[changed] = nextEstimate[changed];
                                fluidField2 probeField = fixed;
                                probeField.updateObstacles( probePoses );
                                probeField.updateCutCellGeometry( probePoses );
                                if( !probeField.cutCellTransition_.usedDisjointAnalyticGeometry )
                                    throw std::runtime_error( probeKey + " probe geometry not analytic" );
                                probeField.previousCellFluidFraction_ = startFraction;
                                probeField.previousObstacles_ = start;
                                probeField.frozenPressureObstacles_ = probePoses;
                                probeField.volumeSourceValid_ = true;
                                probeField.u_ = input.u_; probeField.v_ = input.v_;
                                continuationPressure probe;
                                std::string probeScratch;
                                captureSpaceTimeProjection( probeField, forward, trial.feedback, probeScratch,
                                    probeKey + ".projection", 16, probeKey + ".volume", false,
                                    16, probeKey + ".topology", 0.0, &probe, 0, true,
                                    nullptr, 1.0e-16, false, {}, true, true );
                                double volumeDelta = 0.0, volumeSigned = 0.0, maxVolumeDelta = 0.0;
                                double compatibilityDelta = 0.0, compatibilitySigned = 0.0;
                                double apertureDelta = 0.0, maxApertureDelta = 0.0;
                                double forceDelta = 0.0, torqueDelta = 0.0, pressureDelta = 0.0;
                                int volumeCell = -1, topologyChanges = 0, apertureChanges = 0;
                                int supportChanges = 0, negativeCells = 0;
                                double negative = 0.0, gap = 0.0;
                                for( std::size_t i = 0; i < probe.fluxVolume.size(); ++i )
                                {
                                    const double dv = area * ( static_cast<double>( probeField.cellFluidFraction_[i] ) -
                                        fixed.cellFluidFraction_[i] );
                                    volumeDelta += std::abs( dv ); volumeSigned += dv;
                                    if( std::abs( dv ) > maxVolumeDelta )
                                    {
                                        maxVolumeDelta = std::abs( dv );
                                        volumeCell = static_cast<int>( i );
                                    }
                                    topologyChanges += ( probeField.cellFluidFraction_[i] > 0.0f ) !=
                                        ( fixed.cellFluidFraction_[i] > 0.0f );
                                    supportChanges += ( probe.component[i] >= 0 ) != ( trial.component[i] >= 0 );
                                    const double dc = probe.compatibilityVolume[i] - trial.compatibilityVolume[i];
                                    compatibilityDelta += std::abs( dc ); compatibilitySigned += dc;
                                    pressureDelta = std::max( pressureDelta, std::abs(
                                        static_cast<double>( probe.pressure[i] ) - trial.pressure[i] ) );
                                    if( probe.fluxVolume[i] < 0.0 )
                                    {
                                        negative -= probe.fluxVolume[i]; ++negativeCells;
                                    }
                                    gap += std::abs( area * probeField.cellFluidFraction_[i] - probe.fluxVolume[i] );
                                }
                                const auto compareApertures = [&]( const std::vector<double>& a,
                                    const std::vector<double>& b )
                                {
                                    if( a.size() != b.size() )
                                        throw std::runtime_error( probeKey + " aperture size mismatch" );
                                    for( std::size_t i = 0; i < a.size(); ++i )
                                    {
                                        const double da = std::abs( a[i] - b[i] );
                                        apertureDelta += da;
                                        maxApertureDelta = std::max( maxApertureDelta, da );
                                        apertureChanges += ( a[i] > 0.0 ) != ( b[i] > 0.0 );
                                    }
                                };
                                compareApertures( probe.meanU, trial.meanU );
                                compareApertures( probe.meanV, trial.meanV );
                                for( std::size_t j = 0; j < probe.feedback.size(); ++j )
                                {
                                    forceDelta = std::max( forceDelta, std::max(
                                        std::abs( static_cast<double>( probe.feedback[j].force.x ) - trial.feedback[j].force.x ),
                                        std::abs( static_cast<double>( probe.feedback[j].force.y ) - trial.feedback[j].force.y ) ) );
                                    torqueDelta = std::max( torqueDelta, std::abs(
                                        static_cast<double>( probe.feedback[j].torque ) - trial.feedback[j].torque ) );
                                }
                                const auto emitProbe = [&]( const char* suffix, auto value )
                                {
                                    if( !std::isfinite( static_cast<double>( value ) ) )
                                        throw std::runtime_error( probeKey + suffix + " nonfinite sensitivity" );
                                    appendScalar( trace, probeKey + suffix, value );
                                };
                                emitProbe( ".volumeL1", volumeDelta ); emitProbe( ".volumeSigned", volumeSigned );
                                emitProbe( ".maxVolumeDelta", maxVolumeDelta ); emitProbe( ".maxVolumeCell", volumeCell );
                                emitProbe( ".topologyChanges", topologyChanges ); emitProbe( ".supportChanges", supportChanges );
                                emitProbe( ".compatibilityL1", compatibilityDelta ); emitProbe( ".compatibilitySigned", compatibilitySigned );
                                emitProbe( ".meanApertureL1", apertureDelta ); emitProbe( ".maxMeanApertureDelta", maxApertureDelta );
                                emitProbe( ".apertureSupportChanges", apertureChanges ); emitProbe( ".pressureMaxDelta", pressureDelta );
                                emitProbe( ".forceMaxDelta", forceDelta ); emitProbe( ".torqueMaxDelta", torqueDelta );
                                emitProbe( ".negativeVolume", negative ); emitProbe( ".negativeCells", negativeCells );
                                emitProbe( ".geometryGap", gap ); emitProbe( ".peak", probe.intervalPeak );
                                emitProbe( ".iterations", probe.initialIterations ); emitProbe( ".trueResidualSquared", probe.trueResidualSquared );
                                std::printf( "%s volume L1 %.9g signed %.9g max %.9g cell %d "
                                    "compatibility L1 %.9g signed %.9g aperture L1 %.9g max %.9g "
                                    "topology %d/%d/%d pressure delta %.9g force delta %.9g torque delta %.9g "
                                    "negative %.9g (%d cells) gap %.9g PCG %d\n",
                                    probeKey.c_str(), volumeDelta, volumeSigned, maxVolumeDelta, volumeCell,
                                    compatibilityDelta, compatibilitySigned, apertureDelta, maxApertureDelta,
                                    topologyChanges, supportChanges, apertureChanges, pressureDelta,
                                    forceDelta, torqueDelta, negative, negativeCells, gap, probe.initialIterations );
                            }
                        }
                        estimate = nextEstimate;
                        estimateLoad = std::move( appliedLoad );
                    }
                }
            }
            appendVector( trace, key + ".endpointU", forward.u_ );
            appendVector( trace, key + ".endpointV", forward.v_ );
            appendVector( trace, key + ".endpointPressure", forward.pressure_ );
            appendVector( trace, key + ".endpointFraction", forward.cellFluidFraction_ );
            for( std::size_t j = 0; j < corrected.size(); ++j )
            {
                const auto k = ".body[" + std::to_string( j ) + "]";
                out( k + ".position.x", corrected[j].transform.position.x );
                out( k + ".position.y", corrected[j].transform.position.y );
                out( k + ".rotation.c", corrected[j].transform.rotation.c );
                out( k + ".rotation.s", corrected[j].transform.rotation.s );
                out( k + ".velocity.x", corrected[j].linearVelocity.x );
                out( k + ".velocity.y", corrected[j].linearVelocity.y );
            }
            double bodyDeltaX = 0.0, bodyDeltaY = 0.0, pressureImpulseX = 0.0, pressureImpulseY = 0.0;
            for( std::size_t j = 0; j < corrected.size(); ++j )
            {
                if( start[j].inverseMass <= 0.0f )
                    throw std::runtime_error( key + " non-dynamic momentum fixture" );
                const double mass = 1.0 / static_cast<double>( start[j].inverseMass );
                bodyDeltaX += mass * ( static_cast<double>( corrected[j].linearVelocity.x ) -
                    start[j].linearVelocity.x );
                bodyDeltaY += mass * ( static_cast<double>( corrected[j].linearVelocity.y ) -
                    start[j].linearVelocity.y );
                pressureImpulseX += static_cast<double>( dt ) * candidate.feedback[j].force.x;
                pressureImpulseY += static_cast<double>( dt ) * candidate.feedback[j].force.y;
            }
            const std::string budgetKey = "forwardPersistentMomentum.interval" + std::to_string( index );
            const auto emitBudget = [&]( const char* suffix, double value )
            {
                if( !std::isfinite( value ) )
                    throw std::runtime_error( budgetKey + " nonfinite body budget" );
                appendScalar( trace, budgetKey + suffix, value );
            };
            emitBudget( ".bodyDeltaX", bodyDeltaX ); emitBudget( ".bodyDeltaY", bodyDeltaY );
            emitBudget( ".pressureImpulseX", pressureImpulseX ); emitBudget( ".pressureImpulseY", pressureImpulseY );
            std::printf( "%s body delta (%.9g, %.9g) applied pressure impulse (%.9g, %.9g)\n",
                budgetKey.c_str(), bodyDeltaX, bodyDeltaY, pressureImpulseX, pressureImpulseY );
            committed->field = std::move( forward );
            committed->poses = corrected;
            committed->feedback = candidate.feedback;
        }
        if( timing ) timing->totalMs = elapsedMs( totalStart, benchmarkClock::now() );
    }

    // R4j: actual persistent free-body World + MAC continuation. This
    // deliberately exercises the *lagged* production scheduling, not a
    // forward space-time predictor/corrector. Tracer is excluded.
    static void captureLaggedContinuation( const fluidField2& initial,
        const fluidField2& expectedInitialProjection, std::string& trace )
    {
        using namespace sandbox;
        constexpr int intervals = 4, worldSubsteps = 2, initialIndex = 43;
        const float dt = initial.frozenPressureTimeStep_;
        const auto& obstacles = initial.frozenPressureObstacles_;
        if( obstacles.size() != 3 || dt <= 0.0f )
            throw std::runtime_error( "R4j missing continuation input" );
        for( int model : { 0, 1, 2 } )
        {
            const std::string name = model == 0 ? "production" : model == 1 ? "fullResponse" : "cadenceResponse";
            rigidBodyDemo scene{ demoKind::windFlow };
            auto& world = scene.getWorld();
            if( scene.getShapes().size() != obstacles.size() )
                throw std::runtime_error( "R4j fixture shape count changed" );
            for( std::size_t j = 0; j < obstacles.size(); ++j )
            {
                const auto id = scene.getShapes()[j].bodyHandle;
                if( 1.0f / world.GetBodyMass( id ) != obstacles[j].inverseMass ||
                    1.0f / world.GetBodyRotationalInertia( id ) != obstacles[j].inverseInertia )
                    throw std::runtime_error( "R4j fixture mass differs" );
                world.SetBodyTransform( id, obstacles[j].transform );
                world.SetBodyLinearVelocity( id, obstacles[j].linearVelocity );
                world.SetBodyAngularVelocity( id, obstacles[j].angularVelocity );
                const vec2 center = TransformPoint( world.GetBodyTransform( id ), world.GetBodyLocalCenter( id ) );
                if( center.x != obstacles[j].center.x || center.y != obstacles[j].center.y )
                    throw std::runtime_error( "R4j fixture COM differs" );
            }
            const auto snapshot = [&]()
            {
                std::vector<fluidShapeObstacle2> poses = obstacles;
                for( std::size_t j = 0; j < poses.size(); ++j )
                {
                    const auto id = scene.getShapes()[j].bodyHandle;
                    poses[j].transform = world.GetBodyTransform( id );
                    poses[j].center = TransformPoint( poses[j].transform, world.GetBodyLocalCenter( id ) );
                    poses[j].linearVelocity = world.GetBodyLinearVelocity( id );
                    poses[j].angularVelocity = world.GetBodyAngularVelocity( id );
                }
                return poses;
            };
            fluidField2 field = initial;
            field.setPrePressureSnapshotObserver( {} );
            setStageObserver( field, {} );
            field.tracerEnabled_ = false;
            field.setCoupledPressureWarmStartEnabled( false );
            field.setCoupledPressureWarmStartProbeEnabled( false );
            float windTime = 0.0f;
            for( int i = 0; i <= initialIndex; ++i ) windTime += dt;
            double maxPeak = 0.0, maxContinuity = 0.0, maxNegative = 0.0;
            std::vector<float> forwardPressureHistory;
            for( int step = 0; step < intervals; ++step )
            {
                const std::string key = "laggedContinuation." + name + ".interval" +
                    std::to_string( initialIndex + step );
                std::optional<fluidField2> pre;
                std::vector<fluidObstacleForce2> feedback( obstacles.size() );
                const auto poses = snapshot();
                if( step == 0 )
                {
                    pre.emplace( initial );
                    const auto replay = field.replayFrozenCutCellPressure();
                    if( !replay || replay->size() != feedback.size() )
                        throw std::runtime_error( key + " missing initial pressure replay" );
                    feedback = *replay;
                    std::string actual, expected;
                    captureProjection( field, actual, "initial" );
                    captureProjection( expectedInitialProjection, expected, "initial" );
                    if( actual != expected )
                        throw std::runtime_error( key + " initial production replay differs from historical projection" );
                    field.previousObstacles_ = poses;
                }
                else
                {
                    // Same source schedule as the historical Sandbox; this
                    // initial snapshot already includes source increment 44.
                    windTime += dt;
                    const float sourceX = field.getOrigin().x +
                        std::min( 2.5f, 0.25f * field.getWidth() * field.getCellSize() );
                    field.addVelocity( { sourceX, 0.0f }, { 8.0f * dt,
                        0.5f * 8.0f * dt * rot2::FromRadians( 1.7f * windTime ).s }, 1.7f );
                    field.setPrePressureSnapshotObserver( [&]( const fluidField2& input ) { pre.emplace( input ); } );
                    field.step( dt, poses, feedback, 200 );
                    field.setPrePressureSnapshotObserver( {} );
                }
                if( !pre || !pre->volumeSourceValid_ )
                    throw std::runtime_error( key + " missing pre-pressure history" );
                pre->setPrePressureSnapshotObserver( {} );
                setStageObserver( *pre, {} );
                const auto productionStatus = field.getCutCellPressure().coupledStatus;
                const bool productionFailed = field.getCutCellPressure().coupledSolveFailed;
                const auto productionFlow = field.getFlowAudit( dt );
                continuationPressure candidate;
                if( model != 0 )
                {
                    std::string scratch;
                    captureSpaceTimeProjection( *pre, field, feedback, scratch, key + ".projection",
                        step == 0 ? 32 : 64, key + ".volume", step == 0, step == 0 ? 0 : 16,
                        key + ".topology", model == 1 ? 1.0 : 0.75, &candidate );
                    field.u_ = candidate.u; field.v_ = candidate.v; field.pressure_ = candidate.pressure;
                    feedback = candidate.feedback;
                    // The discarded production projection changed solid
                    // boundary speed. Restore pre-projection values before
                    // syncing with the accepted candidate load, exactly once.
                    field.uSolidVelocity_ = pre->uSolidVelocity_;
                    field.vSolidVelocity_ = pre->vSolidVelocity_;
                    field.cutCellPressure_ = {};
                    field.synchronizeCutCellBoundaries( dt, poses, feedback );
                }
                // The historical frozen replay does not finalize obstacle
                // history. Every next step consumes the actual current pose.
                field.previousObstacles_ = poses;
                const auto flow = field.getFlowAudit( dt );
                const auto finite = []( const std::vector<float>& values )
                {
                    return std::all_of( values.begin(), values.end(), []( float v ) { return std::isfinite( v ); } );
                };
                if( flow.nonFiniteFluidFaces != 0 || !std::isfinite( flow.continuityRms ) ||
                    !finite( field.u_ ) || !finite( field.v_ ) || !finite( field.pressure_ ) ||
                    !finite( field.cellFluidFraction_ ) )
                    throw std::runtime_error( key + " nonfinite committed field" );
                const auto out = [&]( const std::string& suffix, auto value )
                {
                    if( !std::isfinite( static_cast<double>( value ) ) )
                        throw std::runtime_error( key + suffix + " nonfinite continuation" );
                    appendScalar( trace, key + suffix, value );
                };
                out( ".productionStatus", static_cast<int>( productionStatus ) );
                out( ".productionFailed", static_cast<int>( productionFailed ) );
                out( ".productionPeak", productionFlow.maxFluidFaceSpeed );
                out( ".endpointPeak", flow.maxFluidFaceSpeed );
                out( ".endpointContinuity", flow.continuityRms );
                out( ".peakOpening", flow.maxFaceOpenFraction );
                out( ".initialDefect", candidate.initialGap );
                out( ".finalDefect", candidate.finalGap );
                out( ".negativeVolume", candidate.negativeVolume );
                out( ".intervalPeak", candidate.intervalPeak );
                out( ".uncoveredEndpointFaces", candidate.uncoveredEndpointFaces );
                out( ".stalled", static_cast<int>( candidate.stalled ) );
                out( ".windTime", windTime );
                appendVector( trace, key + ".u", field.u_ );
                appendVector( trace, key + ".v", field.v_ );
                appendVector( trace, key + ".pressure", field.pressure_ );
                appendVector( trace, key + ".fraction", field.cellFluidFraction_ );
                for( std::size_t j = 0; j < poses.size(); ++j )
                {
                    const auto id = scene.getShapes()[j].bodyHandle;
                    const auto& load = feedback[j];
                    if( !IsFinite( load.force ) || !std::isfinite( load.torque ) )
                        throw std::runtime_error( key + " nonfinite committed pressure load" );
                    world.ApplyForceToCenter( id, load.force );
                    world.ApplyTorque( id, load.torque );
                }
                world.Step( dt, worldSubsteps );
                const auto next = snapshot();
                for( std::size_t j = 0; j < next.size(); ++j )
                {
                    const auto id = scene.getShapes()[j].bodyHandle;
                    if( world.GetBody( id ).contactCount != 0 || world.GetBody( id ).jointCount != 0 )
                        throw std::runtime_error( key + " unexpected constraint in free-body trajectory" );
                    const auto k = ".body[" + std::to_string( j ) + "]";
                    out( k + ".force.x", feedback[j].force.x ); out( k + ".force.y", feedback[j].force.y );
                    out( k + ".torque", feedback[j].torque );
                    out( k + ".next.position.x", next[j].transform.position.x );
                    out( k + ".next.position.y", next[j].transform.position.y );
                    out( k + ".next.rotation.c", next[j].transform.rotation.c );
                    out( k + ".next.rotation.s", next[j].transform.rotation.s );
                    out( k + ".next.velocity.x", next[j].linearVelocity.x );
                    out( k + ".next.velocity.y", next[j].linearVelocity.y );
                    out( k + ".next.omega", next[j].angularVelocity );
                }
                if( model == 1 )
                {
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step );
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step, 0.75 );
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step, 0.75, 0 );
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step, 0.75, 1 );
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step, 0.75, 0, 32 );
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step, 0.75, 0, 16 );
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step, 0.75, 0, 16, true );
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step,
                        0.75, 0, 16, true, nullptr, 1.0e-12 );
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step,
                        0.75, 0, 16, true, nullptr, 1.0e-10 );
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step,
                        0.75, 0, 16, true, nullptr, 1.0e-10, true, nullptr, true );
                    const auto previousForwardPressure = forwardPressureHistory;
                    captureForwardPredictor( *pre, field, poses, feedback, trace, initialIndex + step,
                        0.75, 0, 16, true, nullptr, 1.0e-10, true, &forwardPressureHistory );
                    if( benchmarkForward )
                    {
                        // Existing calls warm up the same fixture. Rotate the
                        // measured order; timing never enters the exact trace.
                        std::array<std::string, 7> reference;
                        for( int repeat = 0; repeat < 7; ++repeat )
                        {
                            for( int slot = 0; slot < 7; ++slot )
                            {
                                const int variant = ( slot + repeat ) % 7;
                                const int samples = variant == 0 ? 64 : 16;
                                const bool adaptive = variant >= 2;
                                const double relativeSquaredTolerance = variant == 3 ? 1.0e-12 :
                                    variant >= 4 ? 1.0e-10 : 1.0e-16;
                                forwardTiming timing;
                                std::string measured;
                                auto benchmarkHistory = previousForwardPressure;
                                captureForwardPredictor( *pre, field, poses, feedback, measured,
                                    initialIndex + step, 0.75, 0, samples, adaptive, &timing,
                                    relativeSquaredTolerance, variant >= 5,
                                    variant == 6 ? &benchmarkHistory : nullptr );
                                if( variant == 6 && benchmarkHistory != forwardPressureHistory )
                                    throw std::runtime_error( "R4s benchmark pressure history differs" );
                                if( repeat == 0 ) reference[variant] = std::move( measured );
                                else if( measured != reference[variant] )
                                    throw std::runtime_error( "R4p repeated benchmark trace differs" );
                                std::printf( "R4p timing interval=%d variant=%d repeat=%d quadrature_ms=%.6f "
                                    "linear_ms=%.6f total_ms=%.6f\n", initialIndex + step, variant, repeat,
                                    timing.quadratureMs, timing.linearMs, timing.totalMs );
                            }
                        }
                    }
                }
                maxPeak = std::max( maxPeak, static_cast<double>( flow.maxFluidFaceSpeed ) );
                maxContinuity = std::max( maxContinuity, static_cast<double>( flow.continuityRms ) );
                maxNegative = std::max( maxNegative, candidate.negativeVolume );
                std::printf( "%s endpoint peak %.9g m/s endpoint continuity %.9g /s "
                    "candidate gap %.9g negative volume %.9g uncovered faces %d production failed %d\n",
                    key.c_str(), flow.maxFluidFaceSpeed, flow.continuityRms, candidate.finalGap,
                    candidate.negativeVolume, candidate.uncoveredEndpointFaces, static_cast<int>( productionFailed ) );
            }
            std::printf( "R4j %s %d persistent intervals max endpoint peak %.9g m/s "
                "max endpoint continuity %.9g /s max negative volume %.9g m^2\n",
                name.c_str(), intervals, maxPeak, maxContinuity, maxNegative );
        }
    }

    static void captureProjection( const fluidField2& field, std::string& trace, const std::string& key )
    {
        appendVector( trace, key + ".u", field.u_ );
        appendVector( trace, key + ".v", field.v_ );
        appendVector( trace, key + ".pressure", field.pressure_ );
        appendVector( trace, key + ".divergence", field.divergence_ );
        appendVector( trace, key + ".volumeRate", field.cutCellVolumeRate_ );
    }

    // R4v: carry the reconstructed endpoint MAC and free-body poses
    // forward, rather than sampling a separate lagged trajectory each time.
    static void captureForwardContinuation( const fluidField2& initial,
        const fluidField2& initialProjection, std::span<const fluidObstacleForce2> initialLoad,
        std::string& trace, bool staggered = false, int benchmarkRepeat = -1, bool optimized = false )
    {
        constexpr int first = 43;
        const int intervals = optimized ? optimizedContinuationIntervals : 16;
        if( benchmarkRepeat >= 0 && !staggered )
            throw std::runtime_error( "R4ae timing requires staggered trajectory" );
        const float dt = initial.frozenPressureTimeStep_;
        forwardEndpointState state{ initial, initial.frozenPressureObstacles_,
            std::vector<fluidObstacleForce2>( initialLoad.begin(), initialLoad.end() ) };
        if( state.poses.size() != 3 || initialLoad.size() != 3 || dt <= 0.0f ||
            initial.width_ != 48 || initial.height_ != 28 )
            throw std::runtime_error( "R4v invalid initial fixture" );
        state.field.setPrePressureSnapshotObserver( {} );
        setStageObserver( state.field, {} );
        state.field.tracerEnabled_ = false;
        if( staggered )
        {
            state.field.preciseDisjointCapsuleEnabled_ = true;
            state.field.certifiedFullCellCoverageEnabled_ = optimized;
            state.field.updateObstacles( state.poses );
            state.field.updateCutCellGeometry( state.poses );
            if( !state.field.cutCellTransition_.usedDisjointAnalyticGeometry )
                throw std::runtime_error( "R4ad initial precise geometry not analytic" );
        }
        float windTime = 0.0f;
        for( int i = 0; i <= first; ++i ) windTime += dt;
        for( int step = 0; step < intervals; ++step )
        {
            const auto stepStart = benchmarkRepeat >= 0 ? benchmarkClock::now() : benchmarkClock::time_point{};
            if( optimized && benchmarkRepeat < 0 && first + step == 65 )
            {
                // Rebuild the actual failing start pose on an independent
                // copy with the certificate disabled; do not change state.
                fluidField2 raw = state.field;
                raw.certifiedFullCellCoverageEnabled_ = false;
                raw.updateObstacles( state.poses );
                raw.updateCutCellGeometry( state.poses );
                const int cell = raw.cellIndex( 27, 18 );
                const float original = raw.cellFluidFraction_[cell];
                const float corrected = state.field.cellFluidFraction_[cell];
                std::printf( "R4ak interval65 cell891 original %.17g corrected %.17g\n", original, corrected );
                if( !raw.cutCellTransition_.usedDisjointAnalyticGeometry ||
                    !( original > 0.0f && original <= 64.0 * std::numeric_limits<double>::epsilon() &&
                        corrected == 0.0f ) )
                    throw std::runtime_error( "R4ak interval65 full coverage regression" );
            }
            fluidField2 pre = state.field;
            if( step > 0 )
            {
                windTime += dt;
                const float sourceX = pre.getOrigin().x +
                    std::min( 2.5f, 0.25f * pre.getWidth() * pre.getCellSize() );
                pre.addVelocity( { sourceX, 0.0f }, { 8.0f * dt,
                    0.5f * 8.0f * dt * rot2::FromRadians( 1.7f * windTime ).s }, 1.7f );
                // Advect on the actual start geometry, before the forward
                // pressure/geometry trial. No discarded pressure solve.
                pre.applyCutCellBoundaries();
                if( pre.conservativeAdvectionEnabled_ )
                    pre.advectConservative( dt );
                else
                {
                    std::fill( pre.nextU_.begin(), pre.nextU_.end(), 0.0f );
                    std::fill( pre.nextV_.begin(), pre.nextV_.end(), 0.0f );
                    for( int y = 0; y < pre.height_; ++y )
                        for( int x = 1; x < pre.width_; ++x )
                        {
                            const vec2 point = pre.origin_ + pre.cellSize_ *
                                vec2{ static_cast<float>( x ), static_cast<float>( y ) + 0.5f };
                            const vec2 previous = point - dt * pre.sampleFrom( pre.u_, pre.v_, point );
                            pre.nextU_[pre.uIndex( x, y )] = pre.sampleFrom( pre.u_, pre.v_, previous ).x;
                        }
                    for( int y = 1; y < pre.height_; ++y )
                        for( int x = 0; x < pre.width_; ++x )
                        {
                            const vec2 point = pre.origin_ + pre.cellSize_ *
                                vec2{ static_cast<float>( x ) + 0.5f, static_cast<float>( y ) };
                            const vec2 previous = point - dt * pre.sampleFrom( pre.u_, pre.v_, point );
                            pre.nextV_[pre.vIndex( x, y )] = pre.sampleFrom( pre.u_, pre.v_, previous ).y;
                        }
                    pre.u_.swap( pre.nextU_ ); pre.v_.swap( pre.nextV_ );
                }
                pre.applyCutCellBoundaries();
            }
            // Keep reference/seed and start poses independent of the output
            // object so an accepted commit cannot alias its input.
            const fluidField2& seed = step == 0 ? initialProjection : state.field;
            const auto poses = state.poses;
            const auto load = state.feedback;
            forwardTiming timing{};
            const auto prepared = benchmarkRepeat >= 0 ? benchmarkClock::now() : benchmarkClock::time_point{};
            captureForwardPredictor( pre, seed, poses, load, trace, first + step,
                0.75, 0, 16, true, benchmarkRepeat >= 0 ? &timing : nullptr,
                1.0e-10, true, nullptr, true, &state, staggered, optimized );
            if( optimized )
            {
                // R4ah: audit actual carried state, not the predictor's
                // intermediate interval field. Keep body impulse accounting
                // separate from the fluid momentum reconstruction proxy.
                const std::string auditKey = "forwardExtendedAudit.interval" + std::to_string( first + step );
                double peakU = 0.0, peakV = 0.0, peakBodySpeed = 0.0, peakOmega = 0.0;
                double peakAppliedForce = 0.0, peakNextForce = 0.0;
                double maxLinearImpulseError = 0.0, maxAngularImpulseError = 0.0;
                double bodyKineticEnergy = 0.0, volume = 0.0;
                const auto peakComponent = [&]( const std::vector<float>& values )
                {
                    double peak = 0.0;
                    for( float v : values )
                    {
                        if( !std::isfinite( v ) ) throw std::runtime_error( auditKey + " nonfinite committed MAC" );
                        peak = std::max( peak, std::abs( static_cast<double>( v ) ) );
                    }
                    return peak;
                };
                peakU = peakComponent( state.field.u_ );
                peakV = peakComponent( state.field.v_ );
                const auto norm = []( vec2 v )
                {
                    const double x = v.x, y = v.y;
                    return std::sqrt( x * x + y * y );
                };
                for( std::size_t j = 0; j < poses.size(); ++j )
                {
                    const auto& before = poses[j];
                    const auto& after = state.poses[j];
                    if( before.inverseMass <= 0.0f || before.inverseInertia <= 0.0f )
                        throw std::runtime_error( auditKey + " invalid free-body inertia" );
                    const double mass = 1.0 / before.inverseMass, inertia = 1.0 / before.inverseInertia;
                    const double speed = norm( after.linearVelocity ), omega = after.angularVelocity;
                    peakBodySpeed = std::max( peakBodySpeed, speed );
                    peakOmega = std::max( peakOmega, std::abs( omega ) );
                    peakAppliedForce = std::max( peakAppliedForce, norm( load[j].force ) );
                    peakNextForce = std::max( peakNextForce, norm( state.feedback[j].force ) );
                    const double ex = mass * ( static_cast<double>( after.linearVelocity.x ) - before.linearVelocity.x ) -
                        static_cast<double>( dt ) * load[j].force.x;
                    const double ey = mass * ( static_cast<double>( after.linearVelocity.y ) - before.linearVelocity.y ) -
                        static_cast<double>( dt ) * load[j].force.y;
                    maxLinearImpulseError = std::max( maxLinearImpulseError, std::sqrt( ex * ex + ey * ey ) );
                    maxAngularImpulseError = std::max( maxAngularImpulseError, std::abs(
                        inertia * ( omega - before.angularVelocity ) - static_cast<double>( dt ) * load[j].torque ) );
                    bodyKineticEnergy += 0.5 * ( mass * speed * speed + inertia * omega * omega );
                }
                const double h = state.field.cellSize_;
                for( float fraction : state.field.cellFluidFraction_ ) volume += h * h * fraction;
                const double cflBound = dt * ( peakU + peakV ) / h;
                const auto emit = [&]( const char* suffix, double value )
                {
                    if( !std::isfinite( value ) ) throw std::runtime_error( auditKey + suffix + " nonfinite audit" );
                    appendScalar( trace, auditKey + suffix, value );
                };
                emit( ".elapsed", static_cast<double>( dt ) * ( step + 1 ) );
                emit( ".peakU", peakU ); emit( ".peakV", peakV ); emit( ".cflBound", cflBound );
                emit( ".peakBodySpeed", peakBodySpeed ); emit( ".peakOmega", peakOmega );
                emit( ".peakAppliedForce", peakAppliedForce ); emit( ".peakNextForce", peakNextForce );
                emit( ".maxLinearImpulseError", maxLinearImpulseError );
                emit( ".maxAngularImpulseError", maxAngularImpulseError );
                emit( ".bodyKineticEnergy", bodyKineticEnergy ); emit( ".geometricVolume", volume );
                std::printf( "%s elapsed %.9g s peak U/V %.9g/%.9g CFL bound %.9g body speed %.9g omega %.9g "
                    "applied/next force %.9g/%.9g impulse errors %.9g/%.9g body energy %.9g volume %.9g\n",
                    auditKey.c_str(), static_cast<double>( dt ) * ( step + 1 ), peakU, peakV, cflBound,
                    peakBodySpeed, peakOmega, peakAppliedForce, peakNextForce, maxLinearImpulseError,
                    maxAngularImpulseError, bodyKineticEnergy, volume );
            }
            if( benchmarkRepeat >= 0 )
            {
                const double total = elapsedMs( stepStart, prepared ) + timing.totalMs;
                std::printf( "%s timing interval=%d repeat=%d prepare_ms=%.6f setup_ms=%.6f "
                    "quadrature_ms=%.6f linear_ms=%.6f pressure_ms=%.6f endpoint_ms=%.6f total_ms=%.6f\n",
                    optimized ? "R4ag" : "R4ae", first + step, benchmarkRepeat, elapsedMs( stepStart, prepared ), timing.setupMs,
                    timing.quadratureMs, timing.linearMs, timing.pressureMs, timing.endpointMs, total );
            }
        }
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
void configureNearClosedFaceScene( zonai::sandbox::rigidBodyDemo& scene, int fluidSubsteps )
{
    constexpr float h = 0.25f;
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
}

// Real cadence experiment, same initial state and total time, not a
// same-state pressure comparison. Include all intervals, including warmup.
std::string runNearClosedCadenceAudit( float baselinePeak )
{
    using namespace zonai;
    using namespace zonai::sandbox;
    using probe = zonai::fluidStateTraceProbe2;
    constexpr int frames = 16;
    std::string trace;
    for( int fluidSubsteps : { 1, 2, 4, 8 } )
    {
        rigidBodyDemo scene{ demoKind::windFlow };
        configureNearClosedFaceScene( scene, fluidSubsteps );
        auto& field = const_cast<fluidField2&>( *scene.getWindField() );
        const std::string prefix = "nearClosedCadence.steps" + std::to_string( fluidSubsteps );
        std::vector<float> beforeU, beforeV;
        int calls = 0, failed = 0, converged = 0, peakIndex = -1;
        float peak = 0.0f, maxDelta = 0.0f, maxContinuity = 0.0f, peakOpen = 0.0f;
        probe::setStageObserver( field, [&]( const fluidField2& snapshot, int phase )
        {
            check( phase == calls % 10, "R3z cadence phase order" );
            const int index = calls++ / 10;
            if( phase == 7 ) probe::capturePrePressure( snapshot, beforeU, beforeV );
            if( phase != 8 ) return;
            const std::string key = prefix + ".interval." + std::to_string( index );
            const auto a = probe::captureStabilityPhase( snapshot, trace, key, &beforeU, &beforeV );
            const auto& p = snapshot.getCutCellPressure();
            const auto flow = snapshot.getFlowAudit( timeStep / fluidSubsteps );
            check( flow.nonFiniteFluidFaces == 0 && std::isfinite( flow.continuityRms ),
                "R3z nonfinite cadence flow" );
            failed += static_cast<int>( p.coupledSolveFailed );
            converged += static_cast<int>( p.usedCoupledProjection &&
                p.coupledStatus == fluidCoupledPressureStatus2::converged );
            probe::appendScalar( trace, key + ".failed", static_cast<int>( p.coupledSolveFailed ) );
            probe::appendScalar( trace, key + ".status", static_cast<int>( p.coupledStatus ) );
            probe::appendScalar( trace, key + ".iterations", p.coupledIterations );
            probe::appendScalar( trace, key + ".continuity", flow.continuityRms );
            probe::captureTargetV( snapshot, trace, key + ".target", 17, 13 );
            maxDelta = std::max( maxDelta, a.deltaPeak );
            maxContinuity = std::max( maxContinuity, flow.continuityRms );
            if( a.peak > peak )
            {
                peak = a.peak; peakIndex = index; peakOpen = a.peakOpen;
            }
        } );
        for( int tick = 1; tick <= frames; ++tick )
        {
            scene.step( timeStep, 8 );
            const auto feedback = scene.getWindPressureForces();
            check( feedback.size() == 3, "R3z cadence feedback count" );
            for( std::size_t i = 0; i < feedback.size(); ++i )
            {
                const auto key = prefix + ".frame." + std::to_string( tick ) + ".body[" + std::to_string( i ) + "]";
                const auto body = scene.getShapes()[i].bodyHandle;
                const auto transform = scene.getWorld().GetBodyTransform( body );
                const auto velocity = scene.getWorld().GetBodyLinearVelocity( body );
                const float angular = scene.getWorld().GetBodyAngularVelocity( body );
                check( IsFinite( transform.position ) && IsFinite( velocity ) && std::isfinite( angular ) &&
                    std::isfinite( transform.rotation.c ) && std::isfinite( transform.rotation.s ) &&
                    IsFinite( feedback[i].force ) && std::isfinite( feedback[i].torque ),
                    "R3z cadence nonfinite body state" );
                probe::appendScalar( trace, key + ".position.x", transform.position.x );
                probe::appendScalar( trace, key + ".position.y", transform.position.y );
                probe::appendScalar( trace, key + ".rotation.c", transform.rotation.c );
                probe::appendScalar( trace, key + ".rotation.s", transform.rotation.s );
                probe::appendScalar( trace, key + ".velocity.x", velocity.x );
                probe::appendScalar( trace, key + ".velocity.y", velocity.y );
                probe::appendScalar( trace, key + ".omega", angular );
                probe::appendScalar( trace, key + ".force.x", feedback[i].force.x );
                probe::appendScalar( trace, key + ".force.y", feedback[i].force.y );
                probe::appendScalar( trace, key + ".torque", feedback[i].torque );
            }
        }
        probe::setStageObserver( field, {} );
        check( calls == frames * fluidSubsteps * 10, "R3z cadence incomplete trace" );
        if( fluidSubsteps == 4 )
            check( std::bit_cast<std::uint32_t>( peak ) == std::bit_cast<std::uint32_t>( baselinePeak ),
                "R3z four-substep control did not reproduce historical peak" );
        std::printf( "R3z cadence World8/Fluid%d: %d converged, %d failed, peak %.9g m/s index %d "
            "opening %.9g, max delta %.9g m/s, max continuity %.9g /s\n",
            fluidSubsteps, converged, failed, peak, peakIndex, peakOpen, maxDelta, maxContinuity );
    }
    return trace;
}

std::string runNearClosedFaceScene()
{
    using probe = zonai::fluidStateTraceProbe2;
    using namespace zonai;
    using namespace zonai::sandbox;
    constexpr int frames = 16;
    constexpr int fluidSubsteps = 4;
    constexpr int targetIndex = 43; // warmup 4 + measured frame 6, interval 3 (zero-based)
    rigidBodyDemo scene{ demoKind::windFlow };
    configureNearClosedFaceScene( scene, fluidSubsteps );
    auto& field = const_cast<fluidField2&>( *scene.getWindField() );
    std::optional<fluidField2> frozen;
    std::array<std::optional<fluidField2>, 2> adjacent;
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
        else if( frozenCalls == targetIndex - 1 ) adjacent[0].emplace( snapshot );
        else if( frozenCalls == targetIndex + 1 ) adjacent[1].emplace( snapshot );
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

    // Separate source audit segment; preserve every R3w trace row.
    // Do not erase a volume source or alter any aperture to reduce speed.
    const std::string sourceKey = "volumeSourceAudit.frozen.";
    probe::captureVolumeSourceAudit( *frozen, baseline, trace, sourceKey + "baseline", 17, 13 );
    probe::captureSliverFluxBudget( *frozen, baseline, trace, "sliverFluxAudit.baseline", 17, 13 );
    probe::captureSliverSampling( baseline, trace, "sliverFluxAudit.sampling" );
    probe::captureSpaceTimeAperture( *frozen, baseline, trace, "spaceTimeApertureAudit.frozen", 17, 13 );
    for( int samples : { 8, 32 } )
        probe::captureSpaceTimeProjection( *frozen, baseline, *a, trace,
            "spaceTimeProjection.midpoint" + std::to_string( samples ), samples,
            "intervalHandoff.target.midpoint" + std::to_string( samples ) );
    // R4i: separate diagnostic namespace for the two-substep free-body
    // displacement model. Preserve all prior trace rows and their keys.
    const auto cadenceReplay = [&]( const fluidField2& input, const fluidField2& output,
        std::span<const fluidObstacleForce2> feedback, const std::string& label,
        bool requireTarget, int endpointDepth )
    {
        std::string variant;
        probe::captureSpaceTimeProjection( input, output, feedback, variant,
            label + ".projection", endpointDepth > 0 ? 64 : 32,
            label + ".volume", requireTarget, endpointDepth, label + ".topology", 0.75 );
        std::size_t begin = 0;
        while( begin < variant.size() )
        {
            const auto end = variant.find( '\n', begin );
            check( end != std::string::npos, "R4i unterminated cadence trace" );
            trace.append( "cadencePressure." ).append( variant, begin, end - begin + 1 );
            begin = end + 1;
        }
    };
    cadenceReplay( *frozen, baseline, *a, "cadence43", true, 0 );
    probe::captureLaggedContinuation( *frozen, baseline, trace );
    probe::captureForwardContinuation( *frozen, baseline, *a, trace );
    if( probe::benchmarkForward )
    {
        std::string reference;
        probe::captureForwardContinuation( *frozen, baseline, *a, reference, true, 0 );
        for( int repeat = 1; repeat < 3; ++repeat )
        {
            std::string repeated;
            probe::captureForwardContinuation( *frozen, baseline, *a, repeated, true, repeat );
            check( repeated == reference, "R4ae staggered benchmark replay differs" );
        }
        trace += reference;
    }
    else probe::captureForwardContinuation( *frozen, baseline, *a, trace, true );
    if( probe::benchmarkForward )
    {
        std::string reference;
        probe::captureForwardContinuation( *frozen, baseline, *a, reference, true, 0, true );
        for( int repeat = 1; repeat < 3; ++repeat )
        {
            std::string repeated;
            probe::captureForwardContinuation( *frozen, baseline, *a, repeated, true, repeat, true );
            check( repeated == reference, "R4ag optimized benchmark replay differs" );
        }
        trace += reference;
    }
    else probe::captureForwardContinuation( *frozen, baseline, *a, trace, true, -1, true );
    // Adjacent actual intervals exercise the transition before/after the spike.
    // They are separate frozen solves, not a candidate-driven trajectory.
    for( std::size_t i = 0; i < adjacent.size(); ++i )
    {
        check( adjacent[i].has_value(), "R4b adjacent frozen interval missing" );
        const auto& input = *adjacent[i];
        fluidField2 output = input;
        output.setPrePressureSnapshotObserver( {} );
        probe::setStageObserver( output, {} );
        const auto feedback = output.replayFrozenCutCellPressure();
        check( feedback && feedback->size() == 3 &&
            output.getCutCellPressure().coupledStatus == fluidCoupledPressureStatus2::converged,
            "R4b adjacent baseline projection failed" );
        const auto key = "intervalHandoff.adjacent" + std::to_string( i == 0 ? targetIndex - 1 : targetIndex + 1 );
        probe::captureSpaceTimeProjection( input, output, *feedback, trace,
            key + ".projection", 64, key + ".volume", false, i == 1 ? 12 : 0,
            "topologyQuadrature.depth12" );
        if( i == 1 )
            probe::captureSpaceTimeProjection( input, output, *feedback, trace,
                "topologyQuadrature.depth16.projection", 64, "topologyQuadrature.depth16.volume",
                false, 16, "topologyQuadrature.depth16" );
        if( i == 1 )
            cadenceReplay( input, output, *feedback, "cadence44", false, 16 );
    }
    for( bool corrected : { false, true } )
    {
        fluidField2 input = *frozen;
        input.setPrePressureSnapshotObserver( {} );
        probe::setStageObserver( input, {} );
        if( corrected ) probe::useCorrectedSweptVolumeSource( input );
        else probe::useSnapshotVolumeSource( input );
        fluidField2 output = input;
        const auto feedback = output.replayFrozenCutCellPressure();
        const std::string key = sourceKey + ( corrected ? "correctedSwept" : "snapshot" );
        check( feedback && feedback->size() == a->size(), "R3x source replay missing feedback" );
        const auto& p = output.getCutCellPressure();
        check( p.usedCoupledProjection && !p.coupledSolveFailed &&
            p.coupledStatus == fluidCoupledPressureStatus2::converged,
            "R3x source replay pressure failed" );
        probe::captureVolumeSourceAudit( input, output, trace, key, 17, 13 );
        probe::captureSliverFluxBudget( input, output, trace,
            corrected ? "sliverFluxAudit.correctedSwept" : "sliverFluxAudit.snapshot", 17, 13 );
        const auto sample = probe::captureStabilityPhase( output, trace, key + ".ledger", &beforeU, &beforeV );
        float maxForceChange = 0.0f, maxTorqueChange = 0.0f;
        for( std::size_t i = 0; i < feedback->size(); ++i )
        {
            const auto prefix = key + ".body[" + std::to_string( i ) + "]";
            const auto& force = ( *feedback )[i];
            check( std::isfinite( force.force.x ) && std::isfinite( force.force.y ) &&
                std::isfinite( force.torque ), "R3x nonfinite source replay feedback" );
            probe::appendScalar( trace, prefix + ".force.x", force.force.x );
            probe::appendScalar( trace, prefix + ".force.y", force.force.y );
            probe::appendScalar( trace, prefix + ".torque", force.torque );
            const auto d = force.force - ( *a )[i].force;
            maxForceChange = std::max( maxForceChange, std::max( std::abs( d.x ), std::abs( d.y ) ) );
            maxTorqueChange = std::max( maxTorqueChange, std::abs( force.torque - ( *a )[i].torque ) );
        }
        probe::appendScalar( trace, key + ".maxForceComponentChange", maxForceChange );
        probe::appendScalar( trace, key + ".maxTorqueChange", maxTorqueChange );
        probe::appendScalar( trace, key + ".continuityRms", output.getFlowAudit( timeStep / fluidSubsteps ).continuityRms );
        probe::appendScalar( trace, key + ".pcgIterations", p.coupledIterations );
        probe::appendScalar( trace, key + ".tracerMass", output.getTracerMass() );
        std::printf( "%s peak %.9g m/s pressure delta %.9g m/s max force component change %.9g N "
            "max torque change %.9g Nm continuity %.9g /s\n",
            key.c_str(), sample.peak, sample.deltaPeak, maxForceChange, maxTorqueChange,
            output.getFlowAudit( timeStep / fluidSubsteps ).continuityRms );
    }
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
    trace += runNearClosedCadenceAudit( maxSpeed );
    return trace;
}

} // namespace

int main( int argc, char** argv )
{
    try
    {
        if( argc == 2 && std::string( argv[1] ) == "--benchmark-tracer" )
        {
            zonai::fluidStateTraceProbe2::benchmarkTracer = true;
            const auto trace = runNearClosedFaceScene();
            if( trace.empty() ) throw std::runtime_error( "R4bc missing fixture trace" );
            return EXIT_SUCCESS;
        }
        if( argc == 2 && std::string( argv[1] ) == "--benchmark-forward" )
        {
            zonai::fluidStateTraceProbe2::benchmarkForward = true;
            const auto trace = runNearClosedFaceScene();
            if( trace.empty() ) throw std::runtime_error( "R4p missing fixture trace" );
            return EXIT_SUCCESS;
        }
        if( argc != 1 && ( argc != 3 || std::string( argv[1] ) != "--emit" ) )
        {
            std::fprintf( stderr, "Usage: fluidStateDeterminismTests [--emit TRACE.tsv | --benchmark-forward | --benchmark-tracer]\n" );
            return EXIT_FAILURE;
        }

        zonai::fluidStateTraceProbe2::checkCertifiedCellCoverage();
        zonai::fluidStateTraceProbe2::checkClosingTransportRepair();

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
