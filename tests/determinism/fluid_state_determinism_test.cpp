#include "fluid/fluidField2.h"
#include "math/deterministicMath.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <functional>
#include <numbers>
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
    // This records the standard-library operation separately from the
    // candidate engine atan2; it does not alter any physical state.
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
            scalar( ".candidateStartAngle",
                deterministicAtan2( start.transform.rotation.s, start.transform.rotation.c ) );
            scalar( ".candidateEndAngle",
                deterministicAtan2( finish.transform.rotation.s, finish.transform.rotation.c ) );
        }
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

        if( argc == 3 )
        {
            std::FILE* output = std::fopen( argv[2], "wb" );
            if( !output ) throw std::runtime_error( "cannot create Fluid state trace" );
            const std::size_t written = std::fwrite( trace.data(), 1, trace.size(), output );
            const int closed = std::fclose( output );
            check( written == trace.size() && closed == 0, "cannot write Fluid state trace" );
        }

        std::puts( "Fluid sampled/analytic geometry, MAC state and pressure feedback replay passed" );
        return EXIT_SUCCESS;
    }
    catch( const std::exception& error )
    {
        std::fprintf( stderr, "Fluid determinism test error: %s\n", error.what() );
        return EXIT_FAILURE;
    }
}
