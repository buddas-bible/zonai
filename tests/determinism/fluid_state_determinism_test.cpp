#include "fluid/fluidField2.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <functional>
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
            zonai::fluidStateTraceProbe2::setStageObserver( field,
                [this, prefix]( const zonai::fluidField2& snapshot, int phase )
                {
                    static constexpr std::array<const char*, 10> names{
                        "00-entry", "01-occupancy", "02-geometry",
                        "03-swept", "04-remap", "05-firstBoundary",
                        "06-advection", "07-secondBoundary",
                        "08-pressure", "09-tracer"
                    };
                    check( phase >= 0 && phase < static_cast<int>( names.size() ),
                        "Fluid emitted an unknown Step phase" );
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
            check( first.stageTrace == second.stageTrace,
                "two independently instrumented Fields differ within one process" );
            trace.append( first.stageTrace );
        }

        const std::string prefix = std::string( sceneName ) + ".tick." + std::to_string( tick );
        const std::size_t firstStart = trace.size();
        zonai::fluidStateTraceProbe2::capture( first.field, trace, prefix );
        secondTrace.clear();
        zonai::fluidStateTraceProbe2::capture( second.field, secondTrace, prefix );
        check( trace.compare( firstStart, secondTrace.size(), secondTrace ) == 0,
            "two independent Fluid simulations diverged in one process" );
        controlTrace.clear();
        zonai::fluidStateTraceProbe2::capture( control.field, controlTrace, prefix );
        check( trace.compare( firstStart, controlTrace.size(), controlTrace ) == 0,
            "Fluid Stage observer perturbed authoritative field state" );

        zonai::fluidStateTraceProbe2::appendScalar( trace, prefix + ".force.x", first.feedback[0].force.x );
        zonai::fluidStateTraceProbe2::appendScalar( trace, prefix + ".force.y", first.feedback[0].force.y );
        zonai::fluidStateTraceProbe2::appendScalar( trace, prefix + ".torque", first.feedback[0].torque );
        check( std::bit_cast<std::uint32_t>( first.feedback[0].force.x ) ==
                std::bit_cast<std::uint32_t>( second.feedback[0].force.x ) &&
            std::bit_cast<std::uint32_t>( first.feedback[0].force.y ) ==
                std::bit_cast<std::uint32_t>( second.feedback[0].force.y ) &&
            std::bit_cast<std::uint32_t>( first.feedback[0].torque ) ==
                std::bit_cast<std::uint32_t>( second.feedback[0].torque ),
            "Fluid pressure feedback diverged in one process" );
    }

    check( first.sawFractionalCell && second.sawFractionalCell && control.sawFractionalCell &&
        first.sawFlow && second.sawFlow && control.sawFlow,
        "Fluid fixture did not exercise Cut-cell and flow" );
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

        const std::string trace =
            runScene( "sampledCutCell", false ) + runScene( "analyticCircle", true );

        if( argc == 3 )
        {
            std::FILE* output = std::fopen( argv[2], "wb" );
            if( !output ) throw std::runtime_error( "cannot create Fluid state trace" );
            const std::size_t written = std::fwrite( trace.data(), 1, trace.size(), output );
            const int closed = std::fclose( output );
            check( written == trace.size() && closed == 0, "cannot write Fluid state trace" );
        }

        std::puts( "Fluid sampled/analytic MAC state and pressure feedback replay passed" );
        return EXIT_SUCCESS;
    }
    catch( const std::exception& error )
    {
        std::fprintf( stderr, "Fluid determinism test error: %s\n", error.what() );
        return EXIT_FAILURE;
    }
}
