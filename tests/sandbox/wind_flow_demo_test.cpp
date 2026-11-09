#include "rigidBodyDemo.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace zonai;
using namespace zonai::sandbox;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

bool near( vec2 a, vec2 b, float tolerance = 1.0e-3f )
{
    return Length( a - b ) < tolerance;
}

bool close( float a, float b, float tolerance = 1.0e-4f )
{
    return std::abs( a - b ) <= tolerance;
}

void checkWindApertureSamplingDifferential()
{
    constexpr float dt = 1.0f / 60.0f;
    for( float drag : { 0.0f, 3.0f } )
    {
        rigidBodyDemo legacy{ demoKind::windFlow };
        rigidBodyDemo explicitOff{ demoKind::windFlow };
        rigidBodyDemo experiment{ demoKind::windFlow };
        for( rigidBodyDemo* scene : { &legacy, &explicitOff, &experiment } )
        {
            scene->configureWindField( 48, 28, 0.25f );
            scene->setWindCutCellProjectionEnabled( true );
            scene->setWindCutCellCoupledProjectionEnabled( true );
            scene->setWindDisjointAnalyticEnabled( true );
            scene->setWindFluidSubstepCount( 4 );
            scene->setWindSourceStrength( 8.0f );
            scene->setWindDragCoefficient( drag );
            const auto shapes = scene->getShapes();
            scene->getWorld().SetBodyLinearVelocity( shapes[0].bodyHandle, { 0.7f, -0.1f } );
            scene->getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );
        }
        explicitOff.setWindApertureAwareSamplingEnabled( false );
        experiment.setWindApertureAwareSamplingEnabled( true );
        experiment.configureWindField( 48, 28, 0.25f );
        check( !legacy.isWindApertureAwareSamplingEnabled() &&
            !explicitOff.isWindApertureAwareSamplingEnabled() &&
            experiment.isWindApertureAwareSamplingEnabled(),
            "opt-in aperture sampler must retain selection on resize, with default disabled" );

        float largestSampleDifference = 0.0f;
        for( int frame = 0; frame < 12; ++frame )
        {
            legacy.step( dt, 8 );
            explicitOff.step( dt, 8 );
            experiment.step( dt, 8 );

            check( legacy.getWindCouplingSteps() == experiment.getWindCouplingSteps() &&
                legacy.getWindCouplingSteps() == 4 &&
                !legacy.getWindField()->getCutCellPressure().coupledSolveFailed &&
                !experiment.getWindField()->getCutCellPressure().coupledSolveFailed &&
                std::isfinite( experiment.getWindField()->getFlowAudit( dt / 4.0f ).continuityRms ),
                "aperture-sampling trial must preserve cadence and finish finite pressure solves" );
            const vec2 samplePoint = experiment.getWorld().GetBodyTransform(
                experiment.getShapes()[0].bodyHandle ).position;
            const vec2 legacyFlow = legacy.getWindField()->sampleVelocity( samplePoint );
            const vec2 trialFlow = experiment.getWindField()->sampleVelocity( samplePoint );
            largestSampleDifference = std::max( largestSampleDifference, Length( legacyFlow - trialFlow ) );
            for( std::size_t i = 0; i < legacy.getShapes().size(); ++i )
            {
                const bodyId base = legacy.getShapes()[i].bodyHandle;
                const bodyId neutral = explicitOff.getShapes()[i].bodyHandle;
                const bodyId optIn = experiment.getShapes()[i].bodyHandle;
                check( near( legacy.getWorld().GetBodyLinearVelocity( base ),
                        explicitOff.getWorld().GetBodyLinearVelocity( neutral ), 1.0e-5f ) &&
                    near( legacy.getWorld().GetBodyTransform( base ).position,
                        explicitOff.getWorld().GetBodyTransform( neutral ).position, 1.0e-5f ),
                    "explicitly disabled sampler must remain identical to legacy World" );
                check( IsFinite( experiment.getWorld().GetBodyLinearVelocity( optIn ) ) &&
                    IsFinite( experiment.getWorld().GetBodyTransform( optIn ).position ) &&
                    std::isfinite( experiment.getWorld().GetBodyAngularVelocity( optIn ) ),
                    "experimental Cut-cell sample must not produce nonfinite rigid motion" );
            }
        }
        check( std::isfinite( largestSampleDifference ),
            "opt-in sampler must produce finite velocity samples near moving bodies" );
    }
}

void checkWindCouplingAudit()
{
    constexpr float dt = 1.0f / 60.0f;
    for( float h : { 0.5f, 0.25f } )
    {
        rigidBodyDemo reference{ demoKind::windFlow };
        rigidBodyDemo audited{ demoKind::windFlow };
        for( rigidBodyDemo* scene : { &reference, &audited } )
        {
            scene->configureWindField( static_cast<int>( 12.0f / h ),
                static_cast<int>( 7.0f / h ), h );
            scene->setWindCutCellProjectionEnabled( true );
            scene->setWindCutCellCoupledProjectionEnabled( true );
            scene->setWindDisjointAnalyticEnabled( true );
            scene->setWindFluidSubstepCount( 4 );
            scene->setWindSourceStrength( 8.0f );
            scene->setWindDragCoefficient( 3.0f );
            const auto shapes = scene->getShapes();
            scene->getWorld().SetBodyLinearVelocity( shapes[0].bodyHandle, { 0.7f, -0.10f } );
            scene->getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );
        }
        audited.setWindCouplingAuditEnabled( true );
        check( !reference.isWindCouplingAuditEnabled() && audited.isWindCouplingAuditEnabled(),
            "coupling spike diagnostics must be opt-in" );

        int nonzeroDragFrames = 0;
        for( int frame = 0; frame < 12; ++frame )
        {
            reference.step( dt, 8 );
            audited.step( dt, 8 );
            const auto& audit = audited.getWindCouplingAudit();
            const auto& projection = audited.getWindField()->getCutCellPressure();
            check( audited.getWindCouplingSteps() == 4 &&
                reference.getWindCouplingSteps() == 4 &&
                audit.sampledFluidSteps == 4 && audit.nonFiniteFluidFaces == 0 &&
                audit.peakFluidSubstep >= 0 && audit.peakFluidSubstep < 4 &&
                close( audit.peakBeforeDrag.maxFluidFaceSpeed, audit.maxFaceSpeedBeforeDrag ) &&
                audit.peakBeforeDrag.maxFaceOpenFraction > 0.0f &&
                audit.peakBeforeDrag.maxFaceOpenFraction <= 1.0f &&
                std::isfinite( audit.peakSampledNormalVelocity ) &&
                std::isfinite( audit.peakAdjacentCellNormalSpeed ) &&
                audit.peakAdjacentCellFluidFraction >= 0.0f &&
                audit.peakAdjacentCellFluidFraction <= 1.0f &&
                std::abs( audit.peakSampledNormalVelocity -
                    audit.peakBeforeDrag.maxFaceVelocity ) <
                    1.0e-2f * ( 1.0f + audit.peakBeforeDrag.maxFluidFaceSpeed ) &&
                std::isfinite( audit.maxContinuityBeforeDrag ) &&
                std::isfinite( audit.maxContinuityAfterDrag ) &&
                std::isfinite( audit.maxContinuityIncreaseFromDrag ) &&
                std::isfinite( audit.maxFaceSpeedBeforeDrag ) &&
                std::isfinite( audit.maxFaceSpeedAfterDrag ) &&
                !projection.coupledSolveFailed,
                "opt-in audit must collect finite pre/post drag states without disrupting coupled solve" );
            if( LengthSquared( audited.getWindDragBodyImpulse() ) > 1.0e-12f )
                ++nonzeroDragFrames;
            check( near( audited.getWindDragBodyImpulse(), reference.getWindDragBodyImpulse(), 1.0e-5f ) &&
                near( audited.getWindDragFluidImpulse(), reference.getWindDragFluidImpulse(), 1.0e-5f ) &&
                near( audited.getWindField()->getFluidMomentum(),
                    reference.getWindField()->getFluidMomentum(), 1.0e-5f ),
                "observing drag must not change exchanged impulses or fluid momentum" );
            for( std::size_t i = 0; i < reference.getShapes().size(); ++i )
            {
                const bodyId a = reference.getShapes()[i].bodyHandle;
                const bodyId b = audited.getShapes()[i].bodyHandle;
                check( near( reference.getWorld().GetBodyTransform( a ).position,
                        audited.getWorld().GetBodyTransform( b ).position, 1.0e-5f ) &&
                    near( reference.getWorld().GetBodyLinearVelocity( a ),
                        audited.getWorld().GetBodyLinearVelocity( b ), 1.0e-5f ),
                    "coupling audit must not change rigid motion or integration cadence" );
            }
        }
        check( nonzeroDragFrames > 0,
            "driven two-way scene must exercise nonzero drag exchange during audit" );
    }

    // With drag disabled, both samples read exactly the same current field.
    rigidBodyDemo noDrag{ demoKind::windFlow };
    noDrag.setWindCouplingAuditEnabled( true );
    noDrag.setWindDragCoefficient( 0.0f );
    noDrag.setWindFluidSubstepCount( 4 );
    for( int i = 0; i < 6; ++i )
    {
        noDrag.step( dt, 8 );
        const auto& audit = noDrag.getWindCouplingAudit();
        check( audit.sampledFluidSteps == 4 &&
            audit.maxContinuityBeforeDrag == audit.maxContinuityAfterDrag &&
            audit.maxFaceSpeedBeforeDrag == audit.maxFaceSpeedAfterDrag &&
            audit.maxContinuityIncreaseFromDrag == 0.0f &&
            audit.maxFaceSpeedIncreaseFromDrag == 0.0f &&
            LengthSquared( noDrag.getWindDragBodyImpulse() ) == 0.0f &&
            LengthSquared( noDrag.getWindDragFluidImpulse() ) == 0.0f,
            "zero-drag audit must report identical before/after flow states" );
    }

    // The diagnostic flag alone must retain the requested Fluid cadence even
    // when the old global coupling switch would normally force one interval.
    rigidBodyDemo uncoupled{ demoKind::windFlow };
    uncoupled.setWindCouplingAuditEnabled( true );
    uncoupled.setWindFluidSubstepCount( 4 );
    uncoupled.setWindObstacleCouplingEnabled( false );
    uncoupled.setWindDragCoefficient( 0.0f );
    uncoupled.step( dt, 8 );
    check( uncoupled.getWindCouplingSteps() == 4 &&
        uncoupled.getWindCouplingAudit().sampledFluidSteps == 4 &&
        !uncoupled.getWindField()->getPressureMomentum().usedCoupledProjection,
        "uncoupled audit must retain fixed Fluid cadence without enabling two-way forces" );
    uncoupled.setWindCouplingAuditEnabled( false );
    uncoupled.step( dt, 8 );
    check( uncoupled.getWindCouplingSteps() == 1 &&
        uncoupled.getWindCouplingAudit().sampledFluidSteps == 0,
        "turning off diagnostic mode must restore existing single-exchange scheduling" );
}

void checkWorldPressureWarmStartDifferential()
{
    constexpr float dt = 1.0f / 60.0f;
    // The benchmark showed that a continuously forced World/Fluid interleave
    // reuses pressure effectively. Unlike fast-changing isolated geometry,
    // this test must exercise the *accepted* PCG initial-guess path.
    for( float h : { 0.5f, 0.25f } )
    {
        rigidBodyDemo cold{ demoKind::windFlow };
        rigidBodyDemo warm{ demoKind::windFlow };
        for( rigidBodyDemo* scene : { &cold, &warm } )
        {
            scene->configureWindField( static_cast<int>( 12.0f / h ),
                static_cast<int>( 7.0f / h ), h );
            scene->setWindCutCellProjectionEnabled( true );
            scene->setWindCutCellCoupledProjectionEnabled( true );
            scene->setWindDisjointAnalyticEnabled( true );
            scene->setWindFluidSubstepCount( 4 );
            scene->setWindStepProfilingEnabled( true );
            scene->setWindSourceStrength( 8.0f );
            scene->setWindDragCoefficient( 3.0f );
            scene->seedWindTracer();
            const auto shapes = scene->getShapes();
            scene->getWorld().SetBodyLinearVelocity( shapes[0].bodyHandle, { 0.7f, -0.10f } );
            scene->getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.5f );
        }
        warm.setWindPressureWarmStartEnabled( true );
        check( !cold.getWindField()->isCoupledPressureWarmStartEnabled() &&
            warm.getWindField()->isCoupledPressureWarmStartEnabled(),
            "World pressure warm start must be explicitly enabled" );

        int coldIterations = 0, warmIterations = 0, warmAcceptances = 0;
        for( int frame = 0; frame < 12; ++frame )
        {
            cold.step( dt, 8 );
            warm.step( dt, 8 );
            const auto& a = cold.getWindField()->getCutCellPressure();
            const auto& b = warm.getWindField()->getCutCellPressure();
            const auto& coldStats = cold.getWindCoupledTiming();
            const auto& warmStats = warm.getWindCoupledTiming();
            check( a.usedCoupledProjection && b.usedCoupledProjection &&
                !a.coupledSolveFailed && !b.coupledSolveFailed &&
                a.coupledStatus == fluidCoupledPressureStatus2::converged &&
                b.coupledStatus == fluidCoupledPressureStatus2::converged &&
                a.coupledRelativeResidual < 1.1e-5f &&
                b.coupledRelativeResidual < 1.1e-5f &&
                coldStats.warmStartAttempts == 0 &&
                warmStats.warmStartAttempts == 4 &&
                warmStats.warmStartAccepted <= warmStats.warmStartAttempts,
                "accepted pressure warm start must preserve four coupled solves per World frame" );

            coldIterations += coldStats.matrixProducts;
            warmIterations += warmStats.matrixProducts;
            warmAcceptances += warmStats.warmStartAccepted;

            const float continuityScale = 1.0f + std::abs( a.predictedContinuityRms );
            check( std::abs( a.predictedContinuityRms - b.predictedContinuityRms ) <
                1.0e-2f * continuityScale,
                "World warm start must preserve Cut-cell continuity accuracy" );

            const auto baseForces = cold.getWindPressureForces();
            const auto newForces = warm.getWindPressureForces();
            check( baseForces.size() == newForces.size(),
                "World pressure feedback arrays must stay aligned under warm start" );
            for( std::size_t i = 0; i < baseForces.size(); ++i )
            {
                const float scale = 1.0f + Length( baseForces[i].force );
                check( near( baseForces[i].force, newForces[i].force,
                        2.0e-2f * scale ) &&
                    std::abs( baseForces[i].torque - newForces[i].torque ) <
                        2.0e-2f * scale,
                    "World warm start must preserve body pressure force and torque" );
            }
            for( std::size_t i = 0; i < cold.getShapes().size(); ++i )
            {
                const bodyId lhs = cold.getShapes()[i].bodyHandle;
                const bodyId rhs = warm.getShapes()[i].bodyHandle;
                check( near( cold.getWorld().GetBodyTransform( lhs ).position,
                        warm.getWorld().GetBodyTransform( rhs ).position, 2.0e-2f ) &&
                    near( cold.getWorld().GetBodyLinearVelocity( lhs ),
                        warm.getWorld().GetBodyLinearVelocity( rhs ), 2.0e-2f ),
                    "World warm start must preserve rigid-body trajectories" );
            }
        }
        check( warmAcceptances > 0 && warmIterations < coldIterations,
            "World scene must exercise accepted warm starts with less total PCG work" );
    }
}

void checkCachedIc0WorldDifferential()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo reference{ demoKind::windFlow };
    rigidBodyDemo cached{ demoKind::windFlow };
    for( rigidBodyDemo* scene : { &reference, &cached } )
    {
        scene->setWindCutCellProjectionEnabled( true );
        scene->setWindCutCellCoupledProjectionEnabled( true );
        scene->setWindDisjointAnalyticEnabled( true );
        scene->setWindFluidSubstepCount( 4 );
        scene->setWindSourceEnabled( false );
        scene->setWindDragCoefficient( 1.5f );
        const auto shapes = scene->getShapes();
        scene->getWorld().SetBodyLinearVelocity( shapes[0].bodyHandle, { 0.7f, -0.20f } );
        scene->getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.4f );
        scene->addWindVortex();
    }
    reference.setWindCachedIc0Enabled( false );
    check( !reference.isWindCachedIc0Enabled() && cached.isWindCachedIc0Enabled(),
        "World wind must allow reproducible original versus cached IC0 comparison" );

    for( int frame = 0; frame < 6; ++frame )
    {
        reference.step( dt, 8 );
        cached.step( dt, 8 );
        const auto& a = reference.getWindField()->getCutCellPressure();
        const auto& b = cached.getWindField()->getCutCellPressure();
        check( a.usedCoupledProjection && b.usedCoupledProjection &&
            !a.coupledSolveFailed && !b.coupledSolveFailed &&
            a.coupledStatus == fluidCoupledPressureStatus2::converged &&
            b.coupledStatus == fluidCoupledPressureStatus2::converged &&
            std::abs( a.coupledIterations - b.coupledIterations ) <= 3,
            "World interleave must converge for both original and cached IC0" );
        for( std::size_t i = 0; i < reference.getShapes().size(); ++i )
        {
            const bodyId lhs = reference.getShapes()[i].bodyHandle;
            const bodyId rhs = cached.getShapes()[i].bodyHandle;
            const vec2 p1 = reference.getWorld().GetBodyTransform( lhs ).position;
            const vec2 p2 = cached.getWorld().GetBodyTransform( rhs ).position;
            check( near( p1, p2, 2.0e-2f ) &&
                near( reference.getWorld().GetBodyLinearVelocity( lhs ),
                    cached.getWorld().GetBodyLinearVelocity( rhs ), 2.0e-2f ),
                "cached IC0 must preserve World body motion with synchronized fluid steps" );
        }
    }
    reference.configureWindField( 28, 16, 0.5f );
    cached.configureWindField( 28, 16, 0.5f );
    check( !reference.isWindCachedIc0Enabled() && cached.isWindCachedIc0Enabled(),
        "wind field resize must preserve the selected IC0 benchmark mode" );
}

void checkActiveIc0WorldDifferential()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo dense{ demoKind::windFlow };
    rigidBodyDemo active{ demoKind::windFlow };
    for( rigidBodyDemo* scene : { &dense, &active } )
    {
        scene->setWindCutCellProjectionEnabled( true );
        scene->setWindCutCellCoupledProjectionEnabled( true );
        scene->setWindDisjointAnalyticEnabled( true );
        scene->setWindFluidSubstepCount( 4 );
        scene->setWindSourceEnabled( false );
        scene->setWindDragCoefficient( 1.5f );
        const auto shapes = scene->getShapes();
        scene->getWorld().SetBodyLinearVelocity( shapes[0].bodyHandle, { 0.7f, -0.20f } );
        scene->getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.4f );
        scene->addWindVortex();
    }
    dense.setWindActiveIc0SweepEnabled( false );
    check( !dense.isWindActiveIc0SweepEnabled() && active.isWindActiveIc0SweepEnabled(),
        "active IC0 World sweep must be independently selectable" );

    for( int frame = 0; frame < 6; ++frame )
    {
        dense.step( dt, 8 );
        active.step( dt, 8 );
        const auto& a = dense.getWindField()->getCutCellPressure();
        const auto& b = active.getWindField()->getCutCellPressure();
        check( a.usedCoupledProjection && b.usedCoupledProjection &&
            !a.coupledSolveFailed && !b.coupledSolveFailed &&
            a.coupledStatus == fluidCoupledPressureStatus2::converged &&
            b.coupledStatus == fluidCoupledPressureStatus2::converged &&
            a.coupledIterations == b.coupledIterations,
            "World active sweep and dense IC0 must have matching coupled convergence" );
        for( std::size_t i = 0; i < dense.getShapes().size(); ++i )
        {
            const bodyId lhs = dense.getShapes()[i].bodyHandle;
            const bodyId rhs = active.getShapes()[i].bodyHandle;
            check( near( dense.getWorld().GetBodyTransform( lhs ).position,
                    active.getWorld().GetBodyTransform( rhs ).position, 1.0e-3f ) &&
                near( dense.getWorld().GetBodyLinearVelocity( lhs ),
                    active.getWorld().GetBodyLinearVelocity( rhs ), 1.0e-3f ),
                "active-cell IC0 must preserve synchronized World body motion" );
        }
    }
    dense.configureWindField( 28, 16, 0.5f );
    active.configureWindField( 28, 16, 0.5f );
    check( !dense.isWindActiveIc0SweepEnabled() && active.isWindActiveIc0SweepEnabled(),
        "wind grid resize must preserve the active-cell IC0 sweep choice" );
}

void checkCachedSchurEdgesWorldDifferential()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo reference{ demoKind::windFlow };
    rigidBodyDemo cached{ demoKind::windFlow };
    for( rigidBodyDemo* scene : { &reference, &cached } )
    {
        scene->setWindCutCellProjectionEnabled( true );
        scene->setWindCutCellCoupledProjectionEnabled( true );
        scene->setWindDisjointAnalyticEnabled( true );
        scene->setWindFluidSubstepCount( 4 );
        scene->setWindStepProfilingEnabled( true );
        scene->setWindSourceEnabled( false );
        scene->setWindDragCoefficient( 1.5f );
        const auto shapes = scene->getShapes();
        scene->getWorld().SetBodyLinearVelocity( shapes[0].bodyHandle, { 0.7f, -0.20f } );
        scene->getWorld().SetBodyAngularVelocity( shapes[1].bodyHandle, 0.4f );
        scene->addWindVortex();
    }
    reference.setWindCachedSchurEdgesEnabled( false );
    check( !reference.isWindCachedSchurEdgesEnabled() &&
        cached.isWindCachedSchurEdgesEnabled(),
        "wind demo must allow the coupled Schur edge cache to be disabled" );

    for( int frame = 0; frame < 6; ++frame )
    {
        reference.step( dt, 8 );
        cached.step( dt, 8 );
        const auto& a = reference.getWindField()->getCutCellPressure();
        const auto& b = cached.getWindField()->getCutCellPressure();
        check( a.usedCoupledProjection && b.usedCoupledProjection &&
            !a.coupledSolveFailed && !b.coupledSolveFailed &&
            a.coupledStatus == fluidCoupledPressureStatus2::converged &&
            b.coupledStatus == fluidCoupledPressureStatus2::converged &&
            a.coupledIterations == b.coupledIterations &&
            a.coupledTiming.cachedLaplacianEdges == 0 &&
            b.coupledTiming.cachedLaplacianEdges > 0 &&
            std::abs( a.coupledRelativeResidual - b.coupledRelativeResidual ) < 1.0e-6f,
            "World Schur edge cache must preserve convergence and report cached face count" );
        check( reference.getWindCoupledTiming().cachedLaplacianEdges == 0 &&
            cached.getWindCoupledTiming().cachedLaplacianEdges >=
                b.coupledTiming.cachedLaplacianEdges,
            "synchronized World profiling must sum cached faces across fluid substeps" );
        for( std::size_t i = 0; i < reference.getShapes().size(); ++i )
        {
            const bodyId lhs = reference.getShapes()[i].bodyHandle;
            const bodyId rhs = cached.getShapes()[i].bodyHandle;
            check( near( reference.getWorld().GetBodyTransform( lhs ).position,
                    cached.getWorld().GetBodyTransform( rhs ).position, 1.0e-3f ) &&
                near( reference.getWorld().GetBodyLinearVelocity( lhs ),
                    cached.getWorld().GetBodyLinearVelocity( rhs ), 1.0e-3f ),
                "cached Schur face operator must preserve World body motion" );
        }
    }
    reference.configureWindField( 28, 16, 0.5f );
    cached.configureWindField( 28, 16, 0.5f );
    check( !reference.isWindCachedSchurEdgesEnabled() &&
        cached.isWindCachedSchurEdgesEnabled(),
        "wind grid resize must retain Schur edge cache comparison option" );
}

void checkWorldWindStepProfiling()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    check( !scene.isWindStepProfilingEnabled() &&
        !scene.getWindField()->isStepTimingEnabled(),
        "sandbox stage timing must be opt-in" );
    scene.setWindStepProfilingEnabled( true );
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindFluidSubstepCount( 4 );
    scene.getWorld().SetBodyLinearVelocity( scene.getShapes()[0].bodyHandle, { 0.6f, -0.1f } );
    scene.step( dt, 8 );
    const fluidStepTiming2 first = scene.getWindFieldTiming();
    check( scene.getWindCouplingSteps() == 4 && scene.getWindField()->isStepTimingEnabled() &&
        first.total > 0.0 && first.geometry > 0.0 && first.advection > 0.0 &&
        first.pressure > 0.0 && first.accounted() <= first.total + 0.2 &&
        scene.getWindFluidMilliseconds() >= 0.0f && scene.getWindWorldMilliseconds() >= 0.0f,
        "World wind profile must accumulate field phase time across four fluid intervals" );
    const fluidCoupledPressureTiming2 firstPcg = scene.getWindCoupledTiming();
    check( firstPcg.total > 0.0 && firstPcg.matrixProducts > 0 &&
        firstPcg.preconditionerApplications > 0 &&
        firstPcg.accounted() <= firstPcg.total + 0.2 &&
        firstPcg.total <= first.pressure + 0.2 &&
        firstPcg.ic0Forward >= 0.0 && firstPcg.ic0Backward >= 0.0 &&
        firstPcg.ic0Forward + firstPcg.ic0Backward <= firstPcg.preconditioner + 0.2,
        "World wind profile must account for every coupled PCG solve and IC0 sweep timing" );
    scene.step( dt, 8 );
    const fluidStepTiming2 second = scene.getWindFieldTiming();
    check( second.total > 0.0 && second.swept > 0.0 &&
        second.accounted() <= second.total + 0.2,
        "successive synchronized World steps must report a fresh swept phase timing" );
    check( scene.getWindCoupledTiming().total > 0.0 &&
        scene.getWindCoupledTiming().total <= second.pressure + 0.2,
        "Schur solve phase times must reset per frame rather than accumulate across frames" );

    scene.configureWindField( 28, 16, 0.5f );
    check( scene.isWindStepProfilingEnabled() && scene.getWindField()->isStepTimingEnabled() &&
        scene.getWindFieldTiming().total == 0.0 &&
        scene.getWindCoupledTiming().total == 0.0 &&
        scene.getWindField()->isCutCellPressureTimingEnabled(),
        "resizing the wind grid must preserve the profiling choice without stale timings" );
    scene.step( dt, 8 );
    check( scene.getWindFieldTiming().total > 0.0,
        "resized wind field must continue reporting phase timing" );
    scene.setWindStepProfilingEnabled( false );
    scene.step( dt, 8 );
    check( scene.getWindFieldTiming().total == 0.0 &&
        scene.getWindCoupledTiming().total == 0.0 &&
        scene.getWindCoupledTiming().ic0Forward == 0.0 &&
        scene.getWindCoupledTiming().ic0Backward == 0.0 &&
        scene.getWindField()->getStepTiming().total == 0.0 &&
        !scene.getWindField()->isCutCellPressureTimingEnabled(),
        "disabled sandbox profiling must not publish old observations" );
}

void checkFluidCoupling()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    const fluidField2* field = scene.getWindField();
    check( field != nullptr && field->getWidth() == 24 && field->getHeight() == 14,
        "wind demo must own its separate MAC grid" );
    check( scene.getShapes().size() == 3, "wind demo must show three dynamic rigid bodies" );
    check( std::holds_alternative<circle2>( scene.getWorld().GetShape( scene.getShapes()[0].shapeHandle ).geometry ) &&
        std::holds_alternative<capsule2>( scene.getWorld().GetShape( scene.getShapes()[1].shapeHandle ).geometry ) &&
        std::holds_alternative<polygon2>( scene.getWorld().GetShape( scene.getShapes()[2].shapeHandle ).geometry ),
        "wind demo must exercise circle, capsule and convex polygon together" );
    check( near( scene.getWorld().GetGravity(), {} ), "wind trial should initially be zero gravity" );

    const bodyId first = scene.getShapes()[0].bodyHandle;
    const vec2 initial = scene.getWorld().GetBodyTransform( first ).position;

    // 단방향 coupling이 없으면 유체만 움직이고 Body는 정지해야 함.
    scene.setWindObstacleCouplingEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    for( int i = 0; i < 120; ++i ) scene.step( 1.0f / 60.0f, 4 );
    check( near( scene.getWorld().GetBodyTransform( first ).position, initial ),
        "zero drag rate must decouple the wind field from body motion" );

    const vec2 flow = field->sampleVelocity( { -3.0f, 0.0f } );
    check( IsFinite( flow ) && Length( flow ) > 1.0e-4f,
        "wind source must create an observable finite velocity field" );
    check( std::isfinite( field->getDivergenceRms() ) && field->getDivergenceRms() < 0.25f,
        "running wind demo must keep pressure projection residual bounded" );

    scene.resetWindFlow();
    check( field->getDivergenceRms() == 0.0f && near( scene.getWorld().GetBodyTransform( first ).position, initial ),
        "wind reset must clear velocity and restore body transforms" );

    // 소스가 꺼진 빈 유체는 강체를 가속하지 않아야 함.
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 3.0f );
    for( int i = 0; i < 60; ++i ) scene.step( 1.0f / 60.0f, 4 );
    check( near( scene.getWorld().GetBodyTransform( first ).position, initial ),
        "empty field with source disabled must leave bodies at rest" );

    scene.setWindSourceEnabled( true );
    scene.setWindSourceStrength( 25.0f );
    for( int i = 0; i < 300; ++i ) scene.step( 1.0f / 60.0f, 4 );
    check( Length( scene.getWorld().GetBodyTransform( first ).position - initial ) > 0.01f,
        "fluid drag must move a body in the flow" );

    scene.resetWindFlow();
    check( near( scene.getWorld().GetBodyTransform( first ).position, initial ) &&
        near( scene.getWorld().GetBodyLinearVelocity( first ), {} ) &&
        scene.getWindSourceStrength() == 25.0f && scene.getWindDragCoefficient() == 3.0f,
        "wind reset must restore motion while preserving edited parameters" );
}

void checkRotatedSandboxObstacles()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    const auto visuals = scene.getShapes();

    // 회전한 Capsule과 Polygon이 World에서 유체 solver까지 전달되는지 확인.
    for( std::size_t i : { 1u, 2u } )
    {
        const bodyId body = visuals[i].bodyHandle;
        const transform2 pose = scene.getWorld().GetBodyTransform( body );
        scene.getWorld().SetBodyTransform( body, { pose.position, rot2::FromRadians( 0.5f * 3.14159265358979323846f ) } );
    }
    scene.step( 1.0f / 60.0f, 4 );
    check( scene.getWindField()->getSolidCellCount() >= 5,
        "rotated capsule/polygon body geometry must enter the fluid solid occupancy" );
    for( const fluidObstacleForce2& pressure : scene.getWindPressureForces() )
        check( IsFinite( pressure.force ) && std::isfinite( pressure.torque ),
            "mixed rotated shapes in resting fluid must have finite pressure feedback" );
}

void checkMovingSolidReaction()
{
    rigidBodyDemo disabled{ demoKind::windFlow };
    rigidBodyDemo enabled{ demoKind::windFlow };
    const bodyId bodyDisabled = disabled.getShapes()[0].bodyHandle;
    const bodyId bodyEnabled = enabled.getShapes()[0].bodyHandle;

    for( rigidBodyDemo* scene : { &disabled, &enabled } )
    {
        scene->setWindSourceEnabled( false );
        scene->setWindDragCoefficient( 0.0f );
        scene->getWorld().SetBodyLinearVelocity( scene->getShapes()[0].bodyHandle, { 2.0f, 0.0f } );
    }
    disabled.setWindObstacleCouplingEnabled( false );
    enabled.setWindObstacleCouplingEnabled( true );
    // 이 검사는 1회 유체 압력 반력의 방향에 대한 기존 oracle이다.
    // Substep 평균 힘은 별도의 동기화 회귀에서 검증한다.
    enabled.setWindSynchronizedCouplingEnabled( false );

    disabled.step( 1.0f / 60.0f, 4 );
    enabled.step( 1.0f / 60.0f, 4 );

    check( disabled.getWindField()->getSolidCellCount() == 0 &&
        enabled.getWindField()->getSolidCellCount() > 0,
        "bidirectional toggle must control solid occupancy" );
    check( near( disabled.getWorld().GetBodyLinearVelocity( bodyDisabled ), { 2.0f, 0.0f } ),
        "disabled feedback must preserve momentum in an empty fluid" );
    check( enabled.getWindPressureForces()[0].force.x < -0.001f &&
        enabled.getWorld().GetBodyLinearVelocity( bodyEnabled ).x <
            disabled.getWorld().GetBodyLinearVelocity( bodyDisabled ).x,
        "a moving solid must stir fluid and receive opposite pressure force" );

    const vec2 wake = enabled.getWindField()->sampleVelocity( { -1.0f, 0.0f } );
    check( IsFinite( wake ) && Length( wake ) > 0.0001f,
        "moving solid must change the velocity field without a wind source" );

    enabled.resetWindFlow();
    check( enabled.getWindField()->getSolidCellCount() == 0 &&
        near( enabled.getWindPressureForces()[0].force, {} ) &&
        near( enabled.getWorld().GetBodyLinearVelocity( bodyEnabled ), {} ),
        "wind reset must clear pressure feedback and solid mask" );
    check( enabled.isWindObstacleCouplingEnabled(), "wind reset must preserve coupling setting" );
}

void checkDragMomentumBudget()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 10.0f );
    const bodyId body = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( body, { 2.0f, 0.0f } );

    // 압력 feedback은 별도 물리 과정. 여기서는 drag와 그 국소 유체 impulse 쌍만 검증함.
    scene.step( 1.0f / 60.0f, 4 );
    const vec2 bodyImpulse = scene.getWindDragBodyImpulse();
    const vec2 fluidImpulse = scene.getWindDragFluidImpulse();
    check( Length( bodyImpulse ) > 0.0001f && Length( fluidImpulse ) > 0.0001f,
        "moving body must exchange nonzero drag impulse with the fluid" );
    check( Length( scene.getWindDragExchangeResidual() ) < 1.0e-5f &&
        Length( bodyImpulse + fluidImpulse ) < 1.0e-5f,
        "bidirectional drag must deliver equal-and-opposite impulses" );

    scene.resetWindFlow();
    check( LengthSquared( scene.getWindDragBodyImpulse() ) == 0.0f &&
        LengthSquared( scene.getWindDragFluidImpulse() ) == 0.0f,
        "reset must clear per-step drag exchange diagnostics" );

    scene.setWindObstacleCouplingEnabled( false );
    scene.getWorld().SetBodyLinearVelocity( body, { 2.0f, 0.0f } );
    scene.step( 1.0f / 60.0f, 4 );
    check( Length( scene.getWindDragBodyImpulse() ) > 0.0001f &&
        LengthSquared( scene.getWindDragFluidImpulse() ) == 0.0f,
        "one-way mode must not deposit the reaction impulse into fluid" );

    scene.setWindObstacleCouplingEnabled( true );
    scene.resetWindFlow();
    scene.setWindDragCoefficient( 0.0f );
    scene.getWorld().SetBodyLinearVelocity( body, { 2.0f, 0.0f } );
    scene.step( 1.0f / 60.0f, 4 );
    check( LengthSquared( scene.getWindDragBodyImpulse() ) == 0.0f &&
        LengthSquared( scene.getWindDragFluidImpulse() ) == 0.0f &&
        Length( scene.getWindPressureForces()[0].force ) > 0.01f,
        "pressure reaction must remain independent when drag coefficient is zero" );
}

void checkAngularDragResponse()
{
    // 원천 유동이 없는 곳에서 순수 회전하는 강체도 표면 상대속도 때문에 저항 토크를 받는다.
    // 압력 feedback을 비활성화해 Drag만 독립 비교한다.
    rigidBodyDemo noDrag{ demoKind::windFlow };
    rigidBodyDemo withDrag{ demoKind::windFlow };
    noDrag.setWindSourceEnabled( false );
    withDrag.setWindSourceEnabled( false );
    noDrag.setWindObstacleCouplingEnabled( false );
    withDrag.setWindObstacleCouplingEnabled( false );
    noDrag.setWindDragCoefficient( 0.0f );
    withDrag.setWindDragCoefficient( 8.0f );

    const bodyId referenceBody = noDrag.getShapes()[2].bodyHandle;
    const bodyId dragBody = withDrag.getShapes()[2].bodyHandle;
    noDrag.getWorld().SetBodyAngularVelocity( referenceBody, 3.0f );
    withDrag.getWorld().SetBodyAngularVelocity( dragBody, 3.0f );
    constexpr float dt = 1.0f / 60.0f;
    noDrag.step( dt, 4 );
    withDrag.step( dt, 4 );

    const float referenceSpin = noDrag.getWorld().GetBodyAngularVelocity( referenceBody );
    const float dampedSpin = withDrag.getWorld().GetBodyAngularVelocity( dragBody );
    check( referenceSpin > 0.0f && dampedSpin > 0.0f &&
        dampedSpin < referenceSpin - 1.0e-3f,
        "surface-relative drag must slow pure rigid-body rotation without wind" );
    check( near( withDrag.getWorld().GetBodyLinearVelocity( dragBody ), {}, 1.0e-4f ),
        "symmetric surface drag on a spinning box must not create center-of-mass drift" );
    check( LengthSquared( withDrag.getWindDragFluidImpulse() ) == 0.0f,
        "one-way rotational drag must not modify fluid momentum" );

    // 유체 반작용이 활성화되어도 각 표면점의 선형 충격량은 정확히 상쇄되어야 한다.
    rigidBodyDemo coupled{ demoKind::windFlow };
    coupled.setWindSourceEnabled( false );
    coupled.setWindDragCoefficient( 8.0f );
    coupled.getWorld().SetBodyAngularVelocity( coupled.getShapes()[2].bodyHandle, 3.0f );
    coupled.step( dt, 4 );
    check( Length( coupled.getWindDragExchangeResidual() ) < 1.0e-5f &&
        IsFinite( coupled.getWindDragBodyImpulse() ) &&
        IsFinite( coupled.getWindDragFluidImpulse() ),
        "four-point drag must preserve equal-and-opposite linear impulse exchange" );
}

void checkSynchronizedWindCoupling()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindObstacleCouplingEnabled( true );
    check( scene.isWindSynchronizedCouplingEnabled() && scene.getWindFluidSubstepCount() == 1,
        "mass-aware wind coupling must default to one expensive Fluid step independent of World substeps" );
    scene.setWindFluidSubstepCount( 4 );

    const bodyId moving = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( moving, { 2.0f, 0.0f } );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 4 &&
        scene.getWindField()->getPressureMomentum().usedCoupledProjection &&
        !scene.getWindField()->getPressureMomentum().coupledSolveFailed &&
        scene.getWindField()->getSolidCellCount() > 0 &&
        IsFinite( scene.getWorld().GetBodyLinearVelocity( moving ) ) &&
        IsFinite( scene.getWindPressureForces()[0].force ),
        "bidirectional wind must advance fluid and rigid geometry on matching substep clocks" );

    scene.resetWindFlow();
    scene.setWindSynchronizedCouplingEnabled( false );
    scene.getWorld().SetBodyLinearVelocity( moving, { 2.0f, 0.0f } );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        IsFinite( scene.getWindPressureForces()[0].force ),
        "legacy one-exchange wind mode must remain reproducible" );

    scene.resetWindFlow();
    scene.setWindSynchronizedCouplingEnabled( true );
    scene.setWindObstacleCouplingEnabled( false );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        scene.getWindField()->getSolidCellCount() == 0,
        "one-way wind should keep legacy World substeps without fluid obstacle feedback" );

    scene.resetWindFlow();
    scene.setWindObstacleCouplingEnabled( true );
    scene.setWindCutCellProjectionEnabled( true );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        !scene.getWindField()->getPressureMomentum().usedCoupledProjection,
        "Cut-cell pressure must not use unsupported explicit synchronized subcycling" );

    scene.resetWindFlow();
    scene.setWindCutCellProjectionEnabled( false );
    scene.getWorld().SetBodyLinearVelocity( moving, { 2.0f, 0.0f } );
    scene.step( dt, 1 );
    check( scene.getWindCouplingSteps() == 1 &&
        scene.getWindField()->getPressureMomentum().usedCoupledProjection,
        "mass-aware pressure must remain active even with one rigid substep" );
    scene.resetWindFlow();
    scene.getWorld().SetBodyLinearVelocity( moving, { 2.0f, 0.0f } );
    float maxSpeed = 0.0f;
    for( int i = 0; i < 120; ++i )
    {
        scene.step( dt, 4 );
        const vec2 velocity = scene.getWorld().GetBodyLinearVelocity( moving );
        const fluidPressureMomentum2& pressure = scene.getWindField()->getPressureMomentum();
        check( pressure.usedCoupledProjection &&
            pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
            !pressure.coupledSolveFailed &&
            pressure.coupledRelativeResidual <= 1.0001e-5f &&
            IsFinite( velocity ) && IsFinite( scene.getWindPressureForces()[0].force ) &&
            std::isfinite( scene.getWindPressureForces()[0].torque ),
            "synchronized moving obstacle must keep finite velocity and pressure feedback over 120 frames" );
        maxSpeed = std::max( maxSpeed, Length( velocity ) );
        if( maxSpeed >= 20.0f )
        {
            const vec2 pressure = scene.getWindPressureForces()[0].force;
            std::fprintf( stderr, "wind coupled instability: frame=%d speed=%.5f body=(%.5f,%.5f) pressure=(%.5f,%.5f)\n",
                i, maxSpeed, velocity.x, velocity.y, pressure.x, pressure.y );
            check( false, "synchronized pressure feedback must not accelerate a released body without wind source or drag without bound" );
        }
    }
}

void checkPressurePredictedWorldVelocity()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );

    // 중력/Drag/Contact가 없는 단일 World Step에서 실제 속도 증분은
    // Fluid가 반환한 압력 반력 dt*F/m, dt*tau/I와 같아야 한다.
    const bodyId body = scene.getShapes()[2].bodyHandle;
    auto& world = scene.getWorld();
    transform2 pose = world.GetBodyTransform( body );
    pose.rotation = rot2::FromRadians( 0.4f );
    world.SetBodyTransform( body, pose );
    world.SetBodyLinearVelocity( body, { 1.1f, -0.7f } );
    world.SetBodyAngularVelocity( body, 0.8f );

    const vec2 initialLinear = world.GetBodyLinearVelocity( body );
    const float initialAngular = world.GetBodyAngularVelocity( body );
    const float inverseMass = 1.0f / world.GetBodyMass( body );
    const float inverseInertia = 1.0f / world.GetBodyRotationalInertia( body );
    scene.step( dt, 1 );

    const fluidPressureMomentum2& pressure = scene.getWindField()->getPressureMomentum();
    check( pressure.usedCoupledProjection &&
        pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
        pressure.synchronizedBoundaryFaces > 0,
        "World and Fluid test requires converged pressure and synchronized MAC boundaries" );
    const fluidObstacleForce2& force = scene.getWindPressureForces()[2];
    const vec2 expectedLinear = initialLinear + ( dt * inverseMass ) * force.force;
    const float expectedAngular = initialAngular + dt * inverseInertia * force.torque;
    check( near( world.GetBodyLinearVelocity( body ), expectedLinear, 1.0e-3f ) &&
        close( world.GetBodyAngularVelocity( body ), expectedAngular, 1.0e-3f ),
        "without drag, gravity or contacts, World velocity must match Fluid pressure-predicted body velocity" );
    double actualRigidEnergy = 0.0;
    for( const visualShape& visual : scene.getShapes() )
    {
        const double mass = world.GetBodyMass( visual.bodyHandle );
        const double inertia = world.GetBodyRotationalInertia( visual.bodyHandle );
        const vec2 linearVelocity = world.GetBodyLinearVelocity( visual.bodyHandle );
        const double angularVelocity = world.GetBodyAngularVelocity( visual.bodyHandle );
        actualRigidEnergy += 0.5 * mass * Dot( linearVelocity, linearVelocity ) +
            0.5 * inertia * angularVelocity * angularVelocity;
    }
    check( std::abs( actualRigidEnergy - pressure.energy.rigidAfter ) <
        1.0e-3 * ( 1.0 + actualRigidEnergy ) &&
        std::abs( scene.getWindField()->getFluidKineticEnergy() - pressure.energy.fluidAfter ) <
            1.0e-6 * ( 1.0 + pressure.energy.fluidAfter ),
        "isolated World pressure response must match predicted body energy and measured free-fluid face energy" );
}

void checkNoSourceWindEnergyEnvelope()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindFluidSubstepCount( 1 );
    const bodyId moving = scene.getShapes()[0].bodyHandle;
    auto& world = scene.getWorld();
    world.SetBodyLinearVelocity( moving, { 1.8f, 0.4f } );
    world.SetBodyAngularVelocity( moving, 0.7f );

    const auto energy = [&]
    {
        double result = scene.getWindField()->getFluidKineticEnergy();
        for( const visualShape& shape : scene.getShapes() )
        {
            const vec2 v = world.GetBodyLinearVelocity( shape.bodyHandle );
            const double w = world.GetBodyAngularVelocity( shape.bodyHandle );
            result += 0.5 * world.GetBodyMass( shape.bodyHandle ) * Dot( v, v ) +
                0.5 * world.GetBodyRotationalInertia( shape.bodyHandle ) * w * w;
        }
        return result;
    };

    const double initial = energy();
    double peak = initial;
    double cumulativePositivePressure = 0.0;
    float peakMomentumResidual = 0.0f;
    for( int frame = 0; frame < 60; ++frame )
    {
        scene.step( dt, 8 );
        const fluidPressureMomentum2& pressure = scene.getWindField()->getPressureMomentum();
        check( pressure.usedCoupledProjection && !pressure.coupledSolveFailed,
            "no-source World8/Fluid1 energy trial requires converged coupled pressure" );
        const float magnitude = Length( pressure.fluid ) + Length( pressure.solid ) + Length( pressure.wall );
        peakMomentumResidual = std::max( peakMomentumResidual,
            Length( pressure.residual() ) / ( 1.0f + magnitude ) );
        peak = std::max( peak, energy() );
        cumulativePositivePressure += std::max( 0.0, pressure.energy.totalDelta() );
        check( std::isfinite( peak ) && std::isfinite( cumulativePositivePressure ) &&
            peak < 10.0 * ( 1.0 + initial ) &&
            peakMomentumResidual < 5.0e-4f,
            "source-free moving World must avoid explosive total kinetic energy and bound discrete pressure momentum error" );
    }
    std::printf( "Wind moving geometry energy: initial=%.6f peak=%.6f final=%.6f positivePressureDelta=%.6f maxMomentumError=%.3g\n",
        initial, peak, energy(), cumulativePositivePressure, peakMomentumResidual );
}

void checkFluidRateAndStaticObstacles()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    auto& world = scene.getWorld();
    const bodyId dynamicBody = scene.getShapes()[0].bodyHandle;
    world.SetBodyLinearVelocity( dynamicBody, { 2.0f, 0.0f } );
    scene.step( dt, 8 );
    check( scene.getWindCouplingSteps() == 1 &&
        scene.getWindField()->getPressureMomentum().usedCoupledProjection,
        "World substeps must not force additional Fluid projection when fluid rate is one" );
    for( int frame = 0; frame < 120; ++frame )
    {
        scene.step( dt, 8 );
        const vec2 velocity = world.GetBodyLinearVelocity( dynamicBody );
        const fluidPressureMomentum2& pressure = scene.getWindField()->getPressureMomentum();
        check( scene.getWindCouplingSteps() == 1 &&
            pressure.usedCoupledProjection &&
            pressure.coupledStatus == fluidCoupledPressureStatus2::converged &&
            !pressure.coupledSolveFailed &&
            pressure.coupledRelativeResidual <= 1.0001e-5f &&
            IsFinite( velocity ) && Length( velocity ) < 20.0f,
            "one Fluid step with eight World substeps must avoid added-mass speed runaway" );
    }
    scene.resetWindFlow();
    scene.setWindFluidSubstepCount( 2 );
    world.SetBodyLinearVelocity( dynamicBody, { 2.0f, 0.0f } );
    scene.step( dt, 8 );
    check( scene.getWindCouplingSteps() == 2 &&
        scene.getWindField()->getPressureMomentum().usedCoupledProjection,
        "two Fluid exchanges must group eight World substeps without disabling coupled pressure" );
    scene.setWindFluidSubstepCount( 32 );
    scene.step( dt, 3 );
    check( scene.getWindCouplingSteps() == 3,
        "Fluid exchange count must never exceed the World substeps in one external step" );
    scene.setWindFluidSubstepCount( 1 );
    scene.resetWindFlow();

    const shapeId fixedBox = scene.addWindStaticBox( { -3.0f, 0.0f }, { 0.45f, 0.4f } );
    const shapeId fixedCircle = scene.addWindStaticCircle( { 1.5f, -1.0f }, 0.65f );
    check( world.IsValid( fixedBox ) && world.IsValid( fixedCircle ) &&
        scene.getWindStaticObstacleCount() == 2 && scene.getShapes().size() == 5,
        "Static wind objects must be created as real World shapes" );
    const bodyId fixedBody = scene.getShapes()[3].bodyHandle;
    check( world.GetBody( fixedBody ).type == bodyType::Static,
        "Static wind obstacle must use World::Static for collision and mass behaviour" );
    const vec2 fixedPosition = world.GetBodyTransform( fixedBody ).position;
    scene.step( dt, 8 );
    check( scene.getWindField()->isSolidCell( 6, 7 ) &&
        scene.getWindField()->isSolidCell( 15, 5 ) &&
        near( world.GetBodyTransform( fixedBody ).position, fixedPosition ),
        "both static box and circle must block fluid and remain unmoved" );
    scene.setWindObstacleCouplingEnabled( false );
    scene.step( dt, 8 );
    check( scene.getWindField()->isSolidCell( 6, 7 ) &&
        scene.getWindField()->isSolidCell( 15, 5 ) &&
        scene.getWindField()->getSolidCellCount() > 0,
        "static walls must remain fluid barriers when dynamic bidirectional coupling is disabled" );
    scene.setWindObstacleCouplingEnabled( true );

    scene.setWindConservativeAdvectionEnabled( true );
    scene.setWindFluidSubstepCount( 2 );
    scene.configureWindField( 32, 20, 0.5f );
    check( scene.getWindField()->getWidth() == 32 &&
        scene.getWindField()->getHeight() == 20 &&
        close( scene.getWindField()->getCellSize(), 0.5f ) &&
        scene.getWindField()->isConservativeAdvectionEnabled() &&
        scene.getWindStaticObstacleCount() == 2 &&
        near( world.GetBodyTransform( fixedBody ).position, fixedPosition ),
        "resizing Fluid Field must retain static World geometry and mode selection" );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 2 &&
        scene.getWindField()->isSolidCell( 10, 10 ),
        "resized Fluid grid must sample existing Static obstacles in new coordinates" );

    check( scene.removeWindStaticObstacle( fixedBox ) &&
        !world.IsValid( fixedBox ) && scene.getWindStaticObstacleCount() == 1,
        "removing Static obstacle must remove its collision shape from World" );
    scene.step( dt, 8 );
    check( !scene.getWindField()->isSolidCell( 10, 10 ) &&
        world.IsValid( fixedCircle ) &&
        !scene.removeWindStaticObstacle( scene.getShapes()[0].shapeHandle ),
        "removed static geometry must disappear from Fluid while dynamic objects remain protected" );

    scene.configureWindField( 16, 12, 0.25f );
    scene.setWindSourceEnabled( true );
    scene.step( dt, 8 );
    check( scene.getWindField()->getWidth() == 16 &&
        scene.getWindField()->getHeight() == 12 &&
        IsFinite( scene.getWindField()->sampleVelocity( { -1.0f, 0.0f } ) ),
        "small resized Fluid domain must accept wind sources without nonfinite results" );
}

void checkPressureLedgerInSandbox()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindSynchronizedCouplingEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    const bodyId body = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( body, { 2.0f, 0.0f } );
    scene.step( 1.0f / 60.0f, 4 );

    const fluidPressureMomentum2& budget = scene.getWindField()->getPressureMomentum();
    const fluidStepMomentum2& stages = scene.getWindField()->getStepMomentum();
    const float scale = 1.0f + Length( budget.fluid ) + Length( budget.solid ) + Length( budget.wall );
    check( Length( budget.solid ) > 1.0e-5f &&
        Length( budget.residual() ) < 5.0e-4f * scale,
        "wind demo moving rigid body must close the fluid/solid/wall pressure momentum budget" );

    vec2 forceSum{};
    for( const fluidObstacleForce2& pressure : scene.getWindPressureForces() )
        forceSum += pressure.force;
    check( Length( budget.solid - forceSum * ( 1.0f / 60.0f ) ) < 1.0e-4f * scale &&
        Length( stages.pressureDelta() - budget.fluid ) < 1.0e-4f * scale,
        "Sandbox applied pressure reaction must match core pressure ledger" );

    scene.resetWindFlow();
    check( LengthSquared( scene.getWindField()->getPressureMomentum().residual() ) == 0.0f &&
        LengthSquared( scene.getWindField()->getStepMomentum().afterPressure ) == 0.0f,
        "wind reset must clear core pressure diagnostics" );

    scene.setWindObstacleCouplingEnabled( false );
    scene.step( 1.0f / 60.0f, 4 );
    check( LengthSquared( scene.getWindField()->getPressureMomentum().solid ) == 0.0f,
        "one-way mode must not report rigid pressure reaction" );
}

void checkExperimentalCutCellWorldCoupling()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindFluidSubstepCount( 2 );
    check( scene.isWindCutCellCoupledProjectionEnabled() &&
        scene.isWindSynchronizedCouplingEnabled() &&
        scene.isWindObstacleCouplingEnabled(),
        "experimental cut-cell coupling should require an explicit opt-in and existing bidirectional clock" );

    const bodyId moving = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( moving, { 1.1f, 0.4f } );
    scene.getWorld().SetBodyAngularVelocity( moving, 0.8f );
    scene.addWindVortex();
    scene.step( dt, 4 );
    const fluidCutCellPressure2 first = scene.getWindField()->getCutCellPressure();
    check( scene.getWindCouplingSteps() == 2 &&
        first.usedCoupledProjection && !first.coupledSolveFailed &&
        first.coupledStatus == fluidCoupledPressureStatus2::converged &&
        first.synchronizedBoundaryFaces > 0 &&
        std::isfinite( first.predictedContinuityRms ) &&
        IsFinite( scene.getWorld().GetBodyLinearVelocity( moving ) ),
        "opt-in weighted Schur pressure must receive World mass/inertia and advance substep-aligned rigid bodies" );

    scene.step( dt, 4 );
    const fluidCutCellPressure2 moved = scene.getWindField()->getCutCellPressure();
    const fluidSweptFaceFlux2 swept = scene.getWindField()->getSweptFaceFlux();
    check( scene.getWindCouplingSteps() == 2 && swept.usedSweptFlux &&
        moved.hasVolumeHistory && std::isfinite( moved.volumeRate ) &&
        std::isfinite( moved.predictedContinuityRms ) &&
        std::isfinite( moved.predictedSolidVolumeL1 ) &&
        IsFinite( scene.getWorld().GetBodyLinearVelocity( moving ) ),
        "next cut-cell fluid interval must use the actual World geometry swept history without reapplying predicted displacement" );

    scene.setWindAdaptiveSweptSamplingEnabled( true );
    scene.setWindCutCellSampleCount( 8 );
    scene.configureWindField( 32, 18, 0.5f );
    check( scene.isWindCutCellProjectionEnabled() &&
        scene.isWindCutCellCoupledProjectionEnabled() &&
        scene.getWindCutCellSampleCount() == 8 &&
        scene.isWindAdaptiveSweptSamplingEnabled() &&
        scene.getWindField()->getCutCellPressure().activeCells == 0,
        "field resize must keep explicit experimental coupling while resetting old geometry history" );

    scene.setWindSynchronizedCouplingEnabled( false );
    scene.addWindVortex();
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        !scene.getWindField()->getCutCellPressure().usedCoupledProjection,
        "disabling synchronized coupling must keep the weighted legacy solver despite the opt-in" );

    scene.resetWindFlow();
    scene.setWindSynchronizedCouplingEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( false );
    scene.addWindVortex();
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        !scene.getWindField()->getCutCellPressure().usedCoupledProjection &&
        !scene.getWindField()->getCutCellPressure().coupledSolveFailed,
        "cut-cell mass coupling disabled must preserve the old single-exchange weighted projection" );

    scene.resetWindFlow();
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindObstacleCouplingEnabled( false );
    scene.step( dt, 4 );
    check( scene.getWindCouplingSteps() == 1 &&
        !scene.getWindField()->getCutCellPressure().usedCoupledProjection,
        "one-way field with optional static obstacles must not silently insert dynamic-body inverse masses" );
}

void checkSandboxAnalyticCapsuleMode()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindCapsuleOnlyFluidEnabled( true );
    scene.setWindAnalyticCapsuleEnabled( true );
    scene.setWindFluidSubstepCount( 2 );
    const bodyId capsule = scene.getShapes()[1].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( capsule, { 0.7f, -0.25f } );
    scene.getWorld().SetBodyAngularVelocity( capsule, 0.7f );
    scene.addWindVortex();
    scene.step( dt, 4 );

    const fluidCutCellPressure2& first = scene.getWindField()->getCutCellPressure();
    check( scene.getWindCouplingSteps() == 2 &&
        scene.isWindCapsuleOnlyFluidEnabled() &&
        !scene.isWindCircleOnlyFluidEnabled() && !scene.isWindPolygonOnlyFluidEnabled() &&
        scene.isWindAnalyticCapsuleEnabled() &&
        first.usedCoupledProjection && !first.coupledSolveFailed &&
        std::isfinite( first.predictedContinuityRms ) &&
        IsFinite( scene.getWorld().GetBodyLinearVelocity( capsule ) ),
        "isolated analytical Capsule wind trial must run World mass-aware pressure coupling" );

    scene.step( dt, 4 );
    check( scene.getWindField()->getSweptFaceFlux().usedSweptFlux &&
        scene.getWindField()->getCutCellPressure().hasVolumeHistory &&
        std::isfinite( scene.getWindField()->getSweptFaceFlux().geometryMismatch ),
        "moving rotating wind Capsule must produce swept geometry history" );

    scene.configureWindField( 24, 14, 0.5f );
    check( scene.isWindAnalyticCapsuleEnabled() && scene.isWindCapsuleOnlyFluidEnabled() &&
        !scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "wind field resize must preserve analytic Capsule mode and reset volume history" );
    scene.setWindCapsuleOnlyFluidEnabled( false );
    scene.setWindAnalyticCapsuleEnabled( false );
    scene.setWindFluidSubstepCount( 1 );
    scene.step( dt, 4 );
    check( scene.getWindField()->getCutCellPressure().activeCells > 0 &&
        !scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "returning Capsule-only Wind to mixed obstacles must rebuild first sampled geometry snapshot" );
}

void checkSandboxDisjointAnalyticMultiShapeMode()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindDisjointAnalyticEnabled( true );
    scene.setWindFluidSubstepCount( 2 );
    scene.addWindVortex();
    scene.step( dt, 4 );
    const fluidField2* field = scene.getWindField();
    const fluidCutCellTransition2& first = field->getCutCellTransition();
    const fluidCutCellPressure2& pressure = field->getCutCellPressure();
    check( scene.isWindDisjointAnalyticEnabled() && scene.getWindCouplingSteps() == 2 &&
        first.usedDisjointAnalyticGeometry && first.disjointAnalyticShapes == 3 &&
        pressure.usedCoupledProjection && !pressure.coupledSolveFailed &&
        std::isfinite( pressure.predictedContinuityRms ),
        "wind mixed Circle/Capsule/Box must use disjoint analytic multi-owner pressure coupling" );

    for( const visualShape& visual : scene.getShapes() )
        check( IsFinite( scene.getWorld().GetBodyLinearVelocity( visual.bodyHandle ) ),
            "disjoint multi-shape wind pressure response must leave all World body velocities finite" );

    scene.step( dt, 4 );
    check( field->getCutCellTransition().usedDisjointAnalyticGeometry &&
        field->getSweptFaceFlux().usedSweptFlux &&
        field->getCutCellPressure().hasVolumeHistory &&
        std::isfinite( field->getSweptFaceFlux().geometryMismatch ),
        "moving multiple disjoint World shapes must retain swept flux and pressure geometry continuity" );

    scene.setWindRoundedBoxEnabled( true );
    scene.setWindFluidSubstepCount( 1 );
    scene.step( dt, 4 );
    check( field->getCutCellTransition().usedDisjointAnalyticGeometry &&
        field->getCutCellTransition().disjointAnalyticShapes == 3 &&
        !field->getCutCellPressure().hasVolumeHistory,
        "changing the rounded World shape must invalidate old cut-cell history and preserve mixed analytics" );

    scene.configureWindField( 24, 14, 0.5f );
    check( scene.isWindDisjointAnalyticEnabled() &&
        !scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "resizing fluid field must retain mixed analytic selection without old geometry history" );

    scene.setWindDisjointAnalyticEnabled( false );
    scene.step( dt, 4 );
    check( !scene.getWindField()->getCutCellTransition().usedDisjointAnalyticGeometry &&
        !scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "switching Sandbox multi-analytic off must return to sampled union without fake volume change" );
}

void checkSandboxAnalyticRoundedPolygonMode()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindPolygonOnlyFluidEnabled( true );
    scene.setWindRoundedBoxEnabled( true );
    scene.setWindAnalyticRoundedPolygonEnabled( true );
    scene.setWindFluidSubstepCount( 2 );
    const visualShape& box = scene.getShapes()[2];
    const auto* rounded = std::get_if<polygon2>(
        &scene.getWorld().GetShape( box.shapeHandle ).geometry );
    check( rounded && rounded->radius > 0.0f &&
        scene.isWindRoundedBoxEnabled() && scene.isWindAnalyticRoundedPolygonEnabled(),
        "wind box geometry switch must create a real World rounded Polygon shape" );

    const bodyId body = box.bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( body, { -0.7f, 0.25f } );
    scene.getWorld().SetBodyAngularVelocity( body, 0.6f );
    scene.addWindVortex();
    scene.step( dt, 4 );
    const auto& first = scene.getWindField()->getCutCellPressure();
    check( scene.getWindCouplingSteps() == 2 &&
        first.usedCoupledProjection && !first.coupledSolveFailed &&
        std::isfinite( first.predictedContinuityRms ) &&
        IsFinite( scene.getWorld().GetBodyLinearVelocity( body ) ),
        "isolated rounded box must use World mass/inertia-aware pressure coupling" );

    scene.step( dt, 4 );
    check( scene.getWindField()->getSweptFaceFlux().usedSweptFlux &&
        scene.getWindField()->getCutCellPressure().hasVolumeHistory &&
        std::isfinite( scene.getWindField()->getSweptFaceFlux().geometryMismatch ),
        "moving rounded World box must create swept-face geometry history" );

    scene.configureWindField( 24, 14, 0.5f );
    check( scene.isWindRoundedBoxEnabled() && scene.isWindAnalyticRoundedPolygonEnabled() &&
        !scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "field resize must preserve rounded box and analytic settings without old history" );

    scene.setWindRoundedBoxEnabled( false );
    scene.setWindAnalyticRoundedPolygonEnabled( false );
    scene.setWindPolygonOnlyFluidEnabled( false );
    scene.setWindFluidSubstepCount( 1 );
    scene.step( dt, 4 );
    const auto* sharp = std::get_if<polygon2>(
        &scene.getWorld().GetShape( scene.getShapes()[2].shapeHandle ).geometry );
    check( sharp && sharp->radius == 0.0f &&
        scene.getWindField()->getCutCellPressure().activeCells > 0 &&
        !scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "restoring sharp mixed-shape wind must reset rounded geometry and old volume source" );
}

void checkSandboxAnalyticPolygonMode()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindPolygonOnlyFluidEnabled( true );
    scene.setWindAnalyticPolygonEnabled( true );
    scene.setWindFluidSubstepCount( 2 );
    const bodyId body = scene.getShapes()[2].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( body, { -0.7f, 0.25f } );
    scene.getWorld().SetBodyAngularVelocity( body, 0.6f );
    scene.addWindVortex();
    scene.step( dt, 4 );

    const fluidCutCellPressure2& first = scene.getWindField()->getCutCellPressure();
    check( scene.getWindCouplingSteps() == 2 &&
        scene.isWindPolygonOnlyFluidEnabled() && !scene.isWindCircleOnlyFluidEnabled() &&
        scene.isWindAnalyticPolygonEnabled() &&
        first.usedCoupledProjection && !first.coupledSolveFailed &&
        std::isfinite( first.predictedContinuityRms ) &&
        IsFinite( scene.getWorld().GetBodyLinearVelocity( body ) ),
        "isolated analytic polygon must receive mass-aware World pressure coupling" );

    scene.step( dt, 4 );
    check( scene.getWindField()->getSweptFaceFlux().usedSweptFlux &&
        scene.getWindField()->getCutCellPressure().hasVolumeHistory &&
        std::isfinite( scene.getWindField()->getSweptFaceFlux().geometryMismatch ),
        "moving and rotating analytic sandbox polygon must integrate swept face volume" );

    scene.configureWindField( 24, 14, 0.5f );
    check( scene.isWindAnalyticPolygonEnabled() && scene.isWindPolygonOnlyFluidEnabled() &&
        !scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "wind field resize must preserve polygon analytic mode but reset geometry history" );

    scene.setWindPolygonOnlyFluidEnabled( false );
    scene.setWindAnalyticPolygonEnabled( false );
    scene.setWindFluidSubstepCount( 1 );
    scene.step( dt, 4 );
    check( scene.getWindField()->getCutCellPressure().activeCells > 0 &&
        !scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "switching from polygon-only to mixed shapes must rebuild sampled occupancy without fake volume flux" );
}

void checkSandboxAnalyticCircleMode()
{
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindCircleOnlyFluidEnabled( true );
    scene.setWindAnalyticCircleEnabled( true );
    scene.setWindFluidSubstepCount( 2 );
    const bodyId body = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( body, { 1.2f, 0.3f } );
    scene.addWindVortex();
    scene.step( dt, 4 );

    const fluidCutCellPressure2& first = scene.getWindField()->getCutCellPressure();
    check( scene.getWindCouplingSteps() == 2 &&
        scene.isWindCircleOnlyFluidEnabled() && scene.isWindAnalyticCircleEnabled() &&
        first.usedCoupledProjection && !first.coupledSolveFailed &&
        std::isfinite( first.predictedContinuityRms ) &&
        IsFinite( scene.getWorld().GetBodyLinearVelocity( body ) ),
        "isolated analytic Circle wind sandbox must use World mass-aware Schur pressure" );
    scene.step( dt, 4 );
    check( scene.getWindField()->getSweptFaceFlux().usedSweptFlux &&
        scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "moving isolated Circle in wind sandbox must generate a new swept geometry interval" );
    scene.configureWindField( 24, 14, 0.5f );
    check( scene.isWindAnalyticCircleEnabled() && scene.isWindCircleOnlyFluidEnabled() &&
        !scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "wind field resize must preserve circle geometry option without retaining swept history" );
    scene.setWindAnalyticCircleEnabled( false );
    scene.setWindCircleOnlyFluidEnabled( false );
    // 두 번의 Fluid 구간을 쓰면 두 번째 구간은 이미 유효한 현재 모드 이력임.
    // 전환 직후 첫 스냅샷의 무이력 보장을 독립적으로 검사하려고 1회만 진행.
    scene.setWindFluidSubstepCount( 1 );
    scene.step( dt, 4 );
    check( scene.getWindField()->getCutCellPressure().activeCells > 0 &&
        !scene.getWindField()->getCutCellPressure().hasVolumeHistory,
        "wind sandbox must restore mixed-shape sampled cut-cells after disabling analytic mode" );
}

void checkMovingCutCellSweptVolumeLedger()
{
    // World의 실제 pose 이동만 다음 Fluid 구간의 기하 ΔV를 만든다.
    // 압력 단계에서 예측한 추가 boundary swept volume은 진단용이며,
    // 실제 snapshot ΔV에 다시 더해져서는 안 된다.
    constexpr float dt = 1.0f / 60.0f;
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindCutCellCoupledProjectionEnabled( true );
    scene.setWindSynchronizedCouplingEnabled( true );
    scene.setWindFluidSubstepCount( 2 );
    const bodyId circle = scene.getShapes()[0].bodyHandle;
    const bodyId box = scene.getShapes()[2].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( circle, { 1.25f, -0.3f } );
    scene.getWorld().SetBodyAngularVelocity( box, 0.9f );
    scene.addWindVortex();

    int sweptFrames = 0;
    int coupledFrames = 0;
    float largestPredictedVolume = 0.0f;
    for( int frame = 0; frame < 36; ++frame )
    {
        scene.step( dt, 4 );
        const fluidField2* field = scene.getWindField();
        const fluidCutCellPressure2& pressure = field->getCutCellPressure();
        const fluidCutCellTransition2& transition = field->getCutCellTransition();
        const fluidSweptFaceFlux2& swept = field->getSweptFaceFlux();
        check( scene.getWindCouplingSteps() == 2 && pressure.hasVolumeHistory &&
            std::isfinite( pressure.volumeRate ) && std::isfinite( transition.volumeDelta() ) &&
            std::isfinite( pressure.predictedContinuityRms ) &&
            std::isfinite( pressure.predictedSolidVolumeL1 ) &&
            IsFinite( scene.getWorld().GetBodyLinearVelocity( circle ) ) &&
            std::isfinite( scene.getWorld().GetBodyAngularVelocity( box ) ),
            "successive World/Fluid intervals must have finite cut-cell occupancy and pressure diagnostics" );
        // 두 Fluid 구간이 각각 dt/2 동안 진행되므로, 보고된 기하 변화율은
        // 마지막 구간의 실제 snapshot 점유 변화만 포함해야 한다.
        check( std::abs( pressure.volumeRate * ( 0.5f * dt ) - transition.volumeDelta() ) < 2.0e-4f,
            "cut-cell geometric volume rate must match the actual snapshot delta without predicted swept-volume double counting" );
        if( swept.usedSweptFlux ) ++sweptFrames;
        if( pressure.usedCoupledProjection )
        {
            ++coupledFrames;
            largestPredictedVolume = std::max( largestPredictedVolume, pressure.predictedSolidVolumeL1 );
        }
    }
    check( sweptFrames >= 30 && coupledFrames >= 1 && largestPredictedVolume > 0.0f,
        "long-running cut-cell World test must exercise both real swept faces and pressure-predicted boundary motion" );
}

void checkExperimentalCutCellProjection()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindSourceEnabled( false );
    scene.setWindDragCoefficient( 0.0f );
    scene.setWindCutCellProjectionEnabled( true );
    check( scene.isWindCutCellProjectionEnabled(), "wind sandbox must enable weighted cut-cell pressure" );

    const bodyId body = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyLinearVelocity( body, { 1.5f, 0.0f } );
    scene.step( 1.0f / 60.0f, 4 );
    const fluidCutCellPressure2& stats = scene.getWindField()->getCutCellPressure();
    check( stats.activeCells > 0 && std::isfinite( stats.divergenceBefore ) &&
        std::isfinite( stats.divergenceAfter ) && std::isfinite( stats.sourceImbalance ) &&
        LengthSquared( scene.getWindField()->getPressureMomentum().residual() ) == 0.0f,
        "cut-cell sandbox must execute weighted projection without using legacy pressure ledger" );

    scene.resetWindFlow();
    check( scene.isWindCutCellProjectionEnabled() &&
        scene.getWindField()->getCutCellPressure().activeCells == 0,
        "reset must clear weighted pressure diagnostics without disabling experiment" );
    scene.setWindCutCellProjectionEnabled( false );
    scene.step( 1.0f / 60.0f, 4 );
    check( !scene.isWindCutCellProjectionEnabled() &&
        scene.getWindField()->getCutCellPressure().activeCells == 0,
        "sandbox must restore binary pressure mode when experiment is disabled" );
}

void checkSandboxConservativeAdvectionTrial()
{
    rigidBodyDemo scene{ demoKind::windFlow };
    scene.setWindConservativeAdvectionEnabled( true );
    scene.setWindCutCellProjectionEnabled( false );
    scene.beginWindAdvectionTrial();
    check( scene.isWindConservativeAdvectionEnabled() &&
        !scene.isWindSourceEnabled() && !scene.isWindObstacleCouplingEnabled() &&
        scene.getWindDragCoefficient() == 0.0f,
        "advection trial must isolate a repeatable vortex from source, drag and rigid coupling" );

    const fluidField2* field = scene.getWindField();
    scene.seedWindTracer();
    const float tracerMass = field->getTracerMass();
    check( close( tracerMass, 2.0f ), "sandbox must seed a reproducible tracer patch" );
    const vec2 seeded = field->sampleVelocity( { 1.5f, 0.0f } );
    check( IsFinite( seeded ) && Length( seeded ) > 0.05f,
        "advection trial must seed observable fluid circulation before the first step" );

    for( int i = 0; i < 50; ++i )
    {
        scene.step( 1.0f / 60.0f, 4 );
        const fluidAdvectionStats2& adv = field->getAdvectionStats();
        check( adv.conservative && adv.substeps >= 1 && !adv.cflLimited &&
            IsFinite( adv.before ) && IsFinite( adv.after ) &&
            Length( adv.delta() ) < 3.0e-4f * ( 1.0f + Length( adv.before ) ) &&
            std::isfinite( field->getDivergenceRms() ),
            "sandbox isolated vortex must preserve transport face momentum without pressure contamination" );
        check( close( field->getTracerMass(), tracerMass, 1.0e-4f ) &&
            std::abs( field->getTracerStats().delta() ) < 1.0e-4f,
            "sandbox vortex must conserve the total tracer mass on the closed grid" );
    }

    scene.setWindConservativeAdvectionEnabled( false );
    scene.beginWindAdvectionTrial();
    check( close( field->getTracerMass(), 0.0f ) && field->getTracerStats().substeps == 0,
        "sandbox advection trial reset must clear tracer state" );
    scene.step( 1.0f / 60.0f, 4 );
    check( !field->getAdvectionStats().conservative && field->getAdvectionStats().substeps == 1 &&
        IsFinite( field->getAdvectionStats().after ),
        "same sandbox preset must run with the legacy semi-Lagrangian method" );

    scene.setWindConservativeAdvectionEnabled( true );
    scene.setWindCutCellProjectionEnabled( true );
    scene.setWindSweptTracerRemapEnabled( true );
    scene.setWindSweptGeometryCorrectionEnabled( true );
    scene.setWindObstacleCouplingEnabled( true );
    scene.resetWindFlow();
    check( scene.isWindSweptTracerRemapEnabled() &&
        scene.isWindSweptGeometryCorrectionEnabled() && !field->getTracerStats().usedSweptRemap,
        "sandbox reset must preserve swept tracer remap selection and clear last-step diagnostics" );
    scene.step( 1.0f / 60.0f, 4 );
    check( field->getAdvectionStats().conservative &&
        field->getCutCellPressure().activeCells > 0 &&
        !field->getTracerStats().usedSweptRemap &&
        std::isfinite( field->getCutCellPressure().pressureResidual ),
        "conservative advection must also integrate with moving-solid cut-cell wind sandbox" );

    // 장면의 실제 Circle transform을 바꾸어 연속된 강체 snapshot의 swept remap 경로를 검사.
    scene.seedWindTracer();
    const float tracerBefore = field->getTracerMass();
    const bodyId circleBody = scene.getShapes()[0].bodyHandle;
    scene.getWorld().SetBodyTransform( circleBody, { { -2.75f, -0.25f }, {} } );
    scene.step( 1.0f / 60.0f, 4 );
    check( field->getSweptFaceFlux().usedSweptFlux &&
        field->getSweptFaceFlux().geometryMismatch <=
            field->getSweptFaceFlux().rawGeometryMismatch + 1.0e-4f &&
        field->getTracerStats().usedSweptRemap &&
        close( field->getTracerMass(), tracerBefore, 1.0e-4f ) &&
        std::isfinite( field->getTracerStats().unresolvedDisplacementMass ),
        "sandbox moving Circle must run swept-guided tracer remap without mass loss" );

    scene.resetWindFlow();
    check( scene.isWindConservativeAdvectionEnabled() &&
        field->getAdvectionStats().substeps == 0,
        "wind reset must preserve algorithm choice and clear its diagnostics" );
}

} // namespace

int main()
{
    checkFluidCoupling();
    checkWindCouplingAudit();
    checkWindApertureSamplingDifferential();
    checkWorldWindStepProfiling();
    checkCachedIc0WorldDifferential();
    checkWorldPressureWarmStartDifferential();
    checkActiveIc0WorldDifferential();
    checkCachedSchurEdgesWorldDifferential();
    checkMovingSolidReaction();
    checkSynchronizedWindCoupling();
    checkPressurePredictedWorldVelocity();
    checkNoSourceWindEnergyEnvelope();
    checkFluidRateAndStaticObstacles();
    checkRotatedSandboxObstacles();
    checkDragMomentumBudget();
    checkAngularDragResponse();
    checkPressureLedgerInSandbox();
    checkExperimentalCutCellProjection();
    checkExperimentalCutCellWorldCoupling();
    checkMovingCutCellSweptVolumeLedger();
    checkSandboxAnalyticCircleMode();
    checkSandboxAnalyticPolygonMode();
    checkSandboxAnalyticRoundedPolygonMode();
    checkSandboxDisjointAnalyticMultiShapeMode();
    checkSandboxAnalyticCapsuleMode();
    checkSandboxConservativeAdvectionTrial();
}
