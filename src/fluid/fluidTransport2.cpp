#include "fluid/fluidTransport2.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace zonai::fluidDetail
{

throughflowAudit consumeThroughflowPanel(
    const std::vector<transportRepairFace>& faces, const std::vector<double>& amounts,
    std::vector<double>& outputVolume, std::vector<double>& outputMass,
    std::vector<double>& outputUniform, throughflowCommitMode mode )
{
    constexpr int maxPasses = 8;
    const std::size_t n = outputVolume.size();
    if( faces.size() != amounts.size() || outputMass.size() != n || outputUniform.size() != n ||
        &outputVolume == &outputMass || &outputVolume == &outputUniform ||
        &outputMass == &outputUniform || &amounts == &outputVolume ||
        &amounts == &outputMass || &amounts == &outputUniform )
        throw std::invalid_argument( "fluid transport: mismatched or aliased arrays" );
    if( mode != throughflowCommitMode::completeOnly && mode != throughflowCommitMode::allowPartial )
        throw std::invalid_argument( "fluid transport: invalid commit mode" );
    for( std::size_t i = 0; i < n; ++i )
        if( !std::isfinite( outputVolume[i] ) || outputVolume[i] < 0.0 ||
            !std::isfinite( outputMass[i] ) || outputMass[i] < 0.0 ||
            !std::isfinite( outputUniform[i] ) || outputUniform[i] < 0.0 )
            throw std::invalid_argument( "fluid transport: invalid initial state" );
    std::vector<double> pending( amounts.size() ), outgoing( n );
    for( std::size_t j = 0; j < amounts.size(); ++j )
    {
        const auto& face = faces[j];
        if( face.first < 0 || face.second < 0 || face.first == face.second ||
            static_cast<std::size_t>( face.first ) >= n ||
            static_cast<std::size_t>( face.second ) >= n || !std::isfinite( amounts[j] ) )
            throw std::invalid_argument( "fluid transport: invalid face transfer" );
        pending[j] = std::abs( amounts[j] );
    }
    // Work on independent copies, including for partial research commits.
    // Numerical failure must not expose a partly executed panel.
    auto volume = outputVolume, mass = outputMass, uniform = outputUniform;
    throughflowAudit result;
    for( int pass = 0; pass < maxPasses; ++pass )
    {
        std::fill( outgoing.begin(), outgoing.end(), 0.0 );
        bool hasPending = false;
        for( std::size_t j = 0; j < amounts.size(); ++j )
        {
            if( pending[j] == 0.0 ) continue;
            hasPending = true;
            const auto& edge = faces[j];
            auto& demand = outgoing[amounts[j] > 0.0 ? edge.first : edge.second];
            demand += pending[j];
            if( !std::isfinite( demand ) )
                throw std::runtime_error( "fluid transport: outgoing volume overflow" );
        }
        if( !hasPending ) break;
        ++result.passes;
        auto remainingVolume = volume, remainingMass = mass, remainingUniform = uniform;
        std::vector<double> receivedVolume( n ), receivedMass( n ), receivedUniform( n );
        bool progress = false;
        for( std::size_t j = 0; j < amounts.size(); ++j )
        {
            if( pending[j] == 0.0 ) continue;
            const auto& edge = faces[j];
            const int donor = amounts[j] > 0.0 ? edge.first : edge.second;
            const int recipient = amounts[j] > 0.0 ? edge.second : edge.first;
            const double capacity = volume[donor];
            if( capacity <= 0.0 ) continue;
            const double requested = outgoing[donor] <= capacity ? pending[j] :
                pending[j] * ( capacity / outgoing[donor] );
            const double moved = std::min( pending[j], std::min( remainingVolume[donor], requested ) );
            if( moved <= 0.0 ) continue;
            const double ratio = moved / capacity;
            const double movedMass = std::min( remainingMass[donor], mass[donor] * ratio );
            const double movedUniform = std::min( remainingUniform[donor], uniform[donor] * ratio );
            pending[j] -= moved;
            remainingVolume[donor] -= moved; receivedVolume[recipient] += moved;
            remainingMass[donor] -= movedMass; receivedMass[recipient] += movedMass;
            remainingUniform[donor] -= movedUniform; receivedUniform[recipient] += movedUniform;
            progress = true;
        }
        for( std::size_t i = 0; i < n; ++i )
        {
            volume[i] = remainingVolume[i] + receivedVolume[i];
            mass[i] = remainingMass[i] + receivedMass[i];
            uniform[i] = remainingUniform[i] + receivedUniform[i];
            if( !std::isfinite( volume[i] ) || volume[i] < 0.0 ||
                !std::isfinite( mass[i] ) || mass[i] < 0.0 ||
                !std::isfinite( uniform[i] ) || uniform[i] < 0.0 )
                throw std::runtime_error( "fluid transport: invalid throughflow state" );
        }
        if( !progress ) break;
    }
    for( double amount : pending ) result.unserved += amount;
    if( !std::isfinite( result.unserved ) )
        throw std::runtime_error( "fluid transport: remaining volume overflow" );
    result.committed = result.unserved == 0.0 || mode == throughflowCommitMode::allowPartial;
    if( result.committed )
    {
        outputVolume.swap( volume );
        outputMass.swap( mass );
        outputUniform.swap( uniform );
    }
    return result;
}

}
