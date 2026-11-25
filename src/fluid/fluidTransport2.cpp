#include "fluid/fluidTransport2.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace zonai::fluidDetail
{

throughflowAudit consumeThroughflowPanel(
    const std::vector<transportRepairFace>& faces, const std::vector<double>& amounts,
    std::vector<double>& volume, std::vector<double>& mass, std::vector<double>& uniform )
{
    constexpr int maxPasses = 8;
    const std::size_t n = volume.size();
    std::vector<double> pending( amounts.size() ), outgoing( n );
    if( faces.size() != amounts.size() || mass.size() != n || uniform.size() != n )
        throw std::runtime_error( "fluid transport: mismatched throughflow dimensions" );
    for( std::size_t j = 0; j < amounts.size(); ++j )
        pending[j] = std::abs( amounts[j] );
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
            outgoing[amounts[j] > 0.0 ? edge.first : edge.second] += pending[j];
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
    return result;
}

}
