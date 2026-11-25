#pragma once

#include <optional>
#include <vector>

namespace zonai::fluidDetail
{

// Internal signed face transfer; positive amount is first -> second.
struct transportRepairFace
{
    int first, second;
    double amount;
};

struct transportRepairResult
{
    std::vector<transportRepairFace> faces;
    std::vector<double> ledger;
    int repaired = 0, changedFaces = 0;
    double reduction = 0.0, conservationError = 0.0;
};
// Fixed input-face order. Requires valid indices and finite signed amounts.
[[nodiscard]] std::vector<double> rebuildTransportLedger( const std::vector<double>& start,
    const std::vector<transportRepairFace>& faces );

// Bounded conservative repair of tiny negative closing-cell volumes.
// Returns no result for unsupported geometry/deficits; inputs unchanged.
[[nodiscard]] std::optional<transportRepairResult> repairClosingTransport(
    const std::vector<double>& start, const std::vector<double>& endpoint,
    const std::vector<transportRepairFace>& faces );

struct transportFaceSample
{
    bool isU;
    int index;
    double mean;
};

// Map interval signed face amounts to a sampled time panel. Mapping must
// match the interval face order. No normalization or aperture floor.
[[nodiscard]] std::vector<double> buildTransportPanelAmounts(
    const std::vector<transportRepairFace>& faces,
    const std::vector<transportFaceSample>& mapping,
    const std::vector<float>& uOpen, const std::vector<float>& vOpen,
    double weight, double weightSum );

enum class throughflowCommitMode
{
    completeOnly,
    allowPartial
};

struct throughflowAudit
{
    int passes = 0;
    double unserved = 0.0;
    bool committed = false;
};

// Fixed-order simultaneous passes. A pass may use only its start volume.
// Incoming volume becomes available in the next pass. Unserved is the sum
// of remaining face requests after the bounded schedule; it is not dropped
// silently by the caller. Complete-only mode leaves all arrays unchanged
// if requests remain. Research callers may explicitly accept partial
// transport. Invalid inputs or numerical failure throw without changing
// any array. Arrays must be distinct and aligned with cell indices.
// The signed amounts vector describes this panel; face.amount may describe
// the enclosing interval instead and is not used by this kernel.
// Internal experimental kernel; not called by the default fluid step.
[[nodiscard]] throughflowAudit consumeThroughflowPanel(
    const std::vector<transportRepairFace>& faces, const std::vector<double>& amounts,
    std::vector<double>& volume, std::vector<double>& mass, std::vector<double>& uniform,
    throughflowCommitMode mode = throughflowCommitMode::completeOnly );

}
