#pragma once

#include <array>
#include <vector>

#include "demo.h"
#include "dynamics/world.h"

namespace zonai::sandbox
{

enum class overlapProbe { circle, capsule, box };

struct overlapTarget
{
    bodyId body{};
    shapeId shape{};
    const char* name = "";
};

// World의 두 공간 조회를 동일한 query geometry, filter, transform에서 비교함.
class overlapShapeDemo : public demo
{
public:
    overlapShapeDemo();

    void step( float, int ) override {}
    void handleInput( const demoInput& input ) override;
    void cancelInput() override {}
    void setCollisionMatrix( const collisionMatrix& matrix ) override { world_.setCollisionMatrix( matrix ); }

    void setProbe( overlapProbe probe );
    void setQueryPosition( vec2 position );
    void setQueryRotation( float radians );
    void setSensorIncluded( bool included );
    void setCandidateAABBsVisible( bool visible ) noexcept { showCandidateAABBs_ = visible; }

    [[nodiscard]] const world& getWorld() const noexcept { return world_; }
    [[nodiscard]] const std::array<overlapTarget, 4>& getTargets() const noexcept { return targets_; }
    [[nodiscard]] const shapeGeometry& getQueryGeometry() const noexcept { return queryGeometry_; }
    [[nodiscard]] const transform2& getQueryTransform() const noexcept { return queryTransform_; }
    [[nodiscard]] aabb2 getQueryAABB() const;
    [[nodiscard]] overlapProbe getProbe() const noexcept { return probe_; }
    [[nodiscard]] float getQueryRotation() const noexcept { return queryRotation_; }
    [[nodiscard]] bool isSensorIncluded() const noexcept { return sensorIncluded_; }
    [[nodiscard]] bool areCandidateAABBsVisible() const noexcept { return showCandidateAABBs_; }
    [[nodiscard]] const std::vector<shapeId>& getCandidates() const noexcept { return candidates_; }
    [[nodiscard]] const std::vector<shapeId>& getExactOverlaps() const noexcept { return exactOverlaps_; }
    [[nodiscard]] bool isCandidate( shapeId id ) const;
    [[nodiscard]] bool isExactOverlap( shapeId id ) const;

private:
    void updateQueries();

    world world_{};
    std::array<overlapTarget, 4> targets_{};
    shapeGeometry queryGeometry_{};
    transform2 queryTransform_{};
    overlapProbe probe_ = overlapProbe::circle;
    float queryRotation_ = 0.0f;
    bool sensorIncluded_ = true;
    bool showCandidateAABBs_ = true;
    std::vector<shapeId> candidates_{};
    std::vector<shapeId> exactOverlaps_{};
};

} // namespace zonai::sandbox
