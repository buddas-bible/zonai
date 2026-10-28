#pragma once

#include <array>

#include "demo.h"
#include "dynamics/world.h"

namespace zonai::sandbox
{

enum class shapeCastProbe { circle, capsule, box };

struct shapeCastTarget
{
    bodyId body{};
    shapeId shape{};
    const char* name = "";
};

// 독립 공간 조회 실험. 물리 World는 정적 조회 대상만 보관하고 애니메이션은 미리보기임.
class shapeCastDemo : public demo
{
public:
    shapeCastDemo();

    void step( float timeStep, int subStepCount ) override;
    void handleInput( const demoInput& input ) override;
    void cancelInput() override {}
    void setCollisionMatrix( const collisionMatrix& matrix ) override { world_.setCollisionMatrix( matrix ); }

    void setProbe( shapeCastProbe probe );
    void setStart( vec2 start );
    void setTarget( vec2 target );
    void setSensorIncluded( bool included );
    void setCanEncroach( bool enabled );
    void setRotationRadians( float radians );
    void setRayVisible( bool visible ) noexcept { rayVisible_ = visible; }
    void setSweepAABBVisible( bool visible ) noexcept { sweepAABBVisible_ = visible; }
    void resetPreview();

    [[nodiscard]] shapeCastProbe getProbe() const noexcept { return probe_; }
    [[nodiscard]] const world& getWorld() const noexcept { return world_; }
    [[nodiscard]] const std::array<shapeCastTarget, 4>& getTargets() const noexcept { return targets_; }
    [[nodiscard]] const worldShapeCastInput2& getInput() const noexcept { return input_; }
    [[nodiscard]] worldShapeCastResult2 getHit() const noexcept { return hit_; }
    [[nodiscard]] float getRotationRadians() const noexcept { return rotationRadians_; }
    [[nodiscard]] transform2 getPoseAtFraction( float fraction ) const noexcept;
    [[nodiscard]] worldRayCastResult2 getRayHit() const noexcept { return rayHit_; }
    [[nodiscard]] aabb2 getSweepAABB() const noexcept;
    [[nodiscard]] std::size_t getAABBCandidateCount() const noexcept { return aabbCandidateCount_; }
    [[nodiscard]] float getPreviewFraction() const noexcept { return previewFraction_; }
    [[nodiscard]] bool isSensorIncluded() const noexcept { return sensorIncluded_; }
    [[nodiscard]] bool isRayVisible() const noexcept { return rayVisible_; }
    [[nodiscard]] bool isSweepAABBVisible() const noexcept { return sweepAABBVisible_; }

private:
    void updateHit();
    [[nodiscard]] queryFilter makeFilter() const noexcept;

    world world_{};
    std::array<shapeCastTarget, 4> targets_{};
    worldShapeCastInput2 input_{};
    shapeCastProbe probe_ = shapeCastProbe::circle;
    float rotationRadians_ = 0.0f;
    worldShapeCastResult2 hit_{};
    worldRayCastResult2 rayHit_{};
    std::size_t aabbCandidateCount_ = 0;
    float previewFraction_ = 0.0f;
    bool sensorIncluded_ = true;
    bool rayVisible_ = true;
    bool sweepAABBVisible_ = true;
};

} // namespace zonai::sandbox
