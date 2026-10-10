#pragma once

#include <cstdint>
#include <memory>
#include <span>

#include "math/vec2.h"
#include "collision/filter.h"

namespace zonai::sandbox
{

class debugDraw;

enum class demoKind
{
    playground,
    distancePendulum,
    revoluteHinge,
    wheelSuspension,
    prismaticRail,
    weldPair,
    mouseJointPlayground,
    motorJointPlayground,
    motorCar
};

struct demoEntry
{
    demoKind kind{};
    const char* category = "";
    const char* name = "";
    const char* controls = "";
    vec2 cameraCenter{};
    float pixelsPerMeter = 55.0f;
};

// Host가 UI/camera 입력을 제외한 뒤 전달하는 데모 조작.
// held key와 한 frame의 press를 구분해 pause/UI focus에서 force가 남지 않게 함.
struct demoInput
{
    vec2 mousePosition{};
    bool mousePressed = false;
    bool mouseHeld = false;
    bool impulsePressed = false;
    bool left = false;
    bool right = false;
    bool jumpPressed = false;
    bool spinPressed = false;
    bool brake = false; // 자동차의 유지 입력. 다른 데모의 jumpPressed와 구분함.
};

class demo
{
public:
    virtual ~demo() = default;
    virtual void step( float timeStep, int subStepCount ) = 0;
    virtual void handleInput( const demoInput& input ) = 0;
    virtual void cancelInput() = 0;

    virtual void setCollisionMatrix( const collisionMatrix& ) {}

    // Headless model은 화면 callback을 쓰지 않음. World 소유를 공통 interface에 강제하지 않음.
    virtual void drawControls() {}

    virtual void draw( debugDraw& ) const {}
};

using demoFactory = std::unique_ptr<demo> ( * )( demoKind );
[[nodiscard]] std::span<const demoEntry> getDemoEntries() noexcept;
[[nodiscard]] const demoEntry& getDemoEntry( demoKind kind );

class demoSession
{
public:
#pragma region Lifetime
    explicit demoSession( demoFactory factory );
    void selectDemo( demoKind kind );
    void reset();
    void setCollisionMatrix( const collisionMatrix& matrix );
#pragma endregion Lifetime

#pragma region PlaybackAndInput
    void setPlaying( bool playing );
    void advance( float frameTime, int subStepCount );
    void stepOnce( int subStepCount );
    void handleInput( const demoInput& input, bool enabled );
#pragma endregion PlaybackAndInput

#pragma region Queries

    [[nodiscard]] demo& getDemo() noexcept { return *demo_; }

    [[nodiscard]] demoKind getKind() const noexcept { return kind_; }

    [[nodiscard]] bool isPlaying() const noexcept { return playing_; }

    [[nodiscard]] std::uint64_t getStepCount() const noexcept { return stepCount_; }

    [[nodiscard]] const collisionMatrix& getCollisionMatrix() const noexcept { return collisionMatrix_; }

#pragma endregion Queries

private:
    demoFactory factory_;
    collisionMatrix collisionMatrix_{};
    std::unique_ptr<demo> demo_;
    demoKind kind_ = demoKind::playground;
    bool playing_ = false;
    float accumulator_ = 0.0f;
    std::uint64_t stepCount_ = 0;
};

} // namespace zonai::sandbox
