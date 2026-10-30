#pragma once

#include <span>
#include <vector>

#include "math/vec2.h"

namespace zonai
{

// Body 좌표가 아닌 유체 world 좌표. 원형 solid의 cell-centered occupancy 근사.
struct fluidCircleObstacle2
{
    vec2 center{};
    float radius = 0.0f;
    vec2 linearVelocity{};
    float angularVelocity = 0.0f;
};

// 2D 단위 깊이, 유체 밀도 1kg/m³ 가정의 압력 반력. force(N), torque(N·m).
struct fluidObstacleForce2
{
    vec2 force{};
    float torque = 0.0f;
};

// 균일 MAC(staggered) grid: x속도는 수직 face, y속도는 수평 face에 저장.
// 내부 원형 solid의 경계 face는 no-through로 처리하고 유체 압력 반력을 집계함.
class fluidField2
{
public:
#pragma region Lifetime
    // 너비와 높이는 cell 개수. 최소 2, 최대 256. origin은 좌하단 경계.
    fluidField2( int width, int height, float cellSize, vec2 origin = {} );
#pragma endregion Lifetime

#pragma region Simulation
    void clear() noexcept;

    // 지정 위치 주변 face 속도에 world-space 속도 변화(m/s)를 더함.
    // 소스가 격자 바깥이거나 radius가 유효하지 않으면 아무 일도 하지 않음.
    void addVelocity( vec2 position, vec2 velocityDelta, float radius );

    // Semi-Lagrangian advection 후 Jacobi pressure projection.
    // no-through(경계 수직 속도 0) 조건. dt<=0 또는 비유한 값은 무시.
    void step( float timeStep, int pressureIterations = 80 );

    // 매 Step 제공한 원의 위치/속도로 solid occupancy를 갱신한다.
    // feedback은 obstacle 개수와 정확히 같은 크기여야 한다.
    // stationary 또는 이동하는 solid 경계의 법선 속도를 사용하며 pressure force를 돌려준다.
    void step( float timeStep, std::span<const fluidCircleObstacle2> obstacles,
        std::span<fluidObstacleForce2> feedback, int pressureIterations = 80 );
#pragma endregion Simulation

#pragma region Queries
    [[nodiscard]] vec2 sampleVelocity( vec2 position ) const noexcept;
    [[nodiscard]] float getDivergenceRms() const noexcept;
    [[nodiscard]] bool isSolidCell( int x, int y ) const noexcept;
    [[nodiscard]] int getSolidCellCount() const noexcept { return solidCellCount_; }
    [[nodiscard]] vec2 getCellCenter( int x, int y ) const noexcept;
    [[nodiscard]] int getWidth() const noexcept { return width_; }
    [[nodiscard]] int getHeight() const noexcept { return height_; }
    [[nodiscard]] float getCellSize() const noexcept { return cellSize_; }
    [[nodiscard]] vec2 getOrigin() const noexcept { return origin_; }
#pragma endregion Queries

private:
#pragma region Helpers
    [[nodiscard]] int cellIndex( int x, int y ) const noexcept { return y * width_ + x; }
    [[nodiscard]] int uIndex( int x, int y ) const noexcept { return y * ( width_ + 1 ) + x; }
    [[nodiscard]] int vIndex( int x, int y ) const noexcept { return y * width_ + x; }
    [[nodiscard]] vec2 sampleFrom( const std::vector<float>& u, const std::vector<float>& v, vec2 position ) const noexcept;
    static float sampleComponent( const std::vector<float>& values, int width, int height, float x, float y ) noexcept;
    void updateObstacles( std::span<const fluidCircleObstacle2> obstacles );
    void applySolidBoundaries( std::span<const fluidCircleObstacle2> obstacles );
    void project( float timeStep, int pressureIterations, std::span<const fluidCircleObstacle2> obstacles,
        std::span<fluidObstacleForce2> feedback );
#pragma endregion Helpers

#pragma region Data
    int width_ = 0;
    int height_ = 0;
    float cellSize_ = 1.0f;
    vec2 origin_{};
    std::vector<float> u_;
    std::vector<float> v_;
    std::vector<float> nextU_;
    std::vector<float> nextV_;
    std::vector<float> pressure_;
    std::vector<float> nextPressure_;
    std::vector<float> divergence_;
    std::vector<int> solidOwner_;
    int solidCellCount_ = 0;
#pragma endregion Data
};

} // namespace zonai
