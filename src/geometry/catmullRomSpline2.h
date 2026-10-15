#pragma once

#include <cstddef>
#include <span>

#include "math/vec2.h"

namespace zonai
{

// 제어점 배열을 참조하는 열린 centripetal Catmull-Rom 경로. 사용 중 배열의 수명 / 주소를 유지해야 함.
struct catmullRomSpline2
{
    std::span<const vec2> points{};
};

// 2개 이상 유한한 점과 float에서 양수 / 유한한 인접 edge 길이 제곱이 필요함.
bool IsValid( const catmullRomSpline2& spline );

// 점 수만으로 구간 수를 반환함. 좌표 유효성은 IsValid로 별도 확인함.
std::size_t GetSegmentCount( const catmullRomSpline2& spline );

// 유효한 경로와 segmentIndex, 구간별 t in [0, 1]이 필요함. 잘못된 입력은 assert / Release에서 {}.
// shortcut: 평가마다 전체 경로를 검증함, 대량 샘플링을 추가할 때 사전 검증된 경로를 도입할 것.
vec2 Evaluate( const catmullRomSpline2& spline, std::size_t segmentIndex, float t );

// 같은 입력 계약으로 구간별 t에 대한 미분을 반환함. 단위 접선이나 arc-length 미분은 아님.
vec2 Derivative( const catmullRomSpline2& spline, std::size_t segmentIndex, float t );

} // namespace zonai
