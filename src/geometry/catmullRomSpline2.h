#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "collision/aabb2.h"
#include "math/vec2.h"

namespace zonai
{

// 제어점 배열을 참조하는 centripetal Catmull-Rom 경로. 사용 중 배열의 수명 / 주소를 유지해야 함.
struct catmullRomSpline2
{
    std::span<const vec2> points{};
    bool isLoop = false;
};

// 열린 경로는 2점, 닫힌 경로는 3점 이상. 닫힌 입력은 첫 점을 끝에 반복하지 않음.
// 모든 점은 유한해야 하며 닫는 edge도 float에서 양수 / 유한한 길이 제곱이 필요함.
bool IsValid( const catmullRomSpline2& spline );

// 점 수만으로 구간 수를 반환함. 좌표 유효성은 IsValid로 별도 확인함.
std::size_t GetSegmentCount( const catmullRomSpline2& spline );

// 유효한 경로와 segmentIndex, 구간별 t in [0, 1]이 필요함. 잘못된 입력은 assert / Release에서 {}.
// shortcut: 평가마다 전체 경로를 검증함, 대량 샘플링을 추가할 때 사전 검증된 경로를 도입할 것.
vec2 Evaluate( const catmullRomSpline2& spline, std::size_t segmentIndex, float t );

// 같은 입력 계약으로 구간별 t에 대한 미분을 반환함. 단위 접선이나 arc-length 미분은 아님.
vec2 Derivative( const catmullRomSpline2& spline, std::size_t segmentIndex, float t );

// 같은 입력 계약으로 구간별 t에 대한 2차 미분. 접합점은 지정한 구간 쪽 값을 반환함.
vec2 SecondDerivative( const catmullRomSpline2& spline, std::size_t segmentIndex, float t );

// 반시계 방향이 양수인 곡률. 잘못된 입력 / 0 접선 / 비유한 결과는 nullopt. 접합점 연속성은 보장하지 않음.
std::optional<double> Curvature( const catmullRomSpline2& spline, std::size_t segmentIndex, float t );

// Bézier hull과 float 평가 반올림 여유로 전체 경로를 감쌈. 최소 AABB는 보장하지 않음.
// 잘못된 입력 / 유한한 float 경계 구성 실패는 nullopt.
std::optional<aabb2> ComputeAABB( const catmullRomSpline2& spline );

// 같은 정책으로 지정 구간만 감쌈. 잘못된 경로 / 구간 번호 / 유한한 경계 구성 실패는 nullopt.
std::optional<aabb2> ComputeAABB( const catmullRomSpline2& spline, std::size_t segmentIndex );

// 각 구간을 같은 수로 분할하며 접합점은 한 번만 넣음. 닫힌 결과는 끝에 첫 점을 반복함.
// 잘못된 입력 / 불가능한 점 수는 빈 배열.
std::vector<vec2> Sample( const catmullRomSpline2& spline, std::size_t subdivisions );

// Bézier hull이 출력 chord에서 maxError 안에 들 때까지 이분할함. 닫힌 결과는 끝에 첫 점을 반복함.
// 유효하지 않은 입력 / 깊이 12 초과는 빈 배열.
std::vector<vec2> SampleAdaptive( const catmullRomSpline2& spline, float maxError );

// 경로 전체를 같은 길이의 subdivisions개 구간으로 나눠 subdivisions + 1개 위치를 반환함.
// 양 끝점을 포함하며 닫힌 결과는 끝에 첫 점을 반복함. 간격은 chord 길이가 아닌 누적 거리 기준.
// 0 접선도 허용. 잘못된 입력 / 불가능한 점 수 / 역산 또는 위치 정밀도 부족은 빈 배열.
std::vector<vec2> SampleByDistance( const catmullRomSpline2& spline, std::size_t subdivisions, double tolerance );

struct splineSample2
{
    vec2 point{};
    vec2 tangent{};
};

// 목표 절대 길이 오차로 적분함. 잘못된 입력 / 수렴 실패는 nullopt. 지형 샘플링 오차와 별개임.
std::optional<double> GetLength( const catmullRomSpline2& spline, double tolerance );

// 한 구간의 0 <= t0 <= t1 <= 1 범위를 직접 적분함. 같은 t는 0, 0 접선도 허용함.
// tolerance는 양수인 절대 길이 오차 목표. 잘못된 입력 / 수렴 또는 정밀도 실패는 nullopt.
std::optional<double> GetLength( const catmullRomSpline2& spline, std::size_t segmentIndex, float t0, float t1, double tolerance );

// 경로 시작부터 구간별 t in [0, 1]까지의 길이. tolerance는 양수인 절대 길이 오차 목표.
// 닫힌 경로의 마지막 끝은 전체 길이. 0 접선도 허용하며 잘못된 입력 / 정밀도 부족은 nullopt.
std::optional<double> GetDistance( const catmullRomSpline2& spline, std::size_t segmentIndex, float t, double tolerance );

struct splineParameter2
{
    std::size_t segmentIndex = 0;
    float t = 0.0f;
};

// 정방향 범위의 부분 길이를 직접 합산함. 내부 접합점의 앞 t=1 / 다음 t=0은 같은 지점.
// 닫힌 시작과 한 바퀴 끝은 구분하며 자동 순환하지 않음. 같은 지점은 0, 0 접선도 허용함.
// tolerance는 전체 범위의 양수인 절대 길이 오차 목표. 잘못된 입력 / 역순 / 정밀도 또는 수렴 실패는 nullopt.
std::optional<double> GetLength( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, double tolerance );

// distance in [0, GetLength]을 구간 / t로 역산함. tolerance는 양수인 절대 길이 오차 목표이며 t 오차는 아님.
// 시작은 {0, 0}, 마지막 끝은 {마지막 구간, 1}, 정확한 내부 접합점은 앞 구간 t=1.
// 0 접선도 허용하며 잘못된 입력 / 수렴 실패 / float t 정밀도 부족은 nullopt.
std::optional<splineParameter2> GetParameterByDistance( const catmullRomSpline2& spline, double distance, double tolerance );

// 정방향 [from, to]의 시작부터 distance in [0, 범위 길이]을 역산함. 자동 순환하지 않음.
// 거리 0은 입력 from, 양수인 전체 길이는 입력 to. 내부 접합점은 앞 구간 t=1을 선택함.
// 같은 오차 / 실패 정책이며 0 길이 범위는 거리 0만 허용함. 0 접선도 허용함.
std::optional<splineParameter2> GetParameterByDistance( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, double distance, double tolerance );

// 정방향 [from, to]를 같은 길이의 subdivisions개 구간으로 나눠 양 끝을 포함한 위치를 반환함.
// 성공 시 subdivisions + 1개. 0 길이 범위는 같은 위치 반복, 0 접선도 허용하며 자동 순환하지 않음.
// 기존 균등 샘플링과 같은 오차 / 실패 정책. 잘못된 범위도 빈 배열.
std::vector<vec2> SampleByDistance( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, std::size_t subdivisions, double tolerance );

// 정방향 [from, to]의 시작부터 잰 거리로 위치와 단위 접선을 조회함. 자동 순환하지 않음.
// 기존 거리 조회와 같은 오차 / 실패 정책. 0 길이 범위는 거리 0에서 접선이 유효할 때만 성공함.
std::optional<splineSample2> EvaluateByDistance( const catmullRomSpline2& spline, splineParameter2 from, splineParameter2 to, double distance, double tolerance );

// distance in [0, GetLength]. 곡선 위 위치와 진행 방향 단위 접선. 0 접선 / 정밀도 부족도 nullopt.
std::optional<splineSample2> EvaluateByDistance( const catmullRomSpline2& spline, double distance, double tolerance );

struct splineClosestPoint2
{
    vec2 point{};
    std::size_t segmentIndex = 0;
    float t = 0.0f;
    double distanceSquared = 0.0;
};

// 최소거리의 절대오차 목표 tolerance는 유한한 양수. 잘못된 입력 / 정밀도 부족 / 탐색 한도 초과는 nullopt.
std::optional<splineClosestPoint2> ClosestPoint( const catmullRomSpline2& spline, const vec2& point, double tolerance );

// 같은 오차 / 실패 정책으로 지정 구간의 양 끝을 포함해 탐색함. 잘못된 구간 번호도 nullopt.
std::optional<splineClosestPoint2> ClosestPoint( const catmullRomSpline2& spline, std::size_t segmentIndex, const vec2& point, double tolerance );

} // namespace zonai
