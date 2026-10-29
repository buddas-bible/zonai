#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include "collision/broadphase/broadPhase.h"
#include "collision/broadphase/hashSet.h"

using namespace zonai;

namespace
{

constexpr int MEASURE_ROUNDS = 7;
volatile std::uint64_t benchmarkSink = 0;

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

struct randomSequence
{
    std::uint32_t state = 0x13579BDFu;

    std::uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }
};

enum class operation { add, remove, contains };

struct command
{
    operation type{};
    shapePairKey key = 0;
};

struct scenario
{
    const char* name = "";
    std::size_t keyCount = 0;
    std::size_t initialCount = 0;
    bool reserve = false;
    std::vector<command> commands{};
};

// 실제 BroadPhase의 MakeShapePairKey를 사용하고 0 sentinel을 입력에서 제외함.
shapePairKey makeKey( std::size_t index )
{
    return MakeShapePairKey( static_cast<std::int32_t>( 1 + index / 128 ),
        static_cast<std::int32_t>( 257 + index % 128 ) );
}

std::array<scenario, 3> makeScenarios()
{
    std::array<scenario, 3> scenarios{};

    // 생성 직후 Contact pair가 급격하게 늘어나는 경우: 양쪽 모두 성장 비용을 포함함.
    scenario& growth = scenarios[0];
    growth.name = "cold_growth";
    growth.keyCount = 4096;
    growth.commands.reserve( growth.keyCount * 3 + growth.keyCount / 2 );
    for( std::size_t i = 0; i < growth.keyCount; ++i )
        growth.commands.push_back( { operation::add, makeKey( i ) } );
    for( std::size_t i = 0; i < growth.keyCount; ++i )
    {
        growth.commands.push_back( { operation::contains, makeKey( i ) } );
        growth.commands.push_back( { operation::contains, makeKey( growth.keyCount + i ) } );
    }
    for( std::size_t i = 0; i < growth.keyCount; i += 2 )
        growth.commands.push_back( { operation::remove, makeKey( i ) } );

    // 기존 Contact를 확인하는 BroadPhase 후보 필터: hit 3/4, miss 1/4.
    scenario& lookup = scenarios[1];
    lookup.name = "steady_lookup";
    lookup.keyCount = 4096;
    lookup.initialCount = 3072;
    lookup.reserve = true;
    randomSequence random{};
    for( int i = 0; i < 24000; ++i )
    {
        const bool hit = i % 4 != 0;
        const std::size_t index = random.next() % lookup.initialCount;
        lookup.commands.push_back( { operation::contains,
            makeKey( hit ? index : lookup.keyCount + index ) } );
    }

    // Contact가 생성/제거되고 동일한 pair가 재등록되는 혼합 작업.
    scenario& churn = scenarios[2];
    churn.name = "contact_churn";
    churn.keyCount = 4096;
    churn.initialCount = 2048;
    churn.reserve = true;
    for( int i = 0; i < 32000; ++i )
    {
        const std::size_t index = random.next() % churn.keyCount;
        const std::uint32_t choice = random.next() % 10;
        const operation type = choice < 4 ? operation::add :
            choice < 7 ? operation::remove : operation::contains;
        churn.commands.push_back( { type, makeKey( index ) } );
    }

    return scenarios;
}

template <typename Set>
Set makeSet( const scenario& scene )
{
    if constexpr( std::is_same_v<Set, hashSet> )
    {
        // Custom hashSet은 절반 이상 차면 증가하므로 미리 2배를 확보함.
        return hashSet{ scene.reserve ? scene.keyCount * 2 : 32 };
    }
    else
    {
        Set set{};
        set.max_load_factor( 0.5f );
        if( scene.reserve ) set.reserve( scene.keyCount );
        return set;
    }
}

template <typename Set>
bool apply( Set& set, command input )
{
    if constexpr( std::is_same_v<Set, hashSet> )
    {
        switch( input.type )
        {
        case operation::add: return set.Add( input.key ); // 이미 있으면 true
        case operation::remove: return set.Remove( input.key );
        case operation::contains: return set.Contains( input.key );
        }
    }
    else
    {
        switch( input.type )
        {
        case operation::add: return !set.insert( input.key ).second;
        case operation::remove: return set.erase( input.key ) != 0;
        case operation::contains: return set.contains( input.key );
        }
    }
    return false;
}

template <typename Set>
std::size_t count( const Set& set )
{
    if constexpr( std::is_same_v<Set, hashSet> ) return set.GetCount();
    else return set.size();
}

template <typename Set>
void prepare( Set& set, const scenario& scene )
{
    for( std::size_t i = 0; i < scene.initialCount; ++i )
        check( !apply( set, { operation::add, makeKey( i ) } ),
            "Initial pair insertion must be unique" );
}

void verify( const scenario& scene )
{
    auto custom = makeSet<hashSet>( scene );
    auto reference = makeSet<std::unordered_set<shapePairKey>>( scene );
    prepare( custom, scene );
    prepare( reference, scene );

    for( const command& input : scene.commands )
    {
        check( input.key != 0, "Shape pair key must never be the empty-slot sentinel" );
        check( apply( custom, input ) == apply( reference, input ),
            "Custom/std pair set operation result mismatch" );
        check( count( custom ) == count( reference ), "Custom/std pair count mismatch" );
    }

    for( std::size_t i = 0; i < scene.keyCount * 2; ++i )
        check( custom.Contains( makeKey( i ) ) ==
            reference.contains( makeKey( i ) ),
            "Custom/std final pair membership mismatch" );
}

template <typename Set>
double measureOne( const scenario& scene )
{
    auto set = makeSet<Set>( scene );
    prepare( set, scene );

    std::uint64_t checksum = 0;
    const auto start = std::chrono::steady_clock::now();
    for( const command& input : scene.commands )
        checksum += apply( set, input ) ? 1u : 0u;
    const auto finish = std::chrono::steady_clock::now();

    // 결과를 외부로 노출해 연산 전체가 제거되는 최적화를 피함.
    benchmarkSink = benchmarkSink + checksum + count( set );
    return static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>( finish - start ).count() ) /
        static_cast<double>( scene.commands.size() );
}

template <typename Set>
double medianNanoseconds( const scenario& scene )
{
    std::array<double, MEASURE_ROUNDS> times{};
    ( void )measureOne<Set>( scene );
    for( double& value : times ) value = measureOne<Set>( scene );

    std::sort( times.begin(), times.end() );
    return times[MEASURE_ROUNDS / 2];
}

} // namespace

int main( int argc, char** argv )
{
    bool verifyOnly = false;
    bool csv = false;
    for( int i = 1; i < argc; ++i )
    {
        if( std::strcmp( argv[i], "--verify" ) == 0 ) verifyOnly = true;
        else if( std::strcmp( argv[i], "--csv" ) == 0 ) csv = true;
        else
        {
            std::fprintf( stderr, "Unknown option: %s\n", argv[i] );
            return EXIT_FAILURE;
        }
    }

    const auto scenarios = makeScenarios();
    for( const scenario& scene : scenarios ) verify( scene );
    if( verifyOnly ) return EXIT_SUCCESS;

    if( csv ) std::printf(
        "scenario,operations,initial_pairs,custom_ns_per_op,std_ns_per_op,custom_over_std\n" );

    for( const scenario& scene : scenarios )
    {
        const double customNs = medianNanoseconds<hashSet>( scene );
        const double stdNs = medianNanoseconds<std::unordered_set<shapePairKey>>( scene );
        const double ratio = stdNs > 0.0 ? customNs / stdNs : 0.0;

        if( csv )
            std::printf( "%s,%zu,%zu,%.2f,%.2f,%.3f\n",
                scene.name, scene.commands.size(), scene.initialCount, customNs, stdNs, ratio );
        else
            std::printf( "%s: custom %.2f ns/op, std %.2f ns/op, custom/std %.3f\n",
                scene.name, customNs, stdNs, ratio );
    }
}
