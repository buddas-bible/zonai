#include "collision/broadphase/hashSet.h"

#include <bit>
#include <cassert>
#include <utility>

namespace zonai
{

HashSet::HashSet( std::size_t capacity )
    : items_( NormalizeCapacity( capacity ), 0 )
{
}

bool HashSet::Add( std::uint64_t key )
{
    // 0은 빈 slot을 나타내는 sentinel이므로 key로 사용할 수 없음.
    assert( key != 0 );

    const std::uint64_t hash = KeyHash( key );
    const std::size_t index = FindSlot( key, hash );

    if( items_[index] != 0 )
    {
        assert( items_[index] == key );
        return true;
    }

    // 절반 이상 차면 probe 길이가 늘기 전에 capacity를 두 배로 키움.
    if( 2 * count_ >= items_.size() )
    {
        Grow();
    }

    AddHaveCapacity( key, hash );
    return false;
}

bool HashSet::Remove( std::uint64_t key )
{
    assert( key != 0 );

    const std::uint64_t hash = KeyHash( key );
    std::size_t emptyIndex = FindSlot( key, hash );

    if( items_[emptyIndex] == 0 )
    {
        return false;
    }

    // 삭제한 slot을 비우고 뒤쪽 probe chain에서 필요한 항목을 앞으로 당김.
    items_[emptyIndex] = 0;

    assert( count_ > 0 );
    --count_;

    const std::size_t mask = items_.size() - 1;
    std::size_t scanIndex = emptyIndex;

    for( ;; )
    {
        scanIndex = ( scanIndex + 1 ) & mask;

        if( items_[scanIndex] == 0 )
        {
            break;
        }

        // 현재 key가 hash상 처음 들어가려던 slot을 구함.
        const std::size_t homeIndex = static_cast<std::size_t>( KeyHash( items_[scanIndex] ) ) & mask;

        // homeIndex가 현재 빈 slot을 건너뛰지 않는 위치라면 그대로 둠.
        if( emptyIndex <= scanIndex )
        {
            if( emptyIndex < homeIndex && homeIndex <= scanIndex )
            {
                continue;
            }
        }
        else
        {
            // probe chain이 배열 끝에서 처음으로 wrap된 경우의 순환 구간을 처리함.
            if( emptyIndex < homeIndex || homeIndex <= scanIndex )
            {
                continue;
            }
        }

        // 빈 slot 때문에 탐색이 끊길 key를 앞으로 옮겨 probe chain을 복구함.
        items_[emptyIndex] = items_[scanIndex];
        items_[scanIndex] = 0;
        emptyIndex = scanIndex;
    }

    return true;
}

bool HashSet::Contains( std::uint64_t key ) const
{
    assert( key != 0 );

    const std::uint64_t hash = KeyHash( key );
    const std::size_t index = FindSlot( key, hash );

    return items_[index] == key;
}

std::size_t HashSet::GetCount() const
{
    return count_;
}

std::size_t HashSet::GetCapacity() const
{
    return items_.size();
}

std::size_t HashSet::NormalizeCapacity( std::size_t capacity )
{
    if( capacity <= MIN_CAPACITY )
    {
        return MIN_CAPACITY;
    }

    return std::bit_ceil( capacity );
}

std::uint64_t HashSet::KeyHash( std::uint64_t key )
{
    // Box2D와 같은 Murmur hash finalizer를 사용함.
    std::uint64_t hash = key;
    hash ^= hash >> 33;
    hash *= 0xff51afd7ed558ccdULL;
    hash ^= hash >> 33;
    hash *= 0xc4ceb9fe1a85ec53ULL;
    hash ^= hash >> 33;
    return hash;
}

std::size_t HashSet::FindSlot( std::uint64_t key, std::uint64_t hash ) const
{
    const std::size_t mask = items_.size() - 1;
    std::size_t index = static_cast<std::size_t>( hash ) & mask;

    // 같은 key나 빈 slot을 찾을 때까지 다음 slot으로 이동함.
    while( items_[index] != 0 && items_[index] != key )
    {
        index = ( index + 1 ) & mask;
    }

    return index;
}

void HashSet::AddHaveCapacity( std::uint64_t key, std::uint64_t hash )
{
    const std::size_t index = FindSlot( key, hash );

    assert( items_[index] == 0 );

    items_[index] = key;
    ++count_;
}

void HashSet::Grow()
{
    const std::size_t oldCount = count_;
    std::vector<std::uint64_t> oldItems = std::move( items_ );

    items_.assign( oldItems.size() * 2, 0 );
    count_ = 0;

    // capacity가 바뀌면 mask도 달라지므로 기존 key를 모두 다시 배치함.
    for( const std::uint64_t key : oldItems )
    {
        if( key == 0 )
        {
            continue;
        }

        AddHaveCapacity( key, KeyHash( key ) );
    }

    assert( count_ == oldCount );
}

} // namespace zonai
