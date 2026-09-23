#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace zonai
{

class HashSet
{
public:
    explicit HashSet( std::size_t capacity = MIN_CAPACITY );

    // 새 key면 false, 이미 존재하는 key면 true를 반환함.
    bool Add( std::uint64_t key );

    // key를 찾으면 제거하고 true, 없으면 false를 반환함.
    bool Remove( std::uint64_t key );

    bool Contains( std::uint64_t key ) const;

    std::size_t GetCount() const;
    std::size_t GetCapacity() const;

private:
    static constexpr std::size_t MIN_CAPACITY = 16;

    static std::size_t NormalizeCapacity( std::size_t capacity );
    static std::uint64_t KeyHash( std::uint64_t key );

    std::size_t FindSlot( std::uint64_t key, std::uint64_t hash ) const;
    void AddHaveCapacity( std::uint64_t key, std::uint64_t hash );
    void Grow();

private:
    // 0을 빈 slot sentinel로 사용하며 linear probing으로 충돌을 해결함.
    std::vector<std::uint64_t> items_{};
    std::size_t count_ = 0;
};

} // namespace zonai
