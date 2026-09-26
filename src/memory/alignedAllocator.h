#pragma once

#include <cstddef>
#include <limits>
#include <new>
#include <type_traits>

namespace zonai
{

// std::vector를 유지하면서 Box2D가 tree node 배열에 요구하는 정렬을 보존하기 위한 allocator.
template <typename T, std::size_t Alignment>
class AlignedAllocator
{
public:
    using value_type = T;

    static_assert( ( Alignment & ( Alignment - 1 ) ) == 0 );
    static_assert( Alignment >= alignof( T ) );

    AlignedAllocator() noexcept = default;

    template <typename U>
    AlignedAllocator( const AlignedAllocator<U, Alignment>& ) noexcept
    {
    }

    [[nodiscard]] T* allocate( std::size_t count )
    {
        if( count > std::numeric_limits<std::size_t>::max() / sizeof( T ) )
        {
            throw std::bad_array_new_length{};
        }

        return static_cast<T*>(
            ::operator new(
                count * sizeof( T ),
                std::align_val_t{ Alignment } )
        );
    }

    void deallocate( T* pointer, std::size_t ) noexcept
    {
        ::operator delete(
            pointer,
            std::align_val_t{ Alignment }
        );
    }

    template <typename U>
    struct rebind
    {
        using other = AlignedAllocator<U, Alignment>;
    };
};

template <typename T, typename U, std::size_t Alignment>
constexpr bool operator==(
    const AlignedAllocator<T, Alignment>&,
    const AlignedAllocator<U, Alignment>& ) noexcept
{
    return true;
}

template <typename T, typename U, std::size_t Alignment>
constexpr bool operator!=(
    const AlignedAllocator<T, Alignment>&,
    const AlignedAllocator<U, Alignment>& ) noexcept
{
    return false;
}

} // namespace zonai
