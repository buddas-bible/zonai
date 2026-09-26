#pragma once

#include <cstddef>
#include <limits>
#include <new>
#include <type_traits>

namespace zonai
{

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

    // count개의 T를 저장할 Alignment byte 정렬 메모리를 할당함.
    // std::vector가 allocator를 통해 호출하며 반환된 메모리의 객체 생성은 vector가 담당함.

    [[nodiscard]] // 반환된 메모리 주소를 실수로 버리지 않도록 경고하게 함.
    T* allocate( std::size_t count )
    {
        // byte 크기를 계산하기 전에 정수 overflow가 발생할 수 있는지 검사함.
        if( count > std::numeric_limits<std::size_t>::max() / sizeof( T ) )
        {
            throw std::bad_array_new_length{};
        }

        // count * sizeof(T) byte의 연속 메모리를 Alignment byte 경계에 맞춰 할당함.
        return static_cast<T*>(
            ::operator new(
                count * sizeof( T ),
                std::align_val_t{ Alignment } ) // aligned operator new에 정렬 크기를 전달함.
        );
    }

    void deallocate( T* pointer, std::size_t ) noexcept
    {
        // align_val_t를 사용해 할당한 메모리는
        // 동일한 Alignment를 지정한 aligned operator delete로 해제해야 함.
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
