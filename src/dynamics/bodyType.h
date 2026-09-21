#pragma once

#include <cstddef>

namespace zonai
{

enum class BodyType : std::size_t
{
    Static = 0,
    Kinematic = 1,
    Dynamic = 2,
    Count = 3
};

} // namespace zonai
