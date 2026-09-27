#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "dynamics/body.h"

namespace zonai
{

class World
{
public:
    // Body를 World storage 끝에 생성하고 해당 Body index를 반환함.
    [[nodiscard]] std::int32_t CreateBody(
        BodyType type = BodyType::Static,
        transform2 transform = {} );

    [[nodiscard]] Body& GetBody( std::int32_t bodyId );
    [[nodiscard]] const Body& GetBody( std::int32_t bodyId ) const;

    [[nodiscard]] std::size_t GetBodyCount() const noexcept
    {
        return bodies_.size();
    }

private:
    // 현재 단계에서는 Body를 연속 배열로 보관함.
    // DestroyBody / id 재사용은 추후 Body id pool을 추가할 때 구현함.
    std::vector<Body> bodies_;
};

} // namespace zonai
