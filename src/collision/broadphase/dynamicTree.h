#pragma once

#include <cstdint>

#include "collision/aabb2.h"

namespace zonai
{

struct TreeNode
{
	// 노드 바운딩 박스
	aabb2 aabb{};

	// bit 31 : 1 리프 노드에 대해 1
	// 즉, 이 비트가 1이면 해당 노드가 리프 노드임을 나타냅니다.
	// 
	// bit 30 : 1 for moved flag 이동된 플래그에 대해 1
	// 즉, 이 비트가 1이면 해당 노드가 이동되었음을 나타냅니다.
	// 
	// bit 0~29 : 형제 쌍 노드의 인덱스 또는 리프에 대한 프록시 ID
	// 즉, 이 비트들은 형제 노드의 인덱스나 리프 노드에 대한 프록시 ID를 나타냅니다.
	std::uint32_t flagIndex = 0;

	union
	{
		// 내부 노드에 대한 하위 리프의 총 수입니다. 현재 사용되지 않습니다.
		int32_t leafCount;

		// 리프 노드에 대한 모양 인덱스입니다. 프록시 사용자 데이터에서 잘린 값입니다.
		int32_t shapeIndex;
	};
};

class DynamicTree
{

};


}