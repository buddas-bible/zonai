#pragma once

namespace zonai
{
	struct rot2
	{
		float c = 1.f; // Cosine of the rotation angle
		float s = 0.f; // Sine of the rotation angle
	};
}

// x' = c * x - s * y;
// y' = s * x + c * y;