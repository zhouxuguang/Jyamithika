#pragma once
#include <vector>
#include "Core/Primitives/Point.h"
#include "Core/Primitives/Polygon.h"
#include "Core/Primitives/Bounds.h"

// 
namespace jmk
{

	// 随机增量算法构造三角网
	void constructDelaunay_increment(const std::vector<Point2d>& points, std::vector<Edge2dSimple>& edges);
}