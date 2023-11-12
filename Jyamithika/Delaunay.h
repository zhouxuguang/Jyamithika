#pragma once
#include <vector>
#include "Core/Primitives/Point.h"
#include "Core/Primitives/Polygon.h"
#include "Core/Primitives/Bounds.h"

// 
namespace jmk
{

    //判断点是否在三角形的外接圆内
    inline bool in_circle(
        const float ax,
        const float ay,
        const float bx,
        const float by,
        const float cx,
        const float cy,
        const float px,
        const float py) 
    {
        const float dx = ax - px;
        const float dy = ay - py;
        const float ex = bx - px;
        const float ey = by - py;
        const float fx = cx - px;
        const float fy = cy - py;

        const float ap = dx * dx + dy * dy;
        const float bp = ex * ex + ey * ey;
        const float cp = fx * fx + fy * fy;

        return (dx * (ey * cp - bp * fy) -
                dy * (ex * cp - bp * fx) +
                ap * (ex * fy - ey * fx)) < 0.0;
    }

	// 随机增量算法构造三角网
	void constructDelaunay_increment(const std::vector<Point2d>& points, std::vector<Edge2dSimple>& edges);
}