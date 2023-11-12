#include "Delaunay.h"
#include "Core/Primitives/Point.h"
#include "Core/Primitives/PolygonDCEL.h"

namespace jmk
{
constexpr float EPSILON = std::numeric_limits<float>::epsilon();


typedef VertexDCEL<float, 2> Vertex2D;
typedef EdgeDCEL<float, 2> Edge2D;
typedef FaceDCEL<float, 2> Face2D;

class DelaunayMesh
{
public:
    DelaunayMesh()
    {}

    void AddVertex(Vertex2D *vertex)
    {
    }

    void AddEdge(Edge2D* edge)
    {
        //
    }

    void AddFace(Face2D* face)
    {
        //
    }

private:
    std::vector<Vertex2D*> vertex_list;
    std::vector<Edge2D*> edge_list;
    std::vector<Face2D*> face_list;
    
    
};

// 随机增量算法构造三角网
void constructDelaunay_increment(const std::vector<Point2d>& points, std::vector<Edge2dSimple>& edges)
{
    //先计算包围盒，构造一个巨大的三角形
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float min_x = (std::numeric_limits<float>::max)();
    float min_y = (std::numeric_limits<float>::max)();
    for (const Point2d& p : points)
    {
        min_x = std::min(p[X], min_x);
        min_y = std::min(p[Y], min_y);
        max_x = std::max(p[X], max_x);
        max_y = std::max(p[Y], max_y);
    }
    float width = max_x - min_x;
    float height = max_y - min_y;
    
    //构造巨大的三角形，包围住所有的点
    Point2d p0 = Point2d(min_x - 0.5 * width, min_y - 0.5 * height);
    Point2d p1 = Point2d(min_x + 3.0 * width, min_y - 0.5 * height);
    Point2d p2 = Point2d(min_x - 0.5 * width, min_y + 3.0 * height);
}

}
