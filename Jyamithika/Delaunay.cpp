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
        vertex_list.push_back(vertex);
    }

    void AddEdge(Edge2D* edge)
    {
        edge_list.push_back(edge);
    }

    void AddFace(Face2D* face)
    {
        face_list.push_back(face);
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
    
    DelaunayMesh* delaunayMesh = new DelaunayMesh();
    
    //构造巨大的三角形，包围住所有的点
    Point2d p0 = Point2d(min_x - 0.5 * width, min_y - 0.5 * height);
    Point2d p1 = Point2d(min_x + 2.5 * width, min_y - 0.5 * height);
    Point2d p2 = Point2d(min_x - 0.5 * width, min_y + 2.5 * height);
    
    //构造顶点结构
    Vertex2D *vertex0 = new Vertex2D(p0);
    Vertex2D *vertex1 = new Vertex2D(p1);
    Vertex2D *vertex2 = new Vertex2D(p2);
    delaunayMesh->AddVertex(vertex0);
    delaunayMesh->AddVertex(vertex1);
    delaunayMesh->AddVertex(vertex2);
    
    //构造边数据
    Edge2D *edge12 = new Edge2D(vertex0);
    //Edge2D *edge21 = new Edge2D(vertex1);
    
    Edge2D *edge23 = new Edge2D(vertex1);
    //Edge2D *edge32 = new Edge2D(vertex2);
    
    Edge2D *edge31 = new Edge2D(vertex2);
    //Edge2D *edge13 = new Edge2D(vertex0);
    
    edge12->next = edge23;
    edge12->prev = edge31;
    edge12->twin = nullptr;
    
    edge23->next = edge31;
    edge23->prev = edge12;
    edge23->twin = nullptr;
    
    edge31->next = edge12;
    edge31->prev = edge23;
    edge31->twin = nullptr;
    delaunayMesh->AddEdge(edge12);
    delaunayMesh->AddEdge(edge23);
    delaunayMesh->AddEdge(edge31);
    
    //构造面数据
    Face2D *face = new Face2D();
    face->outer = edge12;
    delaunayMesh->AddFace(face);
    
    //设置其它的元素
    edge12->incident_face = face;
    edge23->incident_face = face;
    edge31->incident_face = face;
    
    vertex0->incident_edge = edge12;
    vertex0->index = -1;
    vertex1->incident_edge = edge23;
    vertex1->index = -2;
    vertex2->incident_edge = edge31;
    vertex2->index = -3;
    
    std::vector<Edge2D*> faceEdges = face->getEdgeList();
    for (int i = 0; i < faceEdges.size(); i ++)
    {
        printf("addr = %p\n", faceEdges[i]);
    }
}

}
