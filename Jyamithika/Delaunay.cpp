#include "Delaunay.h"

#include "Core/Primitives/Point.h"
#include "Core/Primitives/PolygonDCEL.h"



namespace jmk
{
constexpr float EPSILON = std::numeric_limits<float>::epsilon();

inline bool in_circle(
    const double ax,
    const double ay,
    const double bx,
    const double by,
    const double cx,
    const double cy,
    const double px,
    const double py) {
    const double dx = ax - px;
    const double dy = ay - py;
    const double ex = bx - px;
    const double ey = by - py;
    const double fx = cx - px;
    const double fy = cy - py;

    const double ap = dx * dx + dy * dy;
    const double bp = ex * ex + ey * ey;
    const double cp = fx * fx + fy * fy;

    return (dx * (ey * cp - bp * fy) -
            dy * (ex * cp - bp * fx) +
            ap * (ex * fy - ey * fx)) < 0.0;
}

void InsertVertex(DelaunayMesh * dmesh, Face2D* currentFace,
                  Vertex2D* a, Vertex2D* b, Vertex2D* c, Vertex2D* p,
                  Edge2D* edgeAB, Edge2D* edgeBC, Edge2D* edgeCA)
{
    Edge2D* edgePA = new Edge2D(p);
    Edge2D* edgePB = new Edge2D(p);
    Edge2D* edgePC = new Edge2D(p);
    Edge2D* edgeAP = new Edge2D(a);
    Edge2D* edgeBP = new Edge2D(b);
    Edge2D* edgeCP = new Edge2D(c);
    
    dmesh->AddEdge(edgePA);
    dmesh->AddEdge(edgePB);
    dmesh->AddEdge(edgePC);
    dmesh->AddEdge(edgeAP);
    dmesh->AddEdge(edgeBP);
    dmesh->AddEdge(edgeCP);
    
    edgePA->twin = edgeAP;
    edgeAP->twin = edgePA;
    edgePB->twin = edgeBP;
    edgeBP->twin = edgePB;
    edgePC->twin = edgeCP;
    edgeCP->twin = edgePC;
    
    //三角形pab的边连接关系
    edgeAB->next = edgeBP;
    edgeBP->next = edgePA;
    edgePA->next = edgeAB;
    edgeAB->prev = edgePA;
    edgePA->prev = edgeBP;
    edgeBP->prev = edgeAB;
    Face2D* faceABP = new Face2D();
    faceABP->outer = edgeAB;
    
    //三角形pbc的边连接关系
    edgeBC->next = edgeCP;
    edgeCP->next = edgePB;
    edgePB->next = edgeBC;
    edgeBC->prev = edgePB;
    edgePB->prev = edgeCP;
    edgeCP->prev = edgeBC;
    Face2D* faceBCP = new Face2D();
    faceBCP->outer = edgeBC;
    
    //三角形pca的边连接关系
    edgeCA->next = edgeAP;
    edgeAP->next = edgePC;
    edgePC->next = edgeCA;
    edgeCA->prev = edgePC;
    edgePC->prev = edgeAP;
    edgeAP->prev = edgeCA;
    Face2D* faceCAP = new Face2D();
    faceCAP->outer = edgeCA;
    
    //设置新增半边的的关联的面
    edgePA->incident_face = faceABP;
    edgeAP->incident_face = faceCAP;
    edgePB->incident_face = faceBCP;
    edgeBP->incident_face = faceABP;
    edgePC->incident_face = faceCAP;
    edgeCP->incident_face = faceBCP;
    
    edgeAB->incident_face = faceABP;
    edgeBC->incident_face = faceBCP;
    edgeCA->incident_face = faceCAP;
    
    dmesh->AddFace(faceABP);
    dmesh->AddFace(faceBCP);
    dmesh->AddFace(faceCAP);
    dmesh->AddVertex(p);
    p->incident_edge = edgePA;
    
    dmesh->RemoveFace(currentFace);
    
//    std::vector<Face2D *> faces;
//    faces.push_back(faceABP);
//    faces.push_back(faceBCP);
//    faces.push_back(faceCAP);
//
//    std::vector<Face2D *> currentFaces;
//    currentFaces.push_back(currentFace);
//
//    //重新构造相关的桶
//    dmesh->Rebucket(faces, currentFaces);
}

Vertex2D* getRightVertex(Edge2D* edge)
{
    if (edge == nullptr)
    {
        return nullptr;
    }
    
    if (edge->twin == nullptr)
    {
        return nullptr;
    }
    
    if (edge->twin->next == nullptr)
    {
        return nullptr;
    }
    
    if (edge->twin->next->next == nullptr)
    {
        return nullptr;
    }
    
    return edge->twin->next->next->origin;
}

//空圆测试
void SwapTest(DelaunayMesh * dmesh, Vertex2D* p,
              Edge2D* edgeAB, Edge2D* edgeBC, Edge2D* edgeCA)
{
    std::queue<Edge2D*> edgeQueue;
    edgeQueue.push(edgeAB);
    edgeQueue.push(edgeBC);
    edgeQueue.push(edgeCA);
    
    while (!edgeQueue.empty())
    {
        Edge2D* edge = edgeQueue.front();
        edgeQueue.pop();
        
        Vertex2D *x = getRightVertex(edge);
        
        if (nullptr == x)
        {
            continue;
        }
        
        Vertex2D* a = edge->origin;
        Vertex2D* b = edge->next->origin;
        
        float ax = a->point[X];
        float ay = a->point[Y];
        float bx = b->point[X];
        float by = b->point[Y];
        float px = p->point[X];
        float py = p->point[Y];
        float xx = x->point[X];
        float xy = x->point[Y];
        
        /* if the pair of triangles doesn't satisfy the Delaunay condition
            * (p1 is inside the circumcircle of [p0, pl, pr]), flip them,
            * then do the same check/flip recursively for the new pair of triangles
            *
            *           pl                    pl
            *          /||\                  /  \
            *       al/ || \bl            al/    \a
            *        /  ||  \              /      \
            *       /  a||b  \    flip    /___ar___\
            *     p0\   ||   /p1   =>   p0\---bl---/p1
            *        \  ||  /              \      /
            *       ar\ || /br             b\    /br
            *          \||/                  \  /
            *           pr                    pr
         */
        
        if (in_circle(px, py, ax, ay, bx, by, xx, xy))
        {
            //老的两个面
            Face2D* leftFace = edge->incident_face;
            Face2D* rightFace = edge->twin->incident_face;
            std::vector<Face2D*> currentFaces;
            currentFaces.push_back(leftFace);
            currentFaces.push_back(rightFace);
            
            //下面是边翻转相关的操作
            
            //连接px
            Edge2D* edgePX = new Edge2D(p);
            Edge2D* edgeXP = new Edge2D(x);
            edgePX->twin = edgeXP;
            edgeXP->twin = edgePX;
            
            //设置相关边的指针
            edge->next->next = edgePX;
            edgePX->next = edge->twin->prev;
            edge->twin->prev->next = edge->next;
            
            edge->next->prev = edge->twin->prev;
            edge->twin->prev->prev = edgePX;
            edgePX->prev = edge->next;
            
            edge->prev->next = edge->twin->next;
            edge->twin->next->next = edgeXP;
            edgeXP->next = edge->prev;
            
            edge->prev->prev = edgeXP;
            edgeXP->prev = edge->twin->next;
            edge->twin->next->prev = edge->prev;
            
            Face2D* facePAX = new Face2D();
            facePAX->outer = edgeXP;
            
            Face2D* facePXB = new Face2D();
            facePXB->outer = edgePX;
            
            //设置边的面
            edgePX->incident_face = facePXB;
            edgeXP->incident_face = facePAX;
            
            edge->next->incident_face = facePXB;
            edgePX->incident_face = facePXB;
            edge->twin->prev->incident_face = facePXB;
            
            edge->prev->incident_face = facePAX;
            edge->twin->next->incident_face = facePAX;
            edgeXP->incident_face = facePAX;
            
            std::vector<Face2D*> newFaces;
            newFaces.push_back(facePXB);
            newFaces.push_back(facePAX);
            //dmesh->Rebucket(newFaces, currentFaces);
            
            edgeQueue.push(edge->twin->next);
            edgeQueue.push(edge->twin->prev);
            
            dmesh->AddFace(facePXB);
            dmesh->AddFace(facePAX);
            
            dmesh->AddEdge(edgePX);
            dmesh->AddEdge(edgeXP);
            
            dmesh->RemoveEdge(edge->twin);
            dmesh->RemoveEdge(edge);
            
            dmesh->RemoveFace(leftFace);
            dmesh->RemoveFace(rightFace);
        }
    }
}

// 随机增量算法构造三角网
DelaunayMesh* constructDelaunay_increment(const std::vector<Point2d>& points)
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
    
    //设置顶点的bucket
    std::unordered_set<Vertex2D*> allVertexs;
    
    //将所有点先放入超级三角形中
    int index = 0;
    for (const auto & point : points)
    {
        Point2d p = point;
        Vertex2D *vertex = new Vertex2D(p);
        vertex->index = index;
        index ++;
        allVertexs.insert(vertex);
    }
    delaunayMesh->InsertBucket(face, allVertexs);
    
    //开始循环每个点
    for (Vertex2D* vertex : allVertexs)
    {
        //找到当前顶点对应的三角形
        Face2D* face = delaunayMesh->FindFace(vertex);
        if (nullptr == face)
        {
            continue;
        }
        
        std::vector<Vertex2D*> vertexs = face->getVertexs();
        assert(vertexs.size() == 3);
        
        std::vector<Edge2D *> edges = face->getEdgeList();
        assert(edges.size() == 3);
        
        Vertex2D* a = vertexs[0];
        Vertex2D* b = vertexs[1];
        Vertex2D* c = vertexs[2];
        
        Edge2D* edgeAB = edges[0];
        Edge2D* edgeBC = edges[1];
        Edge2D* edgeCA = edges[2];
        
        //插入当前点
        InsertVertex(delaunayMesh, face, a, b, c, vertex, edgeAB, edgeBC, edgeCA);
        printf("当前剩余的顶点个数 : %d\n", delaunayMesh->getLeftVextexCount());
        
        SwapTest(delaunayMesh, vertex, edgeAB, edgeBC, edgeCA);
    }
    
    //最后删除多余的三角形
    delaunayMesh->DeleteOuterFace();
    
    return delaunayMesh;;
}

}
