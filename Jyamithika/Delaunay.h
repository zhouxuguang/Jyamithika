#pragma once
#include <vector>
#include "Core/Primitives/Point.h"
#include "Core/Primitives/Polygon.h"
#include "Core/Primitives/Bounds.h"
#include "Core/Primitives/PolygonDCEL.h"

#include <unordered_map>
#include <unordered_set>
#include <queue>

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
                ap * (ex * fy - ey * fx)) > 0.0;
    }

// 判断点是否在三角形内部
inline bool pointInTriangle(float x1, float y1, float x2, float y2, float x3, float y3, float x, float y)
{
    bool b1, b2, b3;

    b1 = ((y - y1) * (x2 - x1) - (x - x1) * (y2 - y1)) <= 0.0;
    b2 = ((y - y2) * (x3 - x2) - (x - x2) * (y3 - y2)) <= 0.0;
    b3 = ((y - y3) * (x1 - x3) - (x - x3) * (y1 - y3)) <= 0.0;

    return ((b1 == b2) && (b2 == b3));
}

typedef VertexDCEL<float, 2> Vertex2D;
typedef EdgeDCEL<float, 2> Edge2D;
typedef FaceDCEL<float, 2> Face2D;

inline bool InsideTriangle(double Ax, double Ay,
                    double Bx, double By,
                    double Cx, double Cy,
                    double Px, double Py)

{
    double ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
    double cCROSSap, bCROSScp, aCROSSbp;

    ax = Cx - Bx;  ay = Cy - By;
    bx = Ax - Cx;  by = Ay - Cy;
    cx = Bx - Ax;  cy = By - Ay;
    apx= Px - Ax;  apy= Py - Ay;
    bpx= Px - Bx;  bpy= Py - By;
    cpx= Px - Cx;  cpy= Py - Cy;

    aCROSSbp = ax*bpy - ay*bpx;
    cCROSSap = cx*apy - cy*apx;
    bCROSScp = bx*cpy - by*cpx;

    return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
}

class DelaunayMesh
{
public:
    DelaunayMesh()
    {}
    
    std::vector<Face2D*> GetFaces()
    {
        std::vector<Face2D*> faces;
        faces.reserve(face_list.size());
        for (auto iter : face_list)
        {
            faces.push_back(iter);
        }
        
        return faces;
    }

    void AddVertex(Vertex2D *vertex)
    {
        vertex_list.push_back(vertex);
    }

    void AddEdge(Edge2D* edge)
    {
        edge_list.insert(edge);
    }

    void AddFace(Face2D* face)
    {
        face_list.insert(face);
    }
    
    void RemoveEdge(Edge2D* edge)
    {
        auto iter = edge_list.find(edge);
        if (iter != edge_list.end())
        {
            Edge2D* iterEdge = *iter;
            edge_list.erase(iter);
            delete iterEdge;
        }
    }
    
    void RemoveFace(Face2D* face)
    {
        auto iter = face_list.find(face);
        if (iter != face_list.end())
        {
            Face2D* iterFace = *iter;
            face_list.erase(iter);
            delete iterFace;
        }
    }
    
    void InsertBucket(Face2D* face, const std::unordered_set<Vertex2D*>& bucket)
    {
        mBuckets.insert(std::make_pair(face, bucket));
    }
    
    Face2D* FindFace(Vertex2D* point)
    {
#if 0
        for (auto &bucket : mBuckets)
        {
            auto iter = bucket.second.find(point);
            if (iter != bucket.second.end())
            {
                bucket.second.erase(iter);
                return bucket.first;
            }
        }
#endif
        for (auto iter : face_list)
        {
            std::vector<Point2d> points = iter->getPoints();
            assert(points.size() == 3);
            float ax = points[0][X];
            float ay = points[0][Y];
            float bx = points[1][X];
            float by = points[1][Y];
            float cx = points[2][X];
            float cy = points[2][Y];
            float px = point->point[X];
            float py = point->point[Y];
            
            if (InsideTriangle(ax, ay, bx, by, cx, cy, px, py))
            {
                return iter;
            }
        }
        
        return nullptr;
    }
    
    int getLeftVextexCount()
    {
        int count = 0;
        for (auto &bucket : mBuckets)
        {
            count += bucket.second.size();
        }
        
        return count;
    }
    
    void DeleteOuterFace()
    {
        std::unordered_set<Face2D*> resultFaces;
        for (auto iter = face_list.begin(); iter != face_list.end(); ++iter)
        {
            std::vector<Vertex2D* > vertexs = (*iter)->getVertexs();
            bool foundOuter = false;
            for (auto vertex : vertexs)
            {
                if (vertex->index < 0)
                {
                    foundOuter = true;
                }
            }
            
            if (foundOuter)
            {
                Face2D* face = *iter;
//                auto iterNext = std::next(iter);
//                face_list.erase(iter);
//                iter = iterNext;
                delete face;
            }
            else
            {
                resultFaces.insert(*iter);
            }
            
        }
        
        face_list.swap(resultFaces);
    }
    
    //重新rebucket
    void Rebucket(std::vector<Face2D*> faces, const std::vector<Face2D*> &currentFaces)
    {
        std::vector<Vertex2D*> rebucketVetexs;
        for (size_t i = 0; i < currentFaces.size(); i ++)
        {
            auto iter = mBuckets.find(currentFaces[i]);
            if (iter != mBuckets.end())
            {
                for (auto &iterVertex : iter->second)
                {
                    rebucketVetexs.push_back(iterVertex);
                }
                iter->second.clear();
                mBuckets.erase(iter);
            }
            
            auto iterFace = std::find(face_list.begin(), face_list.end(), currentFaces[i]);
            if (iterFace != face_list.end())
            {
                Face2D* face = *iterFace;
                face_list.erase(iterFace);
                delete face;
                face = nullptr;
            }
        }
        
        //将rebucketVetexs分配到faces中
        std::vector<std::unordered_set<Vertex2D*>> buckets;
        buckets.resize(faces.size());
        
        int pointCount = rebucketVetexs.size();
        
        int bucketCount = 0;
        
        for (size_t i = 0; i < faces.size(); i ++)
        {
            std::vector<Point2d> points = faces[i]->getPoints();
            assert(points.size() == 3);
            float ax = points[0][X];
            float ay = points[0][Y];
            float bx = points[1][X];
            float by = points[1][Y];
            float cx = points[2][X];
            float cy = points[2][Y];
            
            for (size_t j = 0; j < rebucketVetexs.size(); j ++)
            {
                if (InsideTriangle(ax, ay, bx, by, cx, cy, rebucketVetexs[j]->point[X], rebucketVetexs[j]->point[Y]))
                {
                    buckets[i].insert(rebucketVetexs[j]);
                    bucketCount ++;
                }
            }
            
            mBuckets.emplace(faces[i], buckets[i]);
        }
        
        assert(pointCount == bucketCount);
        
        printf("pointCount = %d, rebucketCount = %d\n", pointCount, bucketCount);
    }

private:
    std::vector<Vertex2D*> vertex_list;
    std::unordered_set<Edge2D*> edge_list;
    std::unordered_set<Face2D*> face_list;
    
    std::unordered_map<Face2D*, std::unordered_set<Vertex2D*>> mBuckets;
    
};

	// 随机增量算法构造三角网
    DelaunayMesh* constructDelaunay_increment(const std::vector<Point2d>& points);
}
