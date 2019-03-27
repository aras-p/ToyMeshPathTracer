#include "scene.h"
#include <algorithm>
#include <string.h>
#include <vector>
#include <utility>

#define USE_BVH 1

// --------------------------------------------------------------------------
// Axis-aligned bounding box and related functions

struct AABB
{
    float3 bmin;
    float3 bmax;
};

// from Peter Shirley's "Ray Tracing: The Next Week"
static bool HitAABB(const Ray& r, const AABB& box, float tMin, float tMax)
{
#define DO_COORD(c) \
    { \
        float invD = 1.0f / r.dir.c; \
        float t0 = (box.bmin.c - r.orig.c) * invD; \
        float t1 = (box.bmax.c - r.orig.c) * invD; \
        if (invD < 0.0f) \
            std::swap(t0, t1); \
        tMin = t0 > tMin ? t0 : tMin; \
        tMax = t1 < tMax ? t1 : tMax; \
        if (tMax < tMin) \
            return false; \
    }
    DO_COORD(x);
    DO_COORD(y);
    DO_COORD(z);
    return true;
}

static AABB AABBUnion(const AABB& a, const AABB& b)
{
    AABB res;
    res.bmin = min(a.bmin, b.bmin);
    res.bmax = max(a.bmax, b.bmax);
    return res;
}

static AABB AABBEnclose(const AABB& a, const float3& p)
{
    AABB res;
    res.bmin = min(a.bmin, p);
    res.bmax = max(a.bmax, p);
    return res;
}

static AABB AABBOfTriangle(const Triangle& tri)
{
    AABB res;
    res.bmin = tri.v0;
    res.bmax = tri.v0;
    res = AABBEnclose(res, tri.v1);
    res = AABBEnclose(res, tri.v2);
    return res;
}


// --------------------------------------------------------------------------
// Checks if one triangle is hit by a ray segment.

static bool HitTriangle(const Ray& r, const Triangle& tri, float tMin, float tMax, Hit& outHit)
{
    float3 edge0 = tri.v1 - tri.v0;
    float3 edge1 = tri.v2 - tri.v1;
    float3 normal = normalize(cross(edge0, edge1));
    float planeOffset = dot(tri.v0, normal);

    float3 p0 = r.pointAt(tMin);
    float3 p1 = r.pointAt(tMax);

    float offset0 = dot(p0, normal);
    float offset1 = dot(p1, normal);

    // does the ray segment between tMin & tMax intersect the triangle plane?
    if ((offset0 - planeOffset) * (offset1 - planeOffset) <= 0.0f)
    {
        float t = tMin + (tMax - tMin)*(planeOffset - offset0) / (offset1 - offset0);
        float3 p = r.pointAt(t);

        float3 c0 = cross(edge0, p - tri.v0);
        float3 c1 = cross(edge1, p - tri.v1);
        if (dot(c0, c1) >= 0.f)
        {
            auto edge2 = tri.v0 - tri.v2;
            auto c2 = cross(edge2, p - tri.v2);
            if (dot(c1, c2) >= 0.f)
            {
                outHit.t = t;
                outHit.pos = p;
                outHit.normal = normal;
                return true;
            }
        }
    }

    return false;
}

// --------------------------------------------------------------------------
//  bounding volume hierarchy

struct BVHNode
{
    AABB box;
    int left;
    int right;
    bool leftLeaf;
    bool rightLeaf;
};

// Scene information: a copy of the input triangles, and the BVH
static int s_TriangleCount;
static Triangle* s_Triangles;
static std::vector<BVHNode> s_BVH;

static uint32_t XorShift32(uint32_t& state)
{
    uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 15;
    state = x;
    return x;
}

static int CreateBVH(int* triIndices, int triCount, uint32_t& rngState)
{
    // sort input triangles by a randomly chosen axis
    int axis = XorShift32(rngState) % 3;
    if (axis == 0)
        std::sort(triIndices, triIndices + triCount, [](int a, int b)
                  {
                      assert(a >= 0 && a < s_TriangleCount);
                      assert(b >= 0 && b < s_TriangleCount);
                      AABB boxa = AABBOfTriangle(s_Triangles[a]);
                      AABB boxb = AABBOfTriangle(s_Triangles[b]);
                      return boxa.bmin.x < boxb.bmin.x;
                  });
    else if (axis == 1)
        std::sort(triIndices, triIndices + triCount, [](int a, int b)
                  {
                      assert(a >= 0 && a < s_TriangleCount);
                      assert(b >= 0 && b < s_TriangleCount);
                      AABB boxa = AABBOfTriangle(s_Triangles[a]);
                      AABB boxb = AABBOfTriangle(s_Triangles[b]);
                      return boxa.bmin.y < boxb.bmin.y;
                  });
    else if (axis == 2)
        std::sort(triIndices, triIndices + triCount, [](int a, int b)
                  {
                      assert(a >= 0 && a < s_TriangleCount);
                      assert(b >= 0 && b < s_TriangleCount);
                      AABB boxa = AABBOfTriangle(s_Triangles[a]);
                      AABB boxb = AABBOfTriangle(s_Triangles[b]);
                      return boxa.bmin.z < boxb.bmin.z;
                  });

    // create the node
    BVHNode node;
    int nodeIndex = (int)s_BVH.size();
    s_BVH.push_back(node);
    AABB boxLeft, boxRight;
    if (triCount == 1)
    {
        node.left = node.right = triIndices[0];
        node.leftLeaf = node.rightLeaf = true;
        assert(triIndices[0] >= 0 && triIndices[0] < s_TriangleCount);
        boxLeft = boxRight = AABBOfTriangle(s_Triangles[triIndices[0]]);
    }
    else if (triCount == 2)
    {
        node.left = triIndices[0];
        node.right = triIndices[1];
        node.leftLeaf = node.rightLeaf = true;
        assert(triIndices[0] >= 0 && triIndices[0] < s_TriangleCount);
        assert(triIndices[1] >= 0 && triIndices[1] < s_TriangleCount);
        boxLeft = AABBOfTriangle(s_Triangles[triIndices[0]]);
        boxRight = AABBOfTriangle(s_Triangles[triIndices[1]]);
    }
    else
    {
        node.left = CreateBVH(triIndices, triCount / 2, rngState);
        node.right = CreateBVH(triIndices + triCount / 2, triCount - triCount / 2, rngState);
        node.leftLeaf = node.rightLeaf = false;
        assert(node.left >= 0 && node.left < s_BVH.size());
        assert(node.right >= 0 && node.right < s_BVH.size());
        boxLeft = s_BVH[node.left].box;
        boxRight = s_BVH[node.right].box;
    }
    node.box = AABBUnion(boxLeft, boxRight);
    s_BVH[nodeIndex] = node;
    return nodeIndex;
}

void InitializeScene(int triangleCount, const Triangle* triangles)
{
    s_TriangleCount = triangleCount;
    s_Triangles = new Triangle[triangleCount];
    memcpy(s_Triangles, triangles, triangleCount * sizeof(triangles[0]));
    
    // build BVH
#if USE_BVH
    int* triIndices = new int[triangleCount];
    for (int i = 0; i < triangleCount; ++i)
        triIndices[i] = i;
    uint32_t rngState = 1;
    CreateBVH(triIndices, triangleCount, rngState);
    delete[] triIndices;
#endif
}

void CleanupScene()
{
    delete[] s_Triangles;
    s_BVH.clear();
}

static int HitBVH(int index, bool leaf, const Ray& r, float tMin, float tMax, Hit& outHit)
{
    // if leaf node, check against a triangle
    if (leaf)
    {
        assert(index >= 0 && index < s_TriangleCount);
        if (HitTriangle(r, s_Triangles[index], tMin, tMax, outHit))
            return index;
        return -1;
    }
    
    // not a leaf node; check if ray hits us at all
    const BVHNode& node = s_BVH[index];
    if (!HitAABB(r, node.box, tMin, tMax))
        return -1;
    
    Hit leftHit, rightHit;
    int leftId = HitBVH(node.left, node.leftLeaf, r, tMin, tMax, leftHit);
    int rightId = HitBVH(node.right, node.rightLeaf, r, tMin, tMax, rightHit);
    if (leftId != -1 && rightId != -1)
    {
        // both are hit, return closest one
        if (leftHit.t < rightHit.t)
        {
            outHit = leftHit;
            return leftId;
        }
        else
        {
            outHit = rightHit;
            return rightId;
        }
    }
    if (leftId != -1)
    {
        // only left was hit
        outHit = leftHit;
        return leftId;
    }
    if (rightId != -1)
    {
        // only right was hit
        outHit = rightHit;
        return rightId;
    }
    return -1;
}


// Check all the triangles in the scene for a hit, and return the closest one.
int HitScene(const Ray& r, float tMin, float tMax, Hit& outHit)
{
#if USE_BVH
    
    if (s_BVH.empty())
        return -1;
    
    return HitBVH(0, false, r, tMin, tMax, outHit);
    
#else

    float hitMinT = tMax;
    int hitID = -1;
    for (int i = 0; i < s_TriangleCount; ++i)
    {
        Hit hit;
        if (HitTriangle(r, s_Triangles[i], tMin, tMax, hit))
        {
            if (hit.t < hitMinT)
            {
                hitMinT = hit.t;
                hitID = i;
                outHit = hit;
            }
        }
    }

    return hitID;
#endif
}
