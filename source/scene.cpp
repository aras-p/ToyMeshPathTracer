#include "scene.h"
#include <algorithm>
#include <string.h>
#include <vector>
#include <utility>

// Use our own simple BVH implementation to speed up ray queries?
#define USE_BVH 1

// Use Intel Embree for BVH and all ray queries?
#define USE_EMBREE 0
#if USE_EMBREE
#include "external/embree3/rtcore.h"
#endif

// --------------------------------------------------------------------------
// Axis-aligned bounding box and related functions

#if USE_BVH && !USE_EMBREE
struct AABB
{
    float3 bmin;
    float3 bmax;
};

// from "A Ray-Box Intersection Algorithm and Efficient Dynamic Voxel Rendering"
// http://jcgt.org/published/0007/03/04/
// note: ray direction should be inverted, i.e 1.0/direction!
static bool HitAABB(const Ray& r, const AABB& box, float tMin, float tMax)
{
    float3 t0 = (box.bmin - r.orig) * r.dir;
    float3 t1 = (box.bmax - r.orig) * r.dir;
    
    float3 tsmaller = min(t0, t1);
    float3 tbigger  = max(t0, t1);
    
    tMin = std::max(tMin, hmax(tsmaller));
    tMax = std::min(tMax, hmin(tbigger));
    
    return tMin <= tMax;
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
#endif // #if USE_BVH && !USE_EMBREE


// --------------------------------------------------------------------------
// Checks if one triangle is hit by a ray segment.
// based on "The Graphics Codex"

#if !USE_EMBREE
static bool HitTriangle(const Ray& r, const Triangle& tri, float tMin, float tMax, Hit& outHit)
{
    float3 e1 = tri.v1 - tri.v0;
    float3 e2 = tri.v2 - tri.v0;
    float3 p = cross(r.dir, e2);
    float a = dot(e1, p);
    if (fabs(a) < 1e-5f)
        return false; // parallel to the plane
    
    float f = 1.0f / a;
    float3 s = r.orig - tri.v0;
    float u = f * dot(s, p);
    
    if (u < 0.0f || u > 1.0f)
        return false; // but outside the triangle
    
    float3 q = cross(s, e1);
    float v = f * dot(r.dir, q);
    
    if (v < 0.0f || (u + v) > 1.0f)
        return false; // but outside the triangle
    
    float t = f * dot(e2, q);
    
    if (t > tMin && t < tMax)
    {
        outHit.t = t;
        outHit.pos = r.pointAt(t);
        outHit.normal = normalize(cross(e1, e2));
        return true;
    }
    return false;
}

static bool HitTriangleShadow(const Ray& r, const Triangle& tri, float tMin, float tMax)
{
    float3 e1 = tri.v1 - tri.v0;
    float3 e2 = tri.v2 - tri.v0;
    float3 p = cross(r.dir, e2);
    float a = dot(e1, p);
    if (fabs(a) < 1e-5f)
        return false; // parallel to the plane
    
    float f = 1.0f / a;
    float3 s = r.orig - tri.v0;
    float u = f * dot(s, p);
    
    if (u < 0.0f || u > 1.0f)
        return false; // but outside the triangle
    
    float3 q = cross(s, e1);
    float v = f * dot(r.dir, q);
    
    if (v < 0.0f || (u + v) > 1.0f)
        return false; // but outside the triangle
    
    float t = f * dot(e2, q);
    
    if (t > tMin && t < tMax)
        return true;
    return false;
}
#endif // #if !USE_EMBREE


// --------------------------------------------------------------------------
//  bounding volume hierarchy

#if USE_BVH && !USE_EMBREE
struct BVHNode
{
    AABB box;
    int left;
    int right;
    bool leftLeaf;
    bool rightLeaf;
};
#endif // #if USE_BVH && !USE_EMBREE

// Scene information: a copy of the input triangles
static int s_TriangleCount;
static Triangle* s_Triangles;
#if USE_BVH && !USE_EMBREE
static std::vector<BVHNode> s_BVH;
#endif

#if USE_EMBREE
static RTCDevice s_Device;
static RTCScene s_Scene;
#endif

#if USE_BVH && !USE_EMBREE
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
                      return boxa.bmin.getX() < boxb.bmin.getX();
                  });
    else if (axis == 1)
        std::sort(triIndices, triIndices + triCount, [](int a, int b)
                  {
                      assert(a >= 0 && a < s_TriangleCount);
                      assert(b >= 0 && b < s_TriangleCount);
                      AABB boxa = AABBOfTriangle(s_Triangles[a]);
                      AABB boxb = AABBOfTriangle(s_Triangles[b]);
                      return boxa.bmin.getY() < boxb.bmin.getY();
                  });
    else if (axis == 2)
        std::sort(triIndices, triIndices + triCount, [](int a, int b)
                  {
                      assert(a >= 0 && a < s_TriangleCount);
                      assert(b >= 0 && b < s_TriangleCount);
                      AABB boxa = AABBOfTriangle(s_Triangles[a]);
                      AABB boxb = AABBOfTriangle(s_Triangles[b]);
                      return boxa.bmin.getZ() < boxb.bmin.getZ();
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
#endif // #if USE_BVH && !USE_EMBREE

void InitializeScene(int triangleCount, const Triangle* triangles)
{
    s_TriangleCount = triangleCount;
    s_Triangles = new Triangle[triangleCount];
    memcpy(s_Triangles, triangles, triangleCount * sizeof(triangles[0]));
    
#if USE_EMBREE
    s_Device = rtcNewDevice("threads=1");
    s_Scene = rtcNewScene(s_Device);
    
    RTCGeometry mesh = rtcNewGeometry (s_Device, RTC_GEOMETRY_TYPE_TRIANGLE);
    float* dstVerts = (float*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 12, triangleCount*3);
    int* indices = (int*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 12, triangleCount);
    for (int i = 0; i < triangleCount; ++i)
    {
        memcpy(dstVerts+i*9+0, &triangles[i].v0, 12);
        memcpy(dstVerts+i*9+3, &triangles[i].v1, 12);
        memcpy(dstVerts+i*9+6, &triangles[i].v2, 12);
        indices[i*3+0] = i*3+0;
        indices[i*3+1] = i*3+1;
        indices[i*3+2] = i*3+2;
    }
    rtcCommitGeometry(mesh);
    rtcAttachGeometry(s_Scene, mesh);
    rtcReleaseGeometry(mesh);
    
    rtcCommitScene(s_Scene);
#endif // #if USE_EMBREE
    
#if USE_BVH && !USE_EMBREE
    // build BVH
    int* triIndices = new int[triangleCount];
    for (int i = 0; i < triangleCount; ++i)
        triIndices[i] = i;
    uint32_t rngState = 1;
    CreateBVH(triIndices, triangleCount, rngState);
    delete[] triIndices;
#endif // #if USE_BVH && !USE_EMBREE
}

void CleanupScene()
{
    delete[] s_Triangles;
#if USE_BVH && !USE_EMBREE
    s_BVH.clear();
#endif
#if USE_EMBREE
    rtcReleaseScene(s_Scene);
    rtcReleaseDevice(s_Device);
#endif
}

#if USE_BVH && !USE_EMBREE
static int HitBVH(int index, bool leaf, const Ray& r, const Ray& invR, float tMin, float tMax, Hit& outHit)
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
    if (!HitAABB(invR, node.box, tMin, tMax))
        return -1;
    
    int leftId = HitBVH(node.left, node.leftLeaf, r, invR, tMin, tMax, outHit);
    if (leftId != -1)
    {
        // left was hit: only check right hit up until left hit distance
        int rightId = HitBVH(node.right, node.rightLeaf, r, invR, tMin, outHit.t, outHit);
        if (rightId != -1)
            return rightId;
        return leftId;
    }
    // left was not hit: check right
    int rightId = HitBVH(node.right, node.rightLeaf, r, invR, tMin, tMax, outHit);
    return rightId;
}

static bool HitShadowBVH(int index, bool leaf, const Ray& r, const Ray& invR, float tMin, float tMax)
{
    // if leaf node, check against a triangle
    if (leaf)
    {
        assert(index >= 0 && index < s_TriangleCount);
        return HitTriangleShadow(r, s_Triangles[index], tMin, tMax);
    }
    
    // not a leaf node; check if ray hits us at all
    const BVHNode& node = s_BVH[index];
    if (!HitAABB(invR, node.box, tMin, tMax))
        return false;
    
    if (HitShadowBVH(node.left, node.leftLeaf, r, invR, tMin, tMax))
        return true;
    if (HitShadowBVH(node.right, node.rightLeaf, r, invR, tMin, tMax))
        return true;
    return false;
}
#endif // #if USE_BVH && !USE_EMBREE


// Check all the triangles in the scene for a hit, and return the closest one.
int HitScene(const Ray& r, float tMin, float tMax, Hit& outHit)
{
#if USE_EMBREE
    RTCIntersectContext ctx;
    rtcInitIntersectContext(&ctx);
    
    RTCRayHit rh;
    r.orig.store(&rh.ray.org_x);
    rh.ray.tnear = tMin;
    r.dir.store(&rh.ray.dir_x);
    rh.ray.time = 0;
    rh.ray.tfar = tMax;
    rh.ray.mask = 0;
    rh.ray.id = 0;
    rh.ray.flags = 0;
    rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rh.hit.primID = RTC_INVALID_GEOMETRY_ID;
    
    rtcIntersect1(s_Scene, &ctx, &rh);
    if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID)
        return -1;
    outHit.t = rh.ray.tfar;
    outHit.pos = r.pointAt(outHit.t);
    outHit.normal = normalize(float3(rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z));
    return rh.hit.primID;
    
#elif USE_BVH
    
    if (s_BVH.empty())
        return -1;
    
    Ray invR = r;
    invR.dir = float3(1.0f) / r.dir;
    return HitBVH(0, false, r, invR, tMin, tMax, outHit);
    
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

bool HitSceneShadow(const Ray& r, float tMin, float tMax)
{
#if USE_EMBREE
    RTCIntersectContext ctx;
    rtcInitIntersectContext(&ctx);
    
    RTCRay rh;
    r.orig.store(&rh.org_x);
    rh.tnear = tMin;
    r.dir.store(&rh.dir_x);
    rh.time = 0;
    rh.tfar = tMax;
    rh.mask = 0;
    rh.id = 0;
    rh.flags = 0;
    
    rtcOccluded1(s_Scene, &ctx, &rh);
    return rh.tfar < 0;
    
#elif USE_BVH
    if (s_BVH.empty())
        return false;

    Ray invR = r;
    invR.dir = float3(1.0f) / r.dir;
    return HitShadowBVH(0, false, r, invR, tMin, tMax);

#else
    for (int i = 0; i < s_TriangleCount; ++i)
    {
        if (HitTriangleShadow(r, s_Triangles[i], tMin, tMax))
            return true;
    }
    return false;

#endif
}
