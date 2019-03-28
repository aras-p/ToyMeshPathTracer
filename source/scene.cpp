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

// Use NanoRT for BVH and all ray queries?
#define USE_NANORT 0
#if USE_NANORT
#include "external/nanort.h"
#endif


#if USE_EMBREE || USE_NANORT
#undef USE_BVH
#endif

// --------------------------------------------------------------------------
// Axis-aligned bounding box and related functions

#if USE_BVH
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
#endif // #if USE_BVH


// --------------------------------------------------------------------------
// Checks if one triangle is hit by a ray segment.
// based on "The Graphics Codex"

#if !USE_EMBREE && !USE_NANORT
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
#endif // #if !USE_EMBREE && !USE_NANORT


// --------------------------------------------------------------------------
//  bounding volume hierarchy

#if USE_BVH
struct BVHNode
{
    AABB box;
    int data1; // node: left index; leaf: start triangle index
    int data2; // node: right index; leaf: triangle count
    bool leaf;
};
#endif // #if USE_BVH

// Scene information: a copy of the input triangles
static int s_TriangleCount;
static Triangle* s_Triangles;
static int* s_TriIndices;
#if USE_BVH
static std::vector<BVHNode> s_BVH;
#endif

#if USE_EMBREE
static RTCDevice s_Device;
static RTCScene s_Scene;
#endif

#if USE_NANORT
static unsigned int* s_Indices;
static nanort::BVHAccel<float> s_BVH;
static nanort::TriangleMesh<float>* s_Mesh;
#endif

#if USE_BVH
static uint32_t XorShift32(uint32_t& state)
{
    uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 15;
    state = x;
    return x;
}

static int CreateBVH(int triStart, int triCount, uint32_t& rngState)
{
    // sort input triangles by a randomly chosen axis
    int axis = XorShift32(rngState) % 3;
    if (axis == 0)
        std::sort(s_TriIndices+triStart, s_TriIndices+triStart + triCount, [](int a, int b)
                  {
                      assert(a >= 0 && a < s_TriangleCount);
                      assert(b >= 0 && b < s_TriangleCount);
                      AABB boxa = AABBOfTriangle(s_Triangles[a]);
                      AABB boxb = AABBOfTriangle(s_Triangles[b]);
                      return boxa.bmin.getX() < boxb.bmin.getX();
                  });
    else if (axis == 1)
        std::sort(s_TriIndices+triStart, s_TriIndices+triStart + triCount, [](int a, int b)
                  {
                      assert(a >= 0 && a < s_TriangleCount);
                      assert(b >= 0 && b < s_TriangleCount);
                      AABB boxa = AABBOfTriangle(s_Triangles[a]);
                      AABB boxb = AABBOfTriangle(s_Triangles[b]);
                      return boxa.bmin.getY() < boxb.bmin.getY();
                  });
    else if (axis == 2)
        std::sort(s_TriIndices+triStart, s_TriIndices+triStart + triCount, [](int a, int b)
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

    // if we have less than N triangles, make this node a leaf that just has all of them
    if (triCount <= 4)
    {
        node.data1 = triStart;
        node.data2 = triCount;
        node.leaf = true;
        node.box = AABBOfTriangle(s_Triangles[s_TriIndices[triStart]]);
        for (int i = 1; i < triCount; ++i)
        {
            auto tribox = AABBOfTriangle(s_Triangles[s_TriIndices[triStart+i]]);
            node.box = AABBUnion(node.box, tribox);
        }
    }
    else
    {
        node.data1 = CreateBVH(triStart, triCount / 2, rngState);
        node.data2 = CreateBVH(triStart + triCount / 2, triCount - triCount / 2, rngState);
        node.leaf = false;
        assert(node.data1 >= 0 && node.data1 < s_BVH.size());
        assert(node.data2 >= 0 && node.data2 < s_BVH.size());
        node.box = AABBUnion(s_BVH[node.data1].box, s_BVH[node.data2].box);
    }
    s_BVH[nodeIndex] = node;
    return nodeIndex;
}
#endif // #if USE_BVH

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
    
#elif USE_NANORT

    nanort::BVHBuildOptions<float> buildOptions;
    buildOptions.cache_bbox = false;

    s_Indices = new unsigned int[triangleCount*3];
    for (int i = 0; i < triangleCount*3; ++i)
        s_Indices[i] = i;
    s_Mesh = new nanort::TriangleMesh<float>((const float*)s_Triangles, s_Indices, sizeof(float3));
    nanort::TriangleSAHPred<float> pred((const float*)s_Triangles, s_Indices, sizeof(float3));
    s_BVH.Build(triangleCount, *s_Mesh, pred, buildOptions);

#elif USE_BVH
    
    // build BVH
    s_TriIndices = new int[triangleCount];
    for (int i = 0; i < triangleCount; ++i)
        s_TriIndices[i] = i;
    uint32_t rngState = 1;
    CreateBVH(0, triangleCount, rngState);
#endif
}

void CleanupScene()
{
    delete[] s_Triangles;
#if USE_EMBREE
    rtcReleaseScene(s_Scene);
    rtcReleaseDevice(s_Device);
#elif USE_NANORT
    delete s_Mesh;
    delete[] s_Indices;
#elif USE_BVH
    s_BVH.clear();
    delete[] s_TriIndices;
#endif
}

#if USE_BVH
static int HitBVH(int index, const Ray& r, const Ray& invR, float tMin, float tMax, Hit& outHit)
{
    // check if ray hits us at all
    const BVHNode& node = s_BVH[index];
    if (!HitAABB(invR, node.box, tMin, tMax))
        return -1;

    // if leaf node, check against triangles
    if (node.leaf)
    {
        int hitID = -1;
        for (int i = 0; i < node.data2; ++i)
        {
            int triIndex = s_TriIndices[node.data1 + i];
            assert(triIndex >= 0 && triIndex < s_TriangleCount);
            if (HitTriangle(r, s_Triangles[triIndex], tMin, tMax, outHit))
            {
                hitID = triIndex;
                tMax = outHit.t;
            }
        }
        return hitID;
    }
    
    // not a leaf node, go into child nodes
    int leftId = HitBVH(node.data1, r, invR, tMin, tMax, outHit);
    if (leftId != -1)
    {
        // left was hit: only check right hit up until left hit distance
        int rightId = HitBVH(node.data2, r, invR, tMin, outHit.t, outHit);
        if (rightId != -1)
            return rightId;
        return leftId;
    }
    // left was not hit: check right
    int rightId = HitBVH(node.data2, r, invR, tMin, tMax, outHit);
    return rightId;
}

static bool HitShadowBVH(int index, const Ray& r, const Ray& invR, float tMin, float tMax)
{
    // check if ray hits us at all
    const BVHNode& node = s_BVH[index];
    if (!HitAABB(invR, node.box, tMin, tMax))
        return false;
    
    // if leaf node, check against triangles
    if (node.leaf)
    {
        for (int i = 0; i < node.data2; ++i)
        {
            int triIndex = s_TriIndices[node.data1 + i];
            assert(triIndex >= 0 && triIndex < s_TriangleCount);
            if (HitTriangleShadow(r, s_Triangles[triIndex], tMin, tMax))
                return true;
        }
        return false;
    }

    if (HitShadowBVH(node.data1, r, invR, tMin, tMax))
        return true;
    if (HitShadowBVH(node.data2, r, invR, tMin, tMax))
        return true;
    return false;
}
#endif // #if USE_BVH


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
    
#elif USE_NANORT
    nanort::Ray<float> ray;
    ray.min_t = tMin;
    ray.max_t = tMax;
    r.orig.store(ray.org);
    r.dir.store(ray.dir);
    
    nanort::TriangleIntersector<> intersector((const float*)s_Triangles, s_Indices, sizeof(float3));
    nanort::TriangleIntersection<> isect;
    bool hit = s_BVH.Traverse(ray, intersector, &isect);
    if (!hit)
        return -1;

    outHit.t = isect.t;
    outHit.pos = r.pointAt(isect.t);
    const Triangle& tri = s_Triangles[isect.prim_id];
    
    float3 e1 = tri.v1 - tri.v0;
    float3 e2 = tri.v2 - tri.v0;
    float3 n = normalize(cross(e1,e2));
    outHit.normal = n;
    return isect.prim_id;
    
#elif USE_BVH
    
    if (s_BVH.empty())
        return -1;
    
    Ray invR = r;
    invR.dir = float3(1.0f) / r.dir;
    return HitBVH(0, r, invR, tMin, tMax, outHit);
    
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
    
#elif USE_NANORT
    nanort::Ray<float> ray;
    ray.min_t = tMin;
    ray.max_t = tMax;
    r.orig.store(ray.org);
    r.dir.store(ray.dir);
    
    nanort::TriangleIntersector<> intersector((const float*)s_Triangles, s_Indices, sizeof(float3));
    nanort::TriangleIntersection<> isect;
    return s_BVH.Traverse(ray, intersector, &isect);

#elif USE_BVH
    if (s_BVH.empty())
        return false;

    Ray invR = r;
    invR.dir = float3(1.0f) / r.dir;
    return HitShadowBVH(0, r, invR, tMin, tMax);

#else
    for (int i = 0; i < s_TriangleCount; ++i)
    {
        if (HitTriangleShadow(r, s_Triangles[i], tMin, tMax))
            return true;
    }
    return false;

#endif
}
