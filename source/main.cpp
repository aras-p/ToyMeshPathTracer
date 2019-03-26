// main program entry point and the actual raytracing bits

#include "maths.h"
#include "scene.h"


// Include external libraries:
// - PNG writing
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "external/stb_image_write.h"
// - time measurement
#define SOKOL_IMPL
#include "external/sokol_time.h"
// - OBJ file loading
#include "external/objparser.h"


// --------------------------------------------------------------------------
// "ray/path tracing" bits

// general minimum/maximum distances for rays (from "very close to surface but not exacttly on it"
// to "ten million units")
const float kMinT = 0.001f;
const float kMaxT = 1.0e7f;
// maximum raytracing recursion depth, i.e. number of light bounces
const int kMaxDepth = 10;

// we have one hardcoded directional light, with this direction and color
static const float3 kLightDir = normalize(float3(-0.7f,1.0f,0.5f));
static const float3 kLightColor = float3(0.7f,0.6f,0.5f);


// when a ray "r" has just hit a surface at point "hit", decide what to do about it:
// in our very simple case, we assume the surface is perfectly diffuse, so we'll return:
// - surface albedo ("color") in "attenuation"
// - new random ray for the next light bounce in "scattered"
// - illumination from the directional light in "outLightE"
static bool Scatter(const Ray& r, const Hit& hit, float3& attenuation, Ray& scattered, float3& outLightE, uint32_t& rngState, int& inoutRayCount)
{
    outLightE = float3(0,0,0);

    // model a perfectly diffuse material:
    
    // random point on unit sphere that is tangent to the hit point
    float3 target = hit.pos + hit.normal + RandomUnitVector(rngState);
    scattered = Ray(hit.pos, normalize(target - hit.pos));
    
    // make color slightly based on surface normals
    float3 albedo = hit.normal * 0.0f + float3(0.7f,0.7f,0.7f);
    attenuation = albedo;
    
    // explicit directional light by shooting a shadow ray
    ++inoutRayCount;
    Hit lightHit;
    int id = HitScene(Ray(hit.pos, kLightDir), kMinT, kMaxT, lightHit);
    if (id == -1)
    {
        // ray towards the light did not hit anything in the scene, so
        // that means we are not in shadow: compute illumination from it
        float3 rdir = r.dir;
        AssertUnit(rdir);
        float3 nl = dot(hit.normal, rdir) < 0 ? hit.normal : -hit.normal;
        outLightE += albedo * kLightColor * (fmax(0.0f, dot(kLightDir, nl)));
    }

    return true;
}


// trace a ray into the scene, and return the final color for it
static float3 Trace(const Ray& r, int depth, uint32_t& rngState, int& inoutRayCount)
{
    ++inoutRayCount;
    Hit hit;
    int id = HitScene(r, kMinT, kMaxT, hit);
    if (id != -1)
    {
        // ray hits something in the scene
        Ray scattered;
        float3 attenuation;
        float3 lightE;
        if (depth < kMaxDepth && Scatter(r, hit, attenuation, scattered, lightE, rngState, inoutRayCount))
        {
            // we got a new ray bounced from the surface; recursively trace it
            return lightE + attenuation * Trace(scattered, depth+1, rngState, inoutRayCount);
        }
        else
        {
            // reached recursion limit, or surface fully absorbed the ray: return black
            return float3(0,0,0);
        }
    }
    else
    {
        // ray does not hit anything: return illumination from the sky (just a simple gradient really)
        float3 unitDir = r.dir;
        float t = 0.5f*(unitDir.getY() + 1.0f);
        return ((1.0f - t)*float3(1.0f, 1.0f, 1.0f) + t * float3(0.5f, 0.7f, 1.0f)) * 0.5f;
    }
}


// load scene from an .OBJ file
static bool LoadScene(const char* dataFile, float3& outBoundsMin, float3& outBoundsMax)
{
    ObjFile objFile;
    if (!objParseFile(objFile, dataFile))
    {
        printf("ERROR: failed to load .obj file\n");
        return false;
    }
    outBoundsMin = float3(+1.0e6f, +1.0e6f, +1.0e6f);
    outBoundsMax = float3(-1.0e6f, -1.0e6f, -1.0e6f);

    int objTriCount = int(objFile.f_size / 9);
    Triangle* tris = new Triangle[objTriCount + 2]; // will add two triangles for the "floor"
    for (int i = 0; i < objTriCount; ++i)
    {
        int idx0 = objFile.f[i * 9 + 0] * 3;
        int idx1 = objFile.f[i * 9 + 3] * 3;
        int idx2 = objFile.f[i * 9 + 6] * 3;
        float3 v0 = float3(objFile.v[idx0 + 0], objFile.v[idx0 + 1], objFile.v[idx0 + 2]);
        float3 v1 = float3(objFile.v[idx1 + 0], objFile.v[idx1 + 1], objFile.v[idx1 + 2]);
        float3 v2 = float3(objFile.v[idx2 + 0], objFile.v[idx2 + 1], objFile.v[idx2 + 2]);
        tris[i].v0 = v0;
        tris[i].v1 = v1;
        tris[i].v2 = v2;
        outBoundsMin = min(outBoundsMin, v0); outBoundsMax = max(outBoundsMax, v0);
        outBoundsMin = min(outBoundsMin, v1); outBoundsMax = max(outBoundsMax, v1);
        outBoundsMin = min(outBoundsMin, v2); outBoundsMax = max(outBoundsMax, v2);
    }

    // add two triangles that are right "under the scene" and covering larger area than the scene
    // itself, to serve as a "floor"
    float3 size = outBoundsMax - outBoundsMin;
    float3 extra = size * 0.7f;
    tris[objTriCount+0].v0 = float3(outBoundsMin.x-extra.x, outBoundsMin.y, outBoundsMin.z-extra.z);
    tris[objTriCount+0].v1 = float3(outBoundsMin.x-extra.x, outBoundsMin.y, outBoundsMax.z+extra.z);
    tris[objTriCount+0].v2 = float3(outBoundsMax.x+extra.x, outBoundsMin.y, outBoundsMin.z-extra.z);
    tris[objTriCount+1].v0 = float3(outBoundsMin.x-extra.x, outBoundsMin.y, outBoundsMax.z+extra.z);
    tris[objTriCount+1].v1 = float3(outBoundsMax.x+extra.x, outBoundsMin.y, outBoundsMax.z+extra.z);
    tris[objTriCount+1].v2 = float3(outBoundsMax.x+extra.x, outBoundsMin.y, outBoundsMin.z-extra.z);

    uint64_t t0 = stm_now();
    InitializeScene(objTriCount + 2, tris);
    printf("Initialized scene '%s' (%i tris) in %.3fs\n", dataFile, objTriCount+2, stm_sec(stm_since(t0)));

    delete[] tris;
    return true;
}

struct TraceData
{
    int screenWidth, screenHeight, samplesPerPixel;
    uint8_t* image;
    const Camera* camera;
    int rayCount;
};

static void TraceImage(TraceData& data)
{
    uint8_t* image = data.image;
    float invWidth = 1.0f / data.screenWidth;
    float invHeight = 1.0f / data.screenHeight;

    int rayCount = 0;
    // go over the image: each pixel row
    for (uint32_t y = 0; y < data.screenHeight; ++y)
    {
        // go over the image: each pixel in the row
        uint32_t rngState = y * 9781 + 1;
        for (int x = 0; x < data.screenWidth; ++x)
        {
            float3 col(0, 0, 0);
            // we'll trace N slightly jittered rays for each pixel, to get anti-aliasing, loop over them here
            for (int s = 0; s < data.samplesPerPixel; s++)
            {
                // get a ray from camera, and trace it
                float u = float(x + RandomFloat01(rngState)) * invWidth;
                float v = float(y + RandomFloat01(rngState)) * invHeight;
                Ray r = data.camera->GetRay(u, v, rngState);
                col += Trace(r, 0, rngState, rayCount);
            }
            col *= 1.0f / float(data.samplesPerPixel);

            // simplistic "gamma correction" by just taking a square root of the final color
            col.x = sqrtf(col.x);
            col.y = sqrtf(col.y);
            col.z = sqrtf(col.z);

            // our image is bytes in 0-255 range, turn our floats into them here and write into the image
            image[0] = uint8_t(saturate(col.x) * 255.0f);
            image[1] = uint8_t(saturate(col.y) * 255.0f);
            image[2] = uint8_t(saturate(col.z) * 255.0f);
            image[3] = 255;
            image += 4;
        }
    }
    data.rayCount += rayCount;
}


int main(int argc, const char** argv)
{
    // initialize timer
    stm_setup();

    // parse screen size command line arguments
    int screenWidth, screenHeight, samplesPerPixel;
    if (argc < 5)
    {
        printf("Usage: TrimeshTracer.exe [width] [height] [samplesPerPixel] [objFile]\n");
        return 1;
    }
    screenWidth = atoi(argv[1]);
    if (screenWidth < 1 || screenWidth > 10000)
    {
        printf("ERROR: invalid width argument '%s'\n", argv[1]);
        return 1;
    }
    screenHeight = atoi(argv[2]);
    if (screenHeight < 1 || screenHeight > 10000)
    {
        printf("ERROR: invalid height argument '%s'\n", argv[2]);
        return 1;
    }
    samplesPerPixel = atoi(argv[3]);
    if (samplesPerPixel < 1 || samplesPerPixel > 1024)
    {
        printf("ERROR: invalid samplesPerPixel argument '%s'\n", argv[3]);
        return 1;
    }

    // load model file and initialize the scene
    float3 sceneMin, sceneMax;
    if (!LoadScene(argv[4], sceneMin, sceneMax))
        return 1;

    // place a camera: put it a bit outside scene bounds, looking at the center of it
    float3 sceneSize = sceneMax - sceneMin;
    float3 sceneCenter = (sceneMin + sceneMax) * 0.5f;
    float3 lookfrom = sceneCenter + sceneSize * float3(0.3f,0.6f,1.2f);
    if (strstr(argv[4], "sponza.obj") != nullptr) // sponza looks bad when viewed from outside; hardcode camera position
        lookfrom = float3(-5.96f, 4.08f, -1.22f);
    float3 lookat = sceneCenter + sceneSize * float3(0,-0.1f,0);
    float distToFocus = length(lookfrom - lookat);
    float aperture = 0.03f;
    auto camera = Camera(lookfrom, lookat, float3(0, 1, 0), 60, float(screenWidth) / float(screenHeight), aperture, distToFocus);

    // create RGBA image for the result
    uint8_t* image = new uint8_t[screenWidth * screenHeight * 4];

    // generate the image - run TraceImage
    uint64_t t0 = stm_now();

    TraceData data;
    data.screenWidth = screenWidth;
    data.screenHeight = screenHeight;
    data.samplesPerPixel = samplesPerPixel;
    data.image = image;
    data.camera = &camera;
    data.rayCount = 0;
    TraceImage(data);

    double dt = stm_sec(stm_since(t0));
    printf("Rendered scene at %ix%i,%ispp in %.3f s\n", screenWidth, screenHeight, samplesPerPixel, dt);
    printf("- %.1f K Rays, %.1f K Rays/s\n", data.rayCount/1000.0, data.rayCount/1000.0/dt);

    // write resulting image as PNG
    stbi_flip_vertically_on_write(1);
    stbi_write_png("output.png", screenWidth, screenHeight, 4, image, screenWidth*4);

    // cleanup and exit
    delete[] image;
    CleanupScene();
    return 0;
}
