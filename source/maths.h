#pragma once

// --------------------------------------------------------------------------
// various math utilities

#define NOMINMAX
#include <cmath>
#include <assert.h>
#include <stdint.h>


#define kPI 3.1415926f


// --------------------------------------------------------------------------
// simple 3D vector with x,y,z components

struct float3
{
    float3() : x(0), y(0), z(0) {}
    float3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    float3 operator-() const { return float3(-x, -y, -z); }
    float3& operator+=(const float3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    float3& operator-=(const float3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    float3& operator*=(const float3& o) { x*=o.x; y*=o.y; z*=o.z; return *this; }
    float3& operator*=(float o) { x*=o; y*=o; z*=o; return *this; }

    inline float getX() const { return x; }
    inline float getY() const { return y; }
    inline float getZ() const { return z; }
    inline void setX(float x_) { x = x_; }
    inline void setY(float y_) { y = y_; }
    inline void setZ(float z_) { z = z_; }
    inline void store(float *p) const { p[0] = getX(); p[1] = getY(); p[2] = getZ(); }

    float x, y, z;
};

inline float3 operator+(const float3& a, const float3& b) { return float3(a.x+b.x,a.y+b.y,a.z+b.z); }
inline float3 operator-(const float3& a, const float3& b) { return float3(a.x-b.x,a.y-b.y,a.z-b.z); }
inline float3 operator*(const float3& a, const float3& b) { return float3(a.x*b.x,a.y*b.y,a.z*b.z); }
inline float3 operator*(const float3& a, float b) { return float3(a.x*b,a.y*b,a.z*b); }
inline float3 operator*(float a, const float3& b) { return float3(a*b.x,a*b.y,a*b.z); }

inline float dot(const float3& a, const float3& b) { return a.x*b.x+a.y*b.y+a.z*b.z; }

inline float3 cross(const float3& a, const float3& b)
{
    return float3(a.y*b.z - a.z*b.y, -(a.x*b.z - a.z*b.x), a.x*b.y - a.y*b.x);
}

inline float3 min(const float3& a, const float3& b)
{
    return float3(fmin(a.x,b.x), fmin(a.y,b.y), fmin(a.z,b.z));
}
inline float3 max(const float3& a, const float3& b)
{
    return float3(fmax(a.x,b.x), fmax(a.y,b.y), fmax(a.z,b.z));
}

inline float length(float3 v) { return sqrtf(dot(v, v)); }
inline float sqLength(float3 v) { return dot(v, v); }
inline float3 normalize(float3 v) { return v * (1.0f / length(v)); }

inline float saturate(float v) { if (v < 0) return 0; if (v > 1) return 1; return v; }


inline void AssertUnit(float3 v)
{
    (void)v;
    assert(fabsf(sqLength(v) - 1.0f) < 0.01f);
}


// --------------------------------------------------------------------------
// ray: starting position (origin) and direction.
// direction is assumed to be normalized

struct Ray
{
    Ray() {}
    Ray(float3 orig_, float3 dir_) : orig(orig_), dir(dir_) { AssertUnit(dir); }

    float3 pointAt(float t) const { return orig + dir * t; }

    float3 orig;
    float3 dir;
};


// --------------------------------------------------------------------------
// ray hit point information: position where it hit something;
// normal of the surface that was hit, and "t" position along the ray

struct Hit
{
    float3 pos;
    float3 normal;
    float t;
};


// --------------------------------------------------------------------------
// random number generator utilities

float RandomFloat01(uint32_t& state);
float3 RandomInUnitDisk(uint32_t& state);
float3 RandomUnitVector(uint32_t& state);


// --------------------------------------------------------------------------
// camera

struct Camera
{
    Camera() {}

    // vfov is top to bottom in degrees
    Camera(const float3& lookFrom, const float3& lookAt, const float3& vup, float vfov, float aspect, float aperture, float focusDist)
    {
        lensRadius = aperture / 2;
        float theta = vfov*kPI/180;
        float halfHeight = tanf(theta/2);
        float halfWidth = aspect * halfHeight;
        origin = lookFrom;
        w = normalize(lookFrom - lookAt);
        u = normalize(cross(vup, w));
        v = cross(w, u);
        lowerLeftCorner = origin - halfWidth*focusDist*u - halfHeight*focusDist*v - focusDist*w;
        horizontal = 2*halfWidth*focusDist*u;
        vertical = 2*halfHeight*focusDist*v;
    }

    Ray GetRay(float s, float t, uint32_t& state) const
    {
        float3 rd = lensRadius * RandomInUnitDisk(state);
        float3 offset = u * rd.getX() + v * rd.getY();
        return Ray(origin + offset, normalize(lowerLeftCorner + s*horizontal + t*vertical - origin - offset));
    }

    float3 origin;
    float3 lowerLeftCorner;
    float3 horizontal;
    float3 vertical;
    float3 u, v, w;
    float lensRadius;
};

