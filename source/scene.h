#pragma once

// Scene: this represents all the scene geometry that the ray tracer works on.

#include "maths.h"


// One triangle: just three vertex positions.
struct Triangle
{
	float3 v0, v1, v2;
};


// Our scene structure is very simple: just a bunch of triangles and nothing else
// (no "objects", "instances" or "materials").
void InitializeScene(int triangleCount, const Triangle* triangles);

// Cleanup any data or memory that the scene might have allocated.
void CleanupScene();

// Checks if the ray segment hits a scene. If any triangle is hit by the ray, this
// function should return information about the closest one.
//
// - r: the ray itself,
// - tMin and tMax: segment of the ray that is checked,
// - outHit: hit information, if any,
//
// Function returns the triangle index, or -1 if nothing is hit by the ray.
int HitScene(const Ray& r, float tMin, float tMax, Hit& outHit);
