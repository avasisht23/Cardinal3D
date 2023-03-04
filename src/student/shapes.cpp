#include "../rays/shapes.h"
#include "debug.h"
#include <iostream>

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    Trace ret;

    // sphere equation is x^2 + y^2 + z^2 = R^2
    // if P = (x,y,z) then we can rewrite to say P^2 = R^2 or P^2-R^2 = 0
    // Since P is our ray, we can write (o + t * d)^2 - R^2 = 0
    // factor it out and we get o^2 + d^2 * t^2 + 2 * o*d*t - R^2 = 0
    // reordering gives us a quadratic equation ax^2 + bx + c = 0:
    //  [d^2 * t^2] + [2 * o*d*t] + [o^2 - R^2] = 0 with x = t
    // so a = d^2, b = 2 * o * d, c = o^2 - R^2

    Vec3 o = ray.point;
    Vec3 d = ray.dir;
    float r = radius;
    float a = dot(d, d);
    float b = 2 * dot(o, d);
    float c = dot(o, o) - (r * r);
    float determ = b * b - 4 * a * c;
    float t0, t1, big_t, little_t;
    Vec3 intersect;
    // with quadratic equtions if there are two roots, then b^2-4ac > 0.
    if(determ > 0) {
        // t = (-b +- sqrt(b^2 - 4ac)) / 2a
        t0 = (-b + determ) / 2 * a;
        t1 = (-b - determ) / 2 * a;

        big_t = std::max(t0, t1);
        little_t = std::min(t0, t1);

        if(big_t > ray.dist_bounds.y && little_t < ray.dist_bounds.x) {
            return ret;
        } else if(big_t <= ray.dist_bounds.y && little_t < ray.dist_bounds.x) {
            intersect = o + big_t * d;
            ret.distance = big_t;
        } else {
            intersect = o + little_t * d;
            ret.distance = little_t;
        }
    }
    // if there is only one root, then b^2-4ac = 0
    else if(determ == 0) {
        t0 = -b / 2 * a;
        if(t0 <= ray.dist_bounds.y && t0 >= ray.dist_bounds.x) {
            intersect = o + t0 * d;
            ret.distance = t0;
        }
    }
    // if there is no root, then b^2-4ac < 0
    else {
        return ret;
    }

    ret.origin = o;
    ret.hit = true;
    ret.position = intersect;
    ret.normal = intersect.normalize();
    return ret;
}

} // namespace PT