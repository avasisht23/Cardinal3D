
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

namespace Samplers {

Vec2 Rect::Uniform::sample(float& pdf) const {

    // TODO (PathTracer): Task 1
    // Generate a uniformly random point on a rectangle of size (size.x by size.y)

    // Tip: consider using RNG::unit()  (see util/rand.h)

    // PDF is the probability density of the chosen sample
    // the PDF should integrate to 1 over the whole rectangle
    // if integral (0 -> area) = 1, then pdf is 1/area
    pdf = 1.0f / (size.x * size.y);

    // Return the randomly generated point
    float random_x = size.x * RNG::unit();
    float random_y = size.y * RNG::unit();

    return Vec2(random_x, random_y);
}

Vec3 Hemisphere::Cosine::sample(float& pdf) const {

    // TODO (PathTracer): Task 6
    // You may implement this, but don't have to.

    // grab random angles according to cos weighted: 
    // pdf = cos(theta) / 
    float rand_phi = RNG::unit() * 2 * PI_F;
    float rand_theta = std::acos(Radians(std::sqrt(RNG::unit())));

    float x = std::sin(rand_phi) * std::cos(rand_theta) ;
    float y = std::sin(rand_phi) * std::sin(rand_theta) ;
    float z = std::cos(rand_phi);

    pdf = std::cos(rand_theta) / PI_F;

    return Vec3(x, y, z);
}

Vec3 Sphere::Uniform::sample(float& pdf) const {

    // TODO (PathTracer): Task 7
    // Generate a uniformly random point on the unit sphere (or equivalently, direction)
    // Tip: start with Hemisphere::Uniform

    pdf = 1.0f; // what was the PDF at the chosen direction?
    return Vec3();
}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7
    // Set up importance sampling for a spherical environment map image.

    // You may make use of the pdf, cdf, and total members, or create your own
    // representation.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
}

Vec3 Sphere::Image::sample(float& out_pdf) const {

    // TODO (PathTracer): Task 7
    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound can easily binary search your CDF

    out_pdf = 1.0f; // what was the PDF (again, PMF here) of your chosen sample?
    return Vec3();
}

Vec3 Point::sample(float& pmf) const {

    pmf = 1.0f;
    return point;
}

Vec3 Two_Points::sample(float& pmf) const {
    if(RNG::unit() < prob) {
        pmf = prob;
        return p1;
    }
    pmf = 1.0f - prob;
    return p2;
}

Vec3 Hemisphere::Uniform::sample(float& pdf) const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    pdf = 1.0f / (2.0f * PI_F);
    return Vec3(xs, ys, zs);
}

} // namespace Samplers
