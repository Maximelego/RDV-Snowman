#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"
#include "constants.h"

// Definition of a sphere.
struct Sphere {
    Vec3f center;
    float radius;

    Sphere(const Vec3f &c, const float &r) : center(c), radius(r) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;
        float tca = L*dir;
        float d2 = L*L - tca*tca;
        if (d2 > radius*radius) return false;
        float thc = sqrtf(radius*radius - d2);
        t0       = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};


const float noise_amplitude = NOISE_LEVEL  ;  // amount of noise applied to the sphere (towards the center)


template <typename T> inline T lerp(const T &v0, const T &v1, float t) {
    return v0 + (v1-v0)*std::max(0.f, std::min(1.f, t));
}


Vec3f palette_snow(const float d) { // simple linear gradient yellow-orange-red-darkgray-gray. d is supposed to vary from 0 to 1
    const Vec3f lightgray(0.7, 0.7, 0.7);
    const Vec3f lightlightgray(0.8, 0.8, 0.8);
    const Vec3f lightlightlightgray(0.9, 0.9, 0.9);
    const Vec3f white(1.0, 1.0, 1.0);

    float x = std::max(0.f, std::min(1.f, d));

    if (x < .25f)
        return lerp(lightgray, white, x * 4.f);

    else if (x < .5f)
        return lerp(lightlightgray, white, x * 4.f - 1.f);

    else if (x < .75f)
        return lerp(lightlightlightgray, white, x * 4.f - 2.f);

    return lerp(white, white, x * 4.f - 3.f);
}


bool cast_ray_on_sphere(const Vec3f &orig, const Vec3f &dir, const Sphere &sphere) {
    float sphere_dist = std::numeric_limits<float>::max();
    return sphere.ray_intersect(orig, dir, sphere_dist);
}


void render(const unsigned long width, const unsigned long height, float fov) {
    // Output buffer.
    std::vector<Vec3f> framebuffer(width*height);

    // SHAPES :
    // Base Sphere :
    Sphere base_sphere(Vec3f(0.0, 0.0, 0.0), BASE_SPHERE_RADIUS);
    // Torso Sphere :
    Sphere torso_sphere(Vec3f(0.0, 0.8, 0.0), TORSO_SPHERE_RADIUS);
    // Head Sphere :
    Sphere head_sphere(Vec3f(0.0, 1.4, 0.0), HEAD_SPHERE_RADIUS);


    // Camera position
    Vec3f camera = Vec3f(CAMERA_X,CAMERA_Y,CAMERA_Z);

    #pragma omp parallel for
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            float x    =  (2 * (i + 0.5)/(float)width  - 1) * tan(fov/2.) * (float)width/(float)height;
            float y    = -(2 * (j + 0.5)/(float)height - 1) * tan(fov/2.);
            Vec3f dir  = Vec3f(x, y, -1).normalize();

            Vec3f color_red   = Vec3f (1.0, 0.0, 0.0);
            Vec3f color_green = Vec3f (0.0, 1.0, 0.0);
            Vec3f color_blue  = Vec3f (0.0, 0.0, 1.0);
            Vec3f bg_color    = Vec3f (0.0, 0.0, 0.0);
            Vec3f res_color   = bg_color;

            if(cast_ray_on_sphere(camera, dir, base_sphere)) {
                res_color = color_red;
            }
            if (cast_ray_on_sphere(camera, dir, torso_sphere)) {
                res_color = color_green;
            }
            if (cast_ray_on_sphere(camera, dir, head_sphere)) {
                res_color = color_blue;
            }
            framebuffer[i+j*width] = res_color;
        }
    }

    std::ofstream ofs; // save the framebuffer to file
    ofs.open("./Snowman.ppm");
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height*width; ++i) {
        for (size_t j = 0; j<3; j++) {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();
}



int main() {

    std::cout << "[INFO] - Building Snowman..." << std::endl;

    render(IMAGE_WIDTH, IMAGE_HEIGHT, M_PI/3.);

    std::cout << "[INFO] - Snowman done ! It was outputted in a Snowman.ppm file." << std::endl;

    return 0;
}

