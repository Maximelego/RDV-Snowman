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
struct Light {
    Light(const Vec3f &p, const float &i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

struct Material {
    Material(const Vec3f &color) : diffuse_color(color) {}
    Material() : diffuse_color() {}
    Vec3f diffuse_color;
};

struct Sphere {
    Vec3f center;
    float radius;
    Material material;

    Sphere(const Vec3f &c, const float &r, const Material &m) : center(c), radius(r), material(m) {}

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

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material) {
    float spheres_dist = std::numeric_limits<float>::max();
    for (size_t i=0; i < spheres.size(); i++) {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }
    return spheres_dist<1000;
}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights) {
    Vec3f point, N;
    Material material;

    if (!scene_intersect(orig, dir, spheres, point, N, material)) {
        return Vec3f(0.2, 0.7, 0.8); // background color
    }

    float diffuse_light_intensity = 0;
    for (size_t i=0; i<lights.size(); i++) {
        Vec3f light_dir      = (lights[i].position - point).normalize();
        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);
    }
    return material.diffuse_color * diffuse_light_intensity;
}

void render(const unsigned long width, const unsigned long height, float fov) {
    // Output buffer.
    std::vector<Vec3f> framebuffer(width*height);

    // Colors :
    Vec3f color_red    = Vec3f (1.0, 0.0, 0.0);
    Vec3f color_green  = Vec3f (0.0, 1.0, 0.0);
    Vec3f color_blue   = Vec3f (0.0, 0.0, 1.0);
    Vec3f color_white  = Vec3f (1.0, 1.0, 1.0);

    // SHAPES :
    // Spheres :
    Sphere base_sphere  (Vec3f(0.0, 0.0, 0.0), BASE_SPHERE_RADIUS,  Material(color_white));
    Sphere torso_sphere (Vec3f(0.0, 1.1, 0.0), TORSO_SPHERE_RADIUS, Material(color_white));
    Sphere head_sphere  (Vec3f(0.0, 2.2, 0.0), HEAD_SPHERE_RADIUS,  Material(color_white));

    std::vector<Sphere> spheres;
    spheres.push_back(base_sphere);
    spheres.push_back(torso_sphere);
    spheres.push_back(head_sphere);

    // LIGHTNING
    std::vector<Light> lights;
    lights.push_back(Light(Vec3f(-5, 5,  5), 1.1));

    // Camera position
    Vec3f camera = Vec3f(CAMERA_X,CAMERA_Y,CAMERA_Z);


    #pragma omp parallel for
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            float x    =  (2 * (i + 0.5)/(float)width  - 1) * tan(fov/2.) * (float)width/(float)height;
            float y    = -(2 * (j + 0.5)/(float)height - 1) * tan(fov/2.);
            Vec3f dir  = Vec3f(x, y, -1).normalize();

            framebuffer[i+j*width] = cast_ray(camera, dir, spheres, lights);
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

