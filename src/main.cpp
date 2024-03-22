#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"
#include "constants.h"


void readPPM(const std::string& filename, std::vector<Vec3f>& framebuffer, int width, int height) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    std::string header;
    std::getline(file, header); // Read the header
    if (header != "P6") {
        std::cerr << "Invalid PPM file format!" << std::endl;
        return;
    }

    // Skip comment lines
    while (file.peek() == '#') {
        std::string comment;
        std::getline(file, comment);
    }

    int fileWidth, fileHeight, maxColor;
    file >> fileWidth >> fileHeight >> maxColor;

    if (fileWidth != width || fileHeight != height) {
        std::cerr << "Image dimensions do not match!" << std::endl;
        return;
    }

    file.ignore(); // Ignore the newline character

    char pixel[3];
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            file.read(pixel, 3); // Read RGB values
            framebuffer[i * width + j].x = static_cast<float>(static_cast<unsigned char>(pixel[0])) / 255.0f;
            framebuffer[i * width + j].y = static_cast<float>(static_cast<unsigned char>(pixel[1])) / 255.0f;
            framebuffer[i * width + j].z = static_cast<float>(static_cast<unsigned char>(pixel[2])) / 255.0f;
        }
    }

    file.close();
}


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

struct Cone {
    Vec3f center;
    Vec3f direction; // Unit vector representing the direction of the cone
    float angle;     // Angle of the cone in radians
    Material material;

    Cone(const Vec3f &c, const Vec3f &dir, float a, const Material &m) : center(c), direction(dir), angle(a), material(m) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;
        float cosAngle = cos(angle);
        float sinAngle = sin(angle);

        // Compute projections
        float tca = L * dir;
        float d2 = L * L - tca * tca;

        // Check if the ray intersects the infinite cone
        if (d2 > (1 - cosAngle * cosAngle) * (tca * tca))
            return false;

        // Check if the intersection is within the height of the cone
        float thc = sqrtf(1 - cosAngle * cosAngle) * tca;
        float distance = tca - thc;

        Vec3f pointOnCone = orig + dir * distance;
        Vec3f vecToPointOnCone = pointOnCone - center;
        float height = vecToPointOnCone * direction;

        if (height < 0 || height > sinAngle)
            return false;

        t0 = distance;
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

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Cone> &cones, Vec3f &hit, Vec3f &N, Material &material) {
    float min_cones_dist = std::numeric_limits<float>::max();
    bool hit_cone = false;
    for (const Cone &cone : cones) {
        float dist;
        if (cone.ray_intersect(orig, dir, dist) && dist < min_cones_dist) {
            min_cones_dist = dist;
            hit = orig + dir * dist;
            Vec3f pointOnCone = hit - cone.center;
            float height = pointOnCone * cone.direction;
            Vec3f surfaceNormal = cone.direction * height - pointOnCone;
            N = surfaceNormal.normalize();
            material = cone.material;
            hit_cone = true;
        }
    }
    return hit_cone;
}


Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Cone> &cones, const std::vector<Light> &lights) {
    Vec3f point, N;
    Vec3f result = Vec3f(0.2, 0.7, 0.8);        // Defaults to background mask color.
    Material material;
    float diffuse_light_intensity;

    if (scene_intersect(orig, dir, spheres, point, N, material)) {
        diffuse_light_intensity = AMBIANT_LIGHT_LEVEL;
        for (size_t i = 0; i < lights.size(); i++) {
            Vec3f light_dir = (lights[i].position - point).normalize();
            diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_dir * N);
        }
        result = material.diffuse_color * diffuse_light_intensity;
    } else if (scene_intersect(orig, dir, cones, point, N, material)) {
        diffuse_light_intensity = AMBIANT_LIGHT_LEVEL;
        for (size_t i=0; i<lights.size(); i++) {
            Vec3f light_dir           = (lights[i].position - point).normalize();
            diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);
        }
        result = material.diffuse_color * diffuse_light_intensity;
    }

    return result;
}

void render(const unsigned long width, const unsigned long height, float fov) {
    // Output buffer.
    std::vector<Vec3f> framebuffer(width*height);
    // Threshold for the mask :
    float threshold = 0.1f;

    // Placing background in framebuffer.
    readPPM("snowy_house.ppm", framebuffer, width, height);

    // Colors :
    Vec3f color_red    = Vec3f (1.0, 0.0, 0.0);
    Vec3f color_green  = Vec3f (0.0, 1.0, 0.0);
    Vec3f color_blue   = Vec3f (0.0, 0.0, 1.0);
    Vec3f color_white  = Vec3f (1.0, 1.0, 1.0);
    Vec3f bg_color     = Vec3f(0.2, 0.7, 0.8);

    // SHAPES :
    // Snowman Spheres :
    Sphere base_sphere  (Vec3f(0.0, 0.0, 0.0), BASE_SPHERE_RADIUS,  Material(color_white));
    Sphere torso_sphere (Vec3f(0.0, 1.1, 0.0), TORSO_SPHERE_RADIUS, Material(color_white));
    Sphere head_sphere  (Vec3f(0.0, 2.2, 0.0), HEAD_SPHERE_RADIUS,  Material(color_white));
    std::vector<Sphere> spheres;
    spheres.push_back(base_sphere);
    spheres.push_back(torso_sphere);
    spheres.push_back(head_sphere);

    // Carrot
    Cone hat_cone(Vec3f(0.0, 3.5, 0.0), Vec3f(0.0, 1.0, 0.0), M_PI / 4, Material(color_blue));
    std::vector<Cone> cones;
    cones.push_back(hat_cone);

    // Hat


    // Arms


    // LIGHTNING
    std::vector<Light> lights;
    lights.push_back(Light(Vec3f(-5, 5,  5), 1.0));

    // Camera position
    Vec3f camera = Vec3f(CAMERA_X,CAMERA_Y,CAMERA_Z);

    #pragma omp parallel for
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            float x    =  (2 * (i + 0.5)/(float)width  - 1) * tan(fov/2.) * (float)width/(float)height;
            float y    = -(2 * (j + 0.5)/(float)height - 1) * tan(fov/2.);
            Vec3f dir  = Vec3f(x, y, -1).normalize();
            Vec3f color = cast_ray(camera, dir, spheres, cones, lights);

            if (!(color.x >= bg_color.x - threshold && color.x <= bg_color.x + threshold &&
                color.y >= bg_color.y - threshold && color.y <= bg_color.y + threshold &&
                color.z >= bg_color.z - threshold && color.z <= bg_color.z + threshold
                )) {
                framebuffer[i+j*width] = color;
            }
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

