#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"
#include "constants.h"


void readPPM(const std::string& filename, std::vector<Vec3f>& framebuffer, unsigned long width, unsigned long height) {
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

    unsigned long int fileWidth, fileHeight, maxColor;
    file >> fileWidth >> fileHeight >> maxColor;

    if (fileWidth != width || fileHeight != height) {
        std::cerr << "Image dimensions do not match!" << std::endl;
        return;
    }

    file.ignore(); // Ignore the newline character

    char pixel[3];
    for (size_t i = 0; i < height; ++i) {
        for (size_t j = 0; j < width; ++j) {
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

struct Cylinder {
    Vec3f center;     // Center of the cylinder
    Vec3f axis;       // Direction of the cylinder's axis
    float radius;     // Radius of the cylinder
    float height;     // Height of the cylinder
    Material material;

    Cylinder(const Vec3f &c, const Vec3f &a, const float &r, const float &h, const Material &m)
            : center(c), axis(a), radius(r), height(h), material(m) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = orig - center;
        float a = dir.x * dir.x + dir.z * dir.z;
        float b = 2 * (L.x * dir.x + L.z * dir.z);
        float c = L.x * L.x + L.z * L.z - radius * radius;

        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return false;

        float t1 = (-b + sqrt(discriminant)) / (2 * a);
        float t2 = (-b - sqrt(discriminant)) / (2 * a);

        if (t1 > t2) std::swap(t1, t2);

        float y0 = orig.y + t1 * dir.y;
        if (y0 >= center.y - height / 2 && y0 <= center.y + height / 2) {
            t0 = t1;
            return true;
        }

        float y1 = orig.y + t2 * dir.y;
        if (y1 >= center.y - height / 2 && y1 <= center.y + height / 2) {
            t0 = t2;
            return true;
        }

        return false;
    }
};

struct Cone {

    float cosa;	    // half cone angle
    Vec3f c;		// tip position
    Vec3f v;		// axis
    Material m;	    // material


    Cone(const Vec3f &tip, const Vec3f &axis, float a, const Material &m) : cosa(a), c(tip), v(axis), m(m) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = orig - c;
        float cos2_a = cos(cosa) * cos(cosa);
        float a = dir.x * dir.x + dir.z * dir.z - cos2_a * dir.y * dir.y;
        float b = 2 * (L.x * dir.x + L.z * dir.z - cos2_a * L.y * dir.y);
        float c_val = L.x * L.x + L.z * L.z - cos2_a * L.y * L.y;

        float discriminant = b * b - 4 * a * c_val;
        if (discriminant < 0) return false;

        float t1 = (-b + sqrt(discriminant)) / (2 * a);
        float t2 = (-b - sqrt(discriminant)) / (2 * a);

        if (t1 > t2) std::swap(t1, t2);

        float hit_y = orig.y + t1 * dir.y;
        if (t1 > 0 && hit_y >= c.y && hit_y <= c.y + v.y) {
            t0 = t1;
            return true;
        }

        hit_y = orig.y + t2 * dir.y;
        if (t2 > 0 && hit_y >= c.y && hit_y <= c.y + v.y) {
            t0 = t2;
            return true;
        }

        return false;
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
    float min_cone_dist = std::numeric_limits<float>::max();
    bool hit_cone = false;

    for (const Cone &cone : cones) {
        float dist;
        if (cone.ray_intersect(orig, dir, dist) && dist < min_cone_dist) {
            std::cout << "Cone hit !" << std::endl;
            min_cone_dist = dist;
            hit = orig + dir * dist;
            N = (hit - cone.c).normalize(); // normal points from the hit point towards the center of the cone
            material = cone.m;
            hit_cone = true;
        }
    }

    return hit_cone;
}

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Cylinder> &cylinders, Vec3f &hit, Vec3f &N, Material &material) {
    float min_cylinder_dist = std::numeric_limits<float>::max();
    bool hit_cylinder = false;
    for (const Cylinder &cylinder : cylinders) {
        float dist;
        if (cylinder.ray_intersect(orig, dir, dist) && dist < min_cylinder_dist) {
            min_cylinder_dist = dist;
            hit = orig + dir * dist;

            // Compute normal vector for cylinder
            Vec3f L = hit - cylinder.center;
            float proj = L * cylinder.axis; // Projection of L onto axis
            Vec3f closest_point = cylinder.center +  cylinder.axis * proj;
            N = (hit - closest_point).normalize();

            material = cylinder.material;
            hit_cylinder = true;
        }
    }
    return hit_cylinder;
}



Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir,
               const std::vector<Sphere> &spheres,
               const std::vector<Cone> &cones,
               const std::vector<Cylinder> &cylinders,
               const std::vector<Light> &lights) {
    Vec3f point, N;
    Vec3f result = Vec3f(0.2, 0.7, 0.8);        // Defaults to background mask color.
    Material material;
    float diffuse_light_intensity = AMBIANT_LIGHT_LEVEL;

    if (scene_intersect(orig, dir, cylinders, point, N, material)
        || scene_intersect(orig, dir, spheres, point, N, material)
        || scene_intersect(orig, dir, cones, point, N, material)
        ) {
        for (size_t i = 0; i < lights.size(); i++) {
            Vec3f light_dir = (lights[i].position - point).normalize();
            diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_dir * N);
        }
        result = material.diffuse_color * diffuse_light_intensity;
    }

    return result;
}

void render(const unsigned long width, const unsigned long height, float fov) {
    // Output buffer.
    std::vector<Vec3f> framebuffer(width*height);

    // Threshold for the background mask :
    float threshold = MASK_THRESHOLD;
    // Some useful vectors.
    Vec3f up_direction        = Vec3f(0.0,  0.0, 0.0).normalize();
    Vec3f front_direction     = Vec3f(1.0,  0.0, 1.0).normalize();
    Vec3f right_direction     = Vec3f(1.0,  0.0, 0.0).normalize();
    Vec3f left_direction      = Vec3f(-1.0, 0.0, 0.0).normalize();

    // Vectors to store the shapes:
    std::vector<Sphere>     spheres;
    std::vector<Cone>       cones;
    std::vector<Cylinder>   cylinders;

    // Light sources
    std::vector<Light> lights;
    Light main_light = Light(Vec3f(MAIN_LIGHT_X, MAIN_LIGHT_Y, MAIN_LIGHT_Z), MAIN_LIGHT_INTENSITY);
    lights.push_back(main_light);

    // Camera position
    Vec3f camera = Vec3f(CAMERA_X,CAMERA_Y,CAMERA_Z);

    // Colors :
    Vec3f color_red    = Vec3f (1.0, 0.0, 0.0);
    Vec3f color_green  = Vec3f (0.0, 1.0, 0.0);
    Vec3f color_blue   = Vec3f (0.0, 0.0, 1.0);

    Vec3f color_orange = Vec3f (1.0, 0.3, 0.0);
    Vec3f color_brown  = Vec3f (210/255.f,105/255.f,30/255.f);

    Vec3f color_white  = Vec3f (1.0, 1.0, 1.0);
    Vec3f color_black  = Vec3f (0.15, 0.15, 0.15);
    Vec3f bg_color     = Vec3f (0.2, 0.7, 0.8);

    // Materials : (basically , the texture used, might be useful to add some noise...)
    Material carrot_material  = Material(color_orange);
    Material hat_material     = Material(color_black);
    Material button_material  = Material(color_black);
    Material arm_material     = Material(color_brown);
    Material snow_material    = Material(color_white);


    // Placing background in framebuffer.
    readPPM("snowy_house.ppm", framebuffer, width, height);


    // -- SHAPES -- //
    // Snowman Spheres :
    Sphere base_sphere  (Vec3f(0.0, 0.0, 0.0), BASE_SPHERE_RADIUS,  snow_material);
    Sphere torso_sphere (Vec3f(0.0, 1.1, 0.0), TORSO_SPHERE_RADIUS, snow_material);
    Sphere head_sphere  (Vec3f(0.0, 2.2, 0.0), HEAD_SPHERE_RADIUS,  snow_material);
    spheres.push_back(base_sphere);
    spheres.push_back(torso_sphere);
    spheres.push_back(head_sphere);


    // Cone representing the carrot
    Cone carrot(Vec3f(0.0, 2.5, 0.0), Vec3f(0.0, -.5, 0.0), M_PI / 4, carrot_material);
    cones.push_back(carrot);

    // Hat
    Cylinder hat_top_cylinder    (Vec3f(0.0, 2.9, 0.0), up_direction, 0.4, 0.5 , hat_material);
    Cylinder hat_bottom_cylinder (Vec3f(0.0, 2.7, 0.0), up_direction, 0.6, 0.05, hat_material);
    // Arms
    Cylinder right_arm_cylinder  (Vec3f( 1.0, 1.0, -0.1),right_direction, 0.1, 3.0 , arm_material);
    Cylinder left_arm_cylinder   (Vec3f(-1.0, 1.0, 0.1), left_direction,  0.1, 3.0 , arm_material);

    cylinders.push_back(hat_top_cylinder);
    cylinders.push_back(hat_bottom_cylinder);
    cylinders.push_back(right_arm_cylinder);
    cylinders.push_back(left_arm_cylinder);

    // Buttons
    Cylinder button_1    (Vec3f(0.0, 0.7 , 0.0), front_direction, 0.1, 0.1 , button_material);
    Cylinder button_2    (Vec3f(0.0, 1.05, 0.0), front_direction, 0.1, 0.1 , button_material);
    Cylinder button_3    (Vec3f(0.0, 1.40, 0.0), front_direction, 0.1, 0.1 , button_material);

    cylinders.push_back(button_1);
    cylinders.push_back(button_2);
    cylinders.push_back(button_3);

    // Eyes
    Cylinder left_eye    (Vec3f(-0.2, 2.3,  0.1), front_direction, 0.1, 0.1 , button_material);
    Cylinder right_eye   (Vec3f( 0.2, 2.3, -0.1), front_direction, 0.1, 0.1 , button_material);
    cylinders.push_back(left_eye);
    cylinders.push_back(right_eye);


    #pragma omp parallel for
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            float x    =  (2 * (i + 0.5)/(float)width  - 1) * tan(fov/2.) * (float)width/(float)height;
            float y    = -(2 * (j + 0.5)/(float)height - 1) * tan(fov/2.);

            // The direction in which we look.
            Vec3f dir  = Vec3f(x, y, -1).normalize();
            // The computed color given by the cast_ray method.
            // It's either a random color (depends on lightning and textures)
            // or the background mask color.
            Vec3f color = cast_ray(camera, dir, spheres, cones, cylinders, lights);

            // We ensure that we do not modify the background using the mask.
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

