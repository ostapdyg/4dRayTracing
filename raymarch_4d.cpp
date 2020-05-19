#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include <algorithm>
#include <vector>
#include <iostream>
#include <exception>
#include <cmath>
#include "v4d.h"
#include "raymarch_4d.h"

#define MAX_DISTANCE 100
#define MAX_STEPS 100
#define EPSILON 0.1f
#define EPSILON_NORMAL 0.001f
// g++ raymarch_4d.cpp -o rotating.exe -std=c++17 -luser32 -lgdi32 -lopengl32 -lgdiplus -lShlwapi -lstdc++fs -O3


struct Hittable
{
    virtual float sdistance(const vf4d &p) = 0;
};

float RayMarch(vf4d origin, vf4d direction, Hittable *scene, float epsilon = EPSILON);

struct Sphere : Hittable
{
    vf4d center;
    float radius;

    Sphere(vf4d o, float r) : center{o}, radius{r}
    {
    }

    float sdistance(const vf4d &p) override
    {
        auto res = ((p - center)).len() - radius;
        // std::cout<<p.x<<" "<<p.y<<" "<<p.z<<" "<<p.t<<" "<<res<<std::endl;
        return res;
    }
};

struct Segment : Hittable
{
    vf4d origin;
    vf4d direction;
    float radius;

    Segment(const vf4d &origin, const vf4d &direction, float radius) : origin{origin}, direction{direction}, radius{radius} {}

    float sdistance(const vf4d &p) override
    {
        auto plen = direction.dot(p - origin) / direction.len2(); // (op . direction) / (direction . direction); len(op projected on direction)/ len(direction)
        plen = std::clamp(static_cast<double>(plen), 0.0, 1.0);

        return (origin + direction * plen - p).len() - radius;
    }
};

struct Plane : Hittable
{
    vf4d origin;

    Plane(vf4d origin) : origin{origin} {}

    float sdistance(const vf4d &p) override
    {
        return (p - origin).y;
    }
};

struct Grid : Hittable
{
    vf4d origin;
    Grid(vf4d origin) : origin{origin} {}

    float sdistance(const vf4d &p_old) override
    {
        auto p = (p_old - origin) / 10;
        // if (std::abs(p.y) < 1.0)
        // {
        float dx = std::abs(p.x - floor(p.x + 0.5));
        float dz = std::abs(p.z - floor(p.z + 0.5));
        return (sqrt(p.y * p.y + std::min(dx, dz) * std::min(dx, dz)) - 0.005) * 10;
        // }
        // return MAX_DISTANCE;
    }
};

vf4d getNormalSlow(vf4d p, Hittable *scene)
{
    return vf4d{
        scene->sdistance(vf4d(p.x + EPSILON_NORMAL, p.y, p.z, p.t)) - scene->sdistance(vf4d(p.x - EPSILON_NORMAL, p.y, p.z, p.t)),
        scene->sdistance(vf4d(p.x, p.y + EPSILON_NORMAL, p.z, p.t)) - scene->sdistance(vf4d(p.x, p.y - EPSILON_NORMAL, p.z, p.t)),
        scene->sdistance(vf4d(p.x, p.y, p.z + EPSILON_NORMAL, p.t)) - scene->sdistance(vf4d(p.x, p.y, p.z - EPSILON_NORMAL, p.t)),
        scene->sdistance(vf4d(p.x, p.y, p.z, p.t + EPSILON_NORMAL)) - scene->sdistance(vf4d(p.x, p.y, p.z, p.t - EPSILON_NORMAL))}
        .norm();
}

vf4d getNormal(vf4d p, Hittable *scene)
{
    float d = scene->sdistance(p);
    return (vf4d{
                scene->sdistance(vf4d(p.x + EPSILON_NORMAL, p.y, p.z, p.t)),
                scene->sdistance(vf4d(p.x, p.y + EPSILON_NORMAL, p.z, p.t)),
                scene->sdistance(vf4d(p.x, p.y, p.z + EPSILON_NORMAL, p.t)),
                scene->sdistance(vf4d(p.x, p.y, p.z, p.t + EPSILON_NORMAL))} -
            vf4d{d, d, d, d})
        .norm();
}
float getColor(vf4d p, vf4d light_source, Hittable *scene)
{
    vf4d light_dir = (light_source - p).norm();

    vf4d normal_dir = getNormal(p, scene);
    float projlen = std::clamp(static_cast<double>(normal_dir.dot(light_dir)), 0.0, 1.0);

    if (RayMarch(p + normal_dir * EPSILON * 2.f, light_dir, scene) < EPSILON + (light_source - p).len())
    {
        projlen *= 0.1;
    }
    return projlen;
}

float RayMarch(vf4d origin, vf4d direction, Hittable *scene, float epsilon)
{

    float dist = 0;
    vf4d p{origin};

    for (int i = 0; i < MAX_STEPS; i++)
    {
        float min_dist = scene->sdistance(p);

        dist += min_dist;
        p += direction * min_dist;

        if (min_dist < epsilon || dist >= MAX_DISTANCE)
        {
            return dist;
        }
    }
    return dist;
};

struct Scene : Hittable
{
    std::vector<Hittable *> objects;
    mf4d rot_matrix = IDENTITY_M;
    vf4d origin = vf4d(0, 0, 0, 0);
    // double (*reduce_fn)(float, float) = std::min;

    Scene() : objects{} {

              };
    void add_object(Hittable *o)
    {
        objects.push_back(o);
    }

    float sdistance(const vf4d &p) override
    {
        vf4d p_tr = rot_matrix * p - origin;
        float res = MAX_DISTANCE;
        for (Hittable *object : objects)
        {
            res = std::min(object->sdistance(p_tr), res);
        }
        // std::cout<<p.x<<" "<<p.y<<" "<<p.z<<" "<<p.t<<" "<<res<<std::endl;
        return res;
    }

    ~Scene()
    {
        for (Hittable *object : objects)
        {
            delete object;
        }
    }
};

struct tScreen
{
    vf4d center;
    float width;
    float height;
};

class Renderer : public olc::PixelGameEngine
{
private:
    tScreen screen;
    vf4d origin;
    vf4d light_source;
    double speed;
    double ang_speed;

    mf4d camera_rot = IDENTITY_M;

    float x0 = 0;
    float y0 = 0;
    float z0 = 6;
    float t0 = 0;
    float time = 0;

    Scene scene;

public:
    Renderer()
    {
        // Name you application
        sAppName = "Example";
        speed = 2;
        ang_speed = 0.5;
    }

    bool OnUserCreate() override
    {
        screen = tScreen{vf4d{0, 0, 1, 0}, 1.5, 1.0};
        origin = vf4d{0, 0, 0, 0};
        light_source = vf4d{0, 10, 6, 0};
        scene.add_object(new Sphere(vf4d{0, -1, 0, 0}, 0.5));
        scene.add_object(new Sphere(vf4d{0, -1, 6, 0}, 2));
        scene.add_object(new Plane(vf4d{0, -5, 0, 0}));
        scene.origin = vf4d{0, 0, 0, 0};
        return true;
    }

    void CameraMovements(Scene& scene, vf4d direction, float fElapsedTime){
        if (olc::PixelGameEngine::GetKey(olc::Key::W).bHeld)
        {
            scene.origin -= direction * speed * fElapsedTime;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::S).bHeld)
        {
            scene.origin += direction * speed * fElapsedTime;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::D).bHeld)
        {
            scene.origin -= mf4d{0, 0, 1, 0,
                                 0, 1, 0, 0,
                                 -1, 0, 0, 0,
                                 0, 0, 0, 1} *
                            (direction * speed * fElapsedTime);
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::A).bHeld)
        {
            scene.origin += mf4d{0, 0, 1, 0,
                                 0, 1, 0, 0,
                                 -1, 0, 0, 0,
                                 0, 0, 0, 1} *
                            (direction * speed * fElapsedTime);
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::SPACE).bHeld)
        {
            scene.origin -= mf4d{1, 0, 0, 0,
                                 0, 0, 1, 0,
                                 0, -1, 0, 0,
                                 0, 0, 0, 1} *
                            (direction * speed * fElapsedTime);
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::SHIFT).bHeld)
        {
            scene.origin += mf4d{1, 0, 0, 0,
                                 0, 0, 1, 0,
                                 0, -1, 0, 0,
                                 0, 0, 0, 1} *
                            (direction * speed * fElapsedTime);
        }
    }

    void Camera2Rotations(Scene& scene, float fElapsedTime){
        if (olc::PixelGameEngine::GetKey(olc::Key::Q).bHeld)
        {
            auto rot_xz = mf4d(cos(fElapsedTime * ang_speed), 0, -sin(fElapsedTime * ang_speed), 0,
                               0,                             1,  0,                             0,
                               sin(fElapsedTime * ang_speed), 0,  cos(fElapsedTime * ang_speed), 0,
                               0,                             0,  0,                             1);
            camera_rot = camera_rot * rot_xz;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::E).bHeld)
        {
            auto rot_xz = mf4d(cos(fElapsedTime * ang_speed), 0, sin(fElapsedTime * ang_speed), 0,
                               0, 1, 0, 0,
                               -sin(fElapsedTime * ang_speed), 0, cos(fElapsedTime * ang_speed), 0,
                               0, 0, 0, 1);
            camera_rot = camera_rot * rot_xz;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::F).bHeld)
        {
            auto rot_xz = mf4d( 1, 0, 0, 0,
                                0, cos(fElapsedTime * ang_speed), sin(fElapsedTime * ang_speed), 0,
                                0, -sin(fElapsedTime * ang_speed), cos(fElapsedTime * ang_speed), 0,
                                0, 0, 0, 1);
            camera_rot = camera_rot * rot_xz;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::R).bHeld)
        {
            auto rot_xz = mf4d(1, 0, 0, 0,
                               0, cos(fElapsedTime * ang_speed), -sin(fElapsedTime * ang_speed), 0,
                               0, sin(fElapsedTime * ang_speed), cos(fElapsedTime * ang_speed), 0,
                               0, 0, 0, 1);
            camera_rot = camera_rot * rot_xz;
        }
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        // t -= 10*speed*fElapsedTime / 100;
        float r = 2;
        time = fElapsedTime;
        float x = x0 + r * cos(time * speed);
        float z = z0 + r * sin(time * speed);
        float y = y0;
        float t = t0;
        vf4d direction = screen.center - origin;

        CameraMovements(scene, camera_rot*direction, fElapsedTime);
        Camera2Rotations(scene, fElapsedTime);
        // scene.origin += vf4d{r*cos(time*speed), r*sin(time*speed), r*sin(time*speed), 0};
        // auto rot_xz = mf4d(cos(time * speed), 0, -sin(time * speed), 0,
        //                    0, 1, 0, 0,
        //                    sin(time * speed), 0, cos(time * speed), 0,
        //                    0, 0, 0, 1);
        // scene.rot_matrix = scene.rot_matrix * rot_xz;

        // auto rot_yx = mf4d(1, 0,                 0,                 0,
        //                    0, cos(time * speed), sin(time * speed), 0,
        //                    0,-sin(time * speed), cos(time * speed), 0,
        //                    0, 0,                 0,                 1);

        // -----------------------------3D-----------------------------------
        auto left_screen_h = ScreenHeight();
        auto left_screen_w = ScreenWidth() / 2;
        auto left_screen_x = 0;
        auto left_screen_y = 0;
        for (int p_x = 0; p_x < left_screen_w; p_x++)
        {
            for (int p_y = 0; p_y < left_screen_h; p_y++)
            {
                vf4d ray_direction = direction + vf4d{screen.width * (static_cast<float>(p_x) / left_screen_w - 0.5),
                                                      -screen.height * (static_cast<float>(p_y) / left_screen_h - 0.5), 0, 0};

                ray_direction = camera_rot * ray_direction;

                auto dist = RayMarch(origin, ray_direction, &scene);
                float col_green = 0;
                float col_red = 0;
                float col_blue = 0;
                if (dist < MAX_DISTANCE)
                {
                    col_green = getColor(origin + ray_direction * dist, light_source, &scene);

                }
                else
                {
                    // Draw Background
                    float fbg_col = ray_direction.norm().dot((light_source - origin).norm());
                    int ibg_col = fbg_col * fbg_col * fbg_col * fbg_col*0.8;
                    col_green = ibg_col;
                    col_red = ibg_col;
                    col_blue = ibg_col;
                }
                if (std::abs(ray_direction.y) < 0.001)
                {
                    col_red += 0.2;
                }
                Draw(p_x + left_screen_x, p_y + left_screen_y, olc::PixelF(col_red, col_green, col_blue));
            }
        }

        //------------ 2D --------------
        auto right_screen_h = ScreenHeight();
        auto right_screen_w = ScreenWidth() / 2;
        auto right_screen_x = ScreenWidth() - right_screen_w;
        auto right_screen_y = ScreenHeight() - right_screen_h;

        float map_x = 40;
        float map_z = 40;
        for (int p_x = 0; p_x < right_screen_w; p_x++)
        {
            for (int p_y = 0; p_y < right_screen_h; p_y++)
            {
                float x = map_x * (p_x / static_cast<float>(right_screen_w) - 0.5);
                float z = -map_z * (p_y / static_cast<float>(right_screen_h) - 0.5);

                vf4d p = vf4d(x, 0, z, 0);
                vf4d p_rot = camera_rot * (p - scene.origin);
                float col;
                if (scene.sdistance(camera_rot*p) < 0.0)
                {

                    col = 0.6;
                }
                else
                {
                    col = 0;
                }
                if ((std::abs(p_rot.x - floor(p_rot.x)) < 0.1) ||
                    (std::abs(p_rot.z - floor(p_rot.z)) < 0.1))
                {
                    col += 0.2;
                }
                if ((z < 0) || ((x / z - (screen.center.x - screen.width / 2) / screen.center.z < 0.0) || (x / z - (screen.center.x + screen.width / 2) / screen.center.z > 0.0)))
                {
                    col *= 0.2;
                }
                Draw(p_x + right_screen_x, p_y + right_screen_y, olc::PixelF(0, col, 0));
            }
        }
        auto center_x = right_screen_w / 2;
        auto center_y = right_screen_h / 2;
        // DrawLine(center_x, center_y, )
        DrawTriangle(center_x + right_screen_x, center_y + 5 + right_screen_y,
                     center_x - 3 + right_screen_x, center_y - 2 + right_screen_y,
                     center_x + 3 + right_screen_x, center_y - 2 + right_screen_y,
                     olc::WHITE);

        return true;
    };
};

int main()
{
    vf4d a{1, 0, 0, 0};
    // a = v4d_generic<vf4d>{
    //                                     vf4d{1, 0, 0, 0},
    //                                     vf4d{0, 1, 0, 0},
    //                                     vf4d{0, 0, 1, 0},
    //                                     vf4d{0, 0, 0, 1}  } * a;

    Renderer demo;
    if (demo.Construct(800, 300, 1, 1, 0, 0))
    {
        std::cout << "demo constructed" << std::endl;
        demo.Start();
    }
    return 0;
};
