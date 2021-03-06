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

#define SHADOWS_MAX_STEPS 10
#define SHADOWS_EPSILON 0.01f

#define EPSILON 0.1f
#define EPSILON_NORMAL 0.001f

// #define SHADOWS

// g++ raymarch_4d.cpp -o rotating.exe -std=c++17 -luser32 -lgdi32 -lopengl32 -lgdiplus -lShlwapi -lstdc++fs -O3

float rf(float min = 0.0, float max = 1.0)
{
    return min + static_cast<float>(rand()) / RAND_MAX * (max - min);
}

struct Hittable
{
    virtual float sdistance(const vf4d &p) = 0;
};

float RayMarch(vf4d origin, vf4d direction, Hittable *scene, float epsilon = EPSILON);

float GetShadows(vf4d origin, vf4d direction, Hittable *scene, float light_dist);

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

struct Hypercube : Hittable
{
    vf4d origin;
    float r;

    Hypercube(vf4d o, float r) : origin{o}, r{r}
    {
    }

    float sdistance(const vf4d &p_old) override
    {
        auto p = p_old - origin;
        p.x = abs(p.x);
        p.y = abs(p.y);
        p.z = abs(p.z);
        p.t = abs(p.t);
        p = p - vf4d{r, r, r, r};
        // auto res = ((p - center)).len() - radius;
        // std::cout<<p.x<<" "<<p.y<<" "<<p.z<<" "<<p.t<<" "<<res<<std::endl;
        auto x = abs(p.x);
        auto y = abs(p.y);
        auto z = abs(p.z);
        auto t = abs(p.t);
        auto ap = vf4d{x, y, z, t};

        // if (std::abs(x - y) < 0.1)
        // {
        //     // x-= 0.1;
        //     // y-= 0.1;
        //     // x = sqrt(x * x + y * y);
        //     // y = 0;
        //     // x += 0.1;
        //     y -= 0.1;
        // }
        // if (std::abs(x - z) < 0.1)
        // {
        //     // x = sqrt(x * x + z * z);
        //     // z = 0;
        //     // x += 0.1;
        //     z -= 0.1;
        // }
        // if (std::abs(x - t) < 0.1)
        // {
        //     // x = sqrt(x * x + t * t);
        //     // t = 0;
        //     // x += 0.1;
        //     t -= 0.1;
        // }
        // if (std::abs(y - z) < 0.1)
        // {
        //     // y = sqrt(y * y + z * z);
        //     // z = 0;
        //     // y += 0.1;
        //     z -= 0.1;
        // }
        // if (std::abs(y - t) < 0.1)
        // {
        //     // y = sqrt(y * y + t * t);
        //     // t = 0;
        //     // y -= 0.1;
        //     t -= 0.1;
        // }

        return (ap + p).len() / 2 + std::min(std::max(std::max(p.x, std::max(p.y, p.z)), p.t), 0.0f) - 0.1;
        // len(p) + min(0, max(x,y,z,t))
        // return std::max(std::max(x, y), std::max(z, t)) - radius;
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

struct Line : Hittable
{
    vf4d origin;
    float r;
    Line(const vf4d &origin, float r) : origin{origin}, r{r}
    {
    }
    float sdistance(const vf4d &p_old) override
    {
        auto p = p_old - origin;
        return std::sqrt(p.x * p.x + p.z * p.z + p.t * p.t) - r;
    }
};

struct Torus : Hittable
{
    vf4d origin;
    float r1;
    float r2;
    Torus(const vf4d &origin, float r1, float r2) : origin{origin}, r1{r1}, r2{r2}
    {
    }

    float sdistance(const vf4d &p_old) override
    {
        auto p = p_old - origin;
        // p.x = std::abs(p.x);
        // p.y = std::abs(p.y);
        // p.z = std::abs(p.z);
        // p.t = std::abs(p.t);

        // p = mf4d(1, 0, 0.1, 0,
        //          0, 1, 0.1, 0,
        //          0.1, 0.1, 1, 0.1,
        //          0, 0, 0.1, 1) *
        //     p;
        // d = min(d,
        // max(
        float r = sqrt(p.x * p.x + p.z * p.z) - r1;
        return std::abs(sqrt(r * r + p.y * p.y + p.t * p.t)) - r2;
        // return std::max(p.len() - r1d, r2-p.len());
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
        auto p = (p_old - origin) / 2;
        // if (std::abs(p.y) < 1.0)
        // {
        float dx = std::abs(p.x - floor(p.x + 0.5));
        float dz = std::abs(p.z - floor(p.z + 0.5));
        // return (sqrt(p.y * p.y + std::min(dx, dz) * std::min(dx, dz)) - 0.05) * 2;
        if (std::min(dx, dz) - 0.05 < 0.01)
        {
            return MAX_DISTANCE;
        }
        return std::abs(p.y) - 0.1;
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
            // d} -
            vf4d{d, d, d, d});
}

struct Scene : Hittable
{
    std::vector<Hittable *> objects;
    mf4d rot_matrix = IDENTITY_M;
    vf4d origin = vf4d(0, 0, 0, 0);
    // double (*reduce_fn)(float, float) = std::min;

    Scene() : objects{} {};

    void add_object(Hittable *o)
    {
        objects.push_back(o);
    }

    float sdistance(const vf4d &p) override
    {
        vf4d p_tr = transform_space(p);
        float res = MAX_DISTANCE;
        for (Hittable *object : objects)
        {
            res = std::min(object->sdistance(p_tr), res);
        }
        // std::cout<<p.x<<" "<<p.y<<" "<<p.z<<" "<<p.t<<" "<<res<<std::endl;
        return res;
    }

    inline vf4d transform_space(const vf4d &p)
    {
        return rot_matrix * (p - origin);
    };

    ~Scene()
    {
        for (Hittable *object : objects)
        {
            delete object;
        }
    }
};

float getColor(vf4d p, vf4d light_source, Hittable *scene)
{
    vf4d light_dir = (light_source - p).norm();

    vf4d normal_dir = getNormal(p, scene);
    // if (normal_dir.len() < 0.0001)
    // {
    //     return 0;
    // }
    normal_dir = normal_dir.norm();

    float projlen = std::clamp(static_cast<double>(normal_dir.dot(light_dir)), 0.0, 0.8) + 0.2;
// float projlen = 1;
#ifdef SHADOWS
    // if (RayMarch(p + normal_dir * EPSILON * 2.f, light_dir, scene) < EPSILON + (light_source - p).len())
    // {
    //     projlen *= 0.2;
    // }
    projlen *= GetShadows(p, light_dir, scene, (light_source - p).len());
#endif
    return projlen / (p.len() * 0.1 + 1.0);
}

float RayMarch(vf4d origin, vf4d direction, Hittable *scene, float epsilon)
{

    float dist = 0;
    vf4d p{origin};
    int i;
    for (i = 0; i < MAX_STEPS; i++)
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

float GetShadows(vf4d origin, vf4d direction, Hittable *scene, float light_dist)
{
    float res = 0.8;
    vf4d p{origin};
    int steps;
    float total_dist = 0;

    for (total_dist = 0.1, steps = 0;
         steps < SHADOWS_MAX_STEPS && total_dist < light_dist;
         steps++)
    {
        float d = scene->sdistance(origin + direction * total_dist);

        if (d < SHADOWS_EPSILON)
        {
            return 0.2;
        }
        res = std::min(res, 2 * d / total_dist);
        total_dist += d;
    }
    return res + 0.2;
    // return 1.0;
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
    // Scene _scene;
    // RotScene scene;

public:
    Renderer()
    {
        // Name you application
        sAppName = "Example";
        speed = 2;
        ang_speed = 1;
    }

    bool OnUserCreate() override
    {
        screen = tScreen{vf4d{0, 0, 1, 0}, 1.5, 1.0};
        origin = vf4d{0, 0, 0, 0};
        light_source = vf4d{0, 5, 0, 0};
        scene.add_object(new Sphere(vf4d{2, -1, 0, 0}, 2.0));
        // scene.add_object(new Line(vf4d{0, 0, 2, 0}, 2));

        scene.add_object(new Torus(vf4d{0, 0, -10, 0}, 3, 1));

        scene.add_object(new Hypercube(vf4d{0, 0, 5, 0}, 2));

        // scene.add_object
        // scene.add_object(new)
        scene.add_object(new Plane(vf4d{0, -5, 0, 0}));

        // for(int i=0; i<5; i++){
        //     scene.add_object(new Sphere(vf4d{rf(), rf(), rf(), 0}*5, rf(0, 3)));
        // }

        return true;
    }

    void CameraMovements(Scene &scene, float fElapsedTime)
    {
        vf4d delta = vf4d{0, 0, 0, 0};
        if (olc::PixelGameEngine::GetKey(olc::Key::W).bHeld)
        {
            scene.origin -= vf4d(0, 0, 1, 0) * speed * fElapsedTime;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::S).bHeld)
        {
            scene.origin -= vf4d(0, 0, -1, 0) * speed * fElapsedTime;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::D).bHeld)
        {
            scene.origin -= vf4d(1, 0, 0, 0) * speed * fElapsedTime;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::A).bHeld)
        {
            scene.origin -= vf4d(-1, 0, 0, 0) * speed * fElapsedTime;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::SPACE).bHeld)
        {
            scene.origin -= vf4d(0, 1, 0, 0) * speed * fElapsedTime;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::SHIFT).bHeld)
        {
            scene.origin -= vf4d(0, -1, 0, 0) * speed * fElapsedTime;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::Z).bHeld)
        {
            scene.origin -= vf4d(0, 0, 0, 1) * speed * fElapsedTime;
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::C).bHeld)
        {
            scene.origin -= vf4d(0, 0, 0, -1) * speed * fElapsedTime;
        }
    }

    void Camera2Rotations(Scene &scene, float fElapsedTime)
    {
        float c = cos(fElapsedTime * ang_speed);
        float s = sin(fElapsedTime * ang_speed);
        mf4d rot = IDENTITY_M;
        // ------------------------XZ
        if (olc::PixelGameEngine::GetKey(olc::Key::Q).bHeld)
        {
            rot = rot * mf4d(c, 0, -s, 0,
                             0, 1, 0, 0,
                             s, 0, c, 0,
                             0, 0, 0, 1);
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::E).bHeld)
        {
            rot = rot * mf4d(c, 0, s, 0,
                             0, 1, 0, 0,
                             -s, 0, c, 0,
                             0, 0, 0, 1);
        }
        // ----------------------------YZ
        if (olc::PixelGameEngine::GetKey(olc::Key::F).bHeld)
        {
            rot = rot * mf4d(1, 0, 0, 0,
                             0, c, s, 0,
                             0, -s, c, 0,
                             0, 0, 0, 1);
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::R).bHeld)
        {
            rot = rot * mf4d(1, 0, 0, 0,
                             0, c, -s, 0,
                             0, s, c, 0,
                             0, 0, 0, 1);
        }
        // ----------------------------XY
        if (olc::PixelGameEngine::GetKey(olc::Key::K1).bHeld)
        {
            rot = rot * mf4d(c, -s, 0, 0,
                             s, c, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1);
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::K3).bHeld)
        {
            rot = rot * mf4d(c, s, 0, 0,
                             -s, c, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1);
        }
        // ---------------------------ZT
        if (olc::PixelGameEngine::GetKey(olc::Key::K).bHeld)
        {
            rot = rot * mf4d(1, 0, 0, 0,
                             0, 1, 0, 0,
                             0, 0, c, s,
                             0, 0, -s, c);
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::L).bHeld)
        {
            rot = rot * mf4d(1, 0, 0, 0,
                             0, 1, 0, 0,
                             0, 0, c, -s,
                             0, 0, s, c);
        }
        if (olc::PixelGameEngine::GetKey(olc::Key::ESCAPE).bHeld)
        {
            rot = scene.rot_matrix.transposed();
        }

        scene.rot_matrix = scene.rot_matrix * rot;

        scene.origin = rot.transposed() * scene.origin;
    }

    // Draw Minimap:
    // map_xp, map_yp: coordinates of the top-left corner on the screen n pixels
    // map_wp, map_hp: coordinates of map width and height in pixels
    // vx, vy: vectors to form map`s x and y axis
    // disp: displacement of map from camera - not used yet(and may never be)
    void DrawMinimap(int map_xp, int map_yp, int map_wp, int map_hp, const vf4d &vx, const vf4d &vy, const vf4d &disp)
    {
        // auto right_screen_h = ScreenHeight() / 2;
        // auto right_screen_w = ScreenWidth() / 4;
        // auto right_screen_x = left_screen_x + left_screen_w;
        // auto right_screen_y = left_screen_y;

        float map_x = 40;
        float map_z = 40;
        for (int p_x = 0; p_x < map_wp; p_x++)
        {
            for (int p_y = 0; p_y < map_hp; p_y++)
            {
                float x = map_x * (p_x / static_cast<float>(map_wp) - 0.5);
                float y = -map_z * (p_y / static_cast<float>(map_hp) - 0.5);

                float col = 0;
                float col_red = 0;

                vf4d p = vx * x + vy * y;
                vf4d p_tr = scene.rot_matrix * p;

                // Objects
                if (scene.sdistance(p) < 0.0)
                {

                    col = 0.6;
                }
                // Coordinate lines:

                if ((std::abs(p_tr.dot(vx)) < 0.1) || (std::abs(p_tr.dot(vy)) < 0.1)) // Related to scene
                {
                    col += 0.4;
                }

                if (std::abs(x) < 0.1 || std::abs(y) < 0.1) // Related to camera
                {
                    col_red += 0.5;
                }
                // Grid on lines
                else if (((std::abs(x / 2 - floor(x / 2)) < 0.2) && std::abs(y) < 1.0) ||
                         (std::abs(y / 2 - floor(y / 2)) < 0.2) && std::abs(x) < 1.0)
                {
                    col_red += 0.5;
                }
                // View
                if ((y < 0) || ((x / y - (screen.center.x - screen.width / 2) / screen.center.z < 0.0) || (x / y - (screen.center.x + screen.width / 2) / screen.center.z > 0.0)))
                {
                    col *= 0.5;
                }

                vf4d proj = (p - origin);
                proj = proj / std::abs(proj.z) * screen.center.z; // displasement on screen
                // ?????????????????? Не пам'ятаю нащо
                // if (abs(proj.x - screen.center.x) > screen.width / 2 || abs(proj.y - screen.center.y) > screen.height / 2 || proj.z < 0)
                // {
                //     col *= 0.2;
                // }

                // Borders
                if (p_x * (map_wp - p_x - 1) * p_y * (map_hp - p_y - 1) == 0)
                {
                    col = 1;
                }
                col = std::min(col, 1.0f);
                col_red = std::min(col_red, 1.0f);
                Draw(p_x + map_xp, p_y + map_yp, olc::PixelF(col_red, col, 0));
            }
        }
        auto center_x = map_wp / 2;
        auto center_y = map_hp / 2;
        // DrawLine(center_x, center_y, )
        DrawTriangle(center_x + map_xp, center_y + 5 + map_yp,
                     center_x - 3 + map_xp, center_y - 2 + map_yp,
                     center_x + 3 + map_xp, center_y - 2 + map_yp,
                     olc::WHITE);
    };

    // Draw Camera view:
    // view_xp, view_yp: coordinates of the top-left corner on the screen n pixels
    // view_wp, view_hp: coordinates of view width and height in pixels
    // direction: direction the camera is facing
    void DrawView(int view_xp, int view_yp, int view_wp, int view_hp, const vf4d &direction)
    {
        // vf4d scene_light_source = scene.rot_matrix.transposed() * light_source + scene.origin;
        vf4d scene_light_source = light_source;

        for (int p_x = 0; p_x < view_wp; p_x++)
        {
            for (int p_y = 0; p_y < view_hp; p_y++)
            {
                vf4d ray_direction = direction + vf4d{screen.width * (static_cast<float>(p_x) / view_wp - 0.5f),
                                                      -screen.height * (static_cast<float>(p_y) / view_hp - 0.5f), 0, 0};
                auto dist = RayMarch(origin, ray_direction, &scene);
                float col_green = 0;
                float col_red = 0;
                float col_blue = 0;
                if (dist < MAX_DISTANCE)
                {
                    col_green = getColor(origin + ray_direction * dist, scene_light_source, &scene);
                }
                else
                {
                    // Draw Background
                    float fbg_col = ray_direction.norm().dot((scene_light_source - origin).norm());
                    float ibg_col = fbg_col * fbg_col * fbg_col * fbg_col * 0.8;
                    // float ibg_col = fbg_col * fbg_col;
                    col_green = ibg_col;
                    col_red = ibg_col;
                    col_blue = ibg_col;
                }
                // X and Y lines
                if (std::abs(ray_direction.y) < 0.001 || std::abs(ray_direction.x) < 0.001)
                {
                    col_red += 0.4;
                }
                Draw(p_x + view_xp, p_y + view_yp, olc::PixelF(col_red, col_green, col_blue));
            }
        }
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        // char title[20];
        // sprintf(title, "FPS: %i", 10);
        // olc::platform->SetWindowTitle(std::string(title));

        CameraMovements(scene, fElapsedTime);
        Camera2Rotations(scene, fElapsedTime);

        // -----------------------------3D-----------------------------------
        vf4d direction = screen.center - origin;

        int map_hp = ScreenHeight() / 2;
        int map_wp = ScreenWidth() / 4;
        auto view_hp = ScreenHeight();
        auto view_wp = ScreenWidth() / 2;

        DrawMinimap(0, 0, map_wp, map_hp, vf4d(1, 0, 0, 0), vf4d{0, 0, 1, 0}, vf4d{0, 0, 0, 0});      // XZ
        DrawMinimap(0, map_hp, map_wp, map_hp, vf4d(0, 1, 0, 0), vf4d{0, 0, 1, 0}, vf4d{0, 0, 0, 0}); // YZ

        DrawView(map_wp, 0, view_wp, view_hp, direction); //3D

        DrawMinimap(map_wp + view_wp, 0, map_wp, map_hp, vf4d(0, 0, 0, 1), vf4d{0, 0, 1, 0}, vf4d{0, 0, 0, 0});      // TZ
        DrawMinimap(map_wp + view_wp, map_hp, map_wp, map_hp, vf4d(1, 0, 0, 0), vf4d{0, 1, 0, 0}, vf4d{0, 0, 0, 0}); //XY

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
