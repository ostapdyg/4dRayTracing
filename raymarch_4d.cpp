#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include <algorithm>
#include <vector>
#include <iostream>
#include <exception>
#include "v4d.h"
#include "raymarch_4d.h"

#define MAX_DISTANCE 100
#define MAX_STEPS 100
#define EPSILON 0.1f
#define EPSILON_NORMAL 0.001f


struct Hittable {
    virtual float sdistance(const vf4d& p) = 0;
};

float RayMarch(vf4d origin, vf4d direction, Hittable* scene, float epsilon=EPSILON);

struct Sphere:Hittable{
    vf4d center;
    float radius;

    Sphere(vf4d o, float r):center{o}, radius{r}{
        
    }

    float sdistance(const vf4d& p) override{
        return std::min((p-center).len() - radius, p.y + 5);
    }
};

struct Segment:Hittable{
    vf4d origin;
    vf4d direction;
    float radius;

    Segment(const vf4d& origin, const vf4d& direction, float radius):origin{origin}, direction{direction}, radius{radius}{}

    float sdistance(const vf4d& p) override{
        auto plen = direction.dot(p - origin)/direction.len2(); // (op . direction) / (direction . direction); len(op projected on direction)/ len(direction)
        plen = std::clamp(static_cast<double>(plen), 0.0, 1.0);
        
        return (origin + direction*plen - p).len() - radius;
    }
};

// struct Box:Hittable{
    

// };

vf4d getNormalSlow(vf4d p, Hittable* scene){
    return vf4d{
            scene->sdistance(vf4d(p.x+EPSILON_NORMAL, p.y, p.z, p.t)) - scene->sdistance(vf4d(p.x-EPSILON_NORMAL, p.y, p.z, p.t)),
            scene->sdistance(vf4d(p.x, p.y+EPSILON_NORMAL, p.z, p.t)) - scene->sdistance(vf4d(p.x, p.y-EPSILON_NORMAL, p.z, p.t)),
            scene->sdistance(vf4d(p.x, p.y, p.z+EPSILON_NORMAL, p.t)) - scene->sdistance(vf4d(p.x, p.y, p.z-EPSILON_NORMAL, p.t)),
            scene->sdistance(vf4d(p.x, p.y, p.z, p.t+EPSILON_NORMAL)) - scene->sdistance(vf4d(p.x, p.y, p.z, p.t-EPSILON_NORMAL))
                            }.norm();
}

vf4d getNormal(vf4d p, Hittable* scene){
    float d = scene->sdistance(p);
    return (vf4d{
            scene->sdistance(vf4d(p.x+EPSILON_NORMAL, p.y, p.z, p.t)),
            scene->sdistance(vf4d(p.x, p.y+EPSILON_NORMAL, p.z, p.t)),
            scene->sdistance(vf4d(p.x, p.y, p.z+EPSILON_NORMAL, p.t)),
            scene->sdistance(vf4d(p.x, p.y, p.z, p.t+EPSILON_NORMAL))
                            } - vf4d{d, d, d, d}).norm();
}
olc::Pixel getColor(vf4d p, vf4d light_source, Hittable* scene){
    vf4d light_dir = (light_source - p).norm();

    vf4d normal_dir = getNormal(p, scene);
    float projlen = std::clamp(static_cast<double>(normal_dir.dot(light_dir)), 0.0, 1.0);
    
    if(RayMarch(p + light_dir*EPSILON*2.f, light_dir, scene) < EPSILON + (light_source - p).len()){projlen *= 0.1;}
    return olc::Pixel(0, 255*projlen, 0);
}


float RayMarch(vf4d origin, vf4d direction, Hittable* scene, float epsilon) {

    float dist = 0;
    vf4d p{origin};

    for(int i = 0; i < MAX_STEPS; i++)
    {
        float min_dist = scene->sdistance(p);

        dist += min_dist;
        p += direction*min_dist;

        if(min_dist < epsilon || dist >= MAX_DISTANCE){
            return dist;
        }
    }
    return dist;
};



struct tScreen
{
    vf4d top_left;
    float width;
    float height;
};

class Example : public olc::PixelGameEngine
{
private:
    tScreen screen;
    double speed;
    float x0 = 0;
    float y0 = 0;
    float z0 = 6;
    float t0 = 0;
    float time = 0;
    Hittable* scene;

public:
    Example()
    {
        // Name you application
        sAppName = "Example";
        speed = 1;
    }

    bool OnUserCreate() override
    {
        screen = tScreen{vf4d{-1.5, 1, 1, 0}, 3.0, 2.0};


        return true;
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        // t -= 10*speed*fElapsedTime / 100;
        float r = 2;
        time += fElapsedTime;
        float x = x0 + r*cos(time*speed);
        float z = z0 + r*sin(time*speed);
        float y = y0;
        float t = t0;
        vf4d origin{0, 0, 0, 0};
        vf4d source{0, 10, 6, 0 };
        auto seg = Segment{vf4d{x, y, z, t}, vf4d{1, 0, 0, 0}, 0.1};
        auto sphere = Sphere{vf4d{x, y, z, t}, 1};
        scene = &sphere;

        auto sphere_dist = scene->sdistance(origin);
        
        for (int p_x = 0; p_x < ScreenWidth(); p_x++)
        {
            for (int p_y = 0; p_y < ScreenHeight(); p_y++)
            {
                vf4d direction = screen.top_left + vf4d{screen.width*p_x/ScreenWidth(), -screen.height*p_y/ScreenHeight(), 0, 0};
                float fbg_col = direction.norm().dot((source-origin).norm());
                int ibg_col = 255*fbg_col*fbg_col*fbg_col*fbg_col;
                Draw(p_x, p_y, olc::Pixel(ibg_col, ibg_col, ibg_col));
                auto dist = RayMarch(origin, direction, scene);
                if(dist< MAX_DISTANCE ){
                    // olc::Pixel color = olc::Pixel(0, 255*(0.5f - (dist-sphere_dist)/MAX_DISTANCE*5), 0);
                    Draw(p_x, p_y, getColor(origin + direction*dist, source, scene));
                }
                
            }
        }
        return true;
    };


};

int main()
{
    vf4d a{1,0,0,0};
    // a = v4d_generic<vf4d>{
    //                                     vf4d{1, 0, 0, 0},
    //                                     vf4d{0, 1, 0, 0},
    //                                     vf4d{0, 0, 1, 0},
    //                                     vf4d{0, 0, 0, 1}  } * a;

    Example demo;
    if (demo.Construct(400, 300, 1, 1, 0, 0)){
        std::cout<<"demo constructed"<<std::endl;
        demo.Start();
    }
    return 0;
};
