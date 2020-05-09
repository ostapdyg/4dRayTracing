#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include <algorithm>
#include <vector>
#include <iostream>
#include <exception>
#include "v4d.h"

#define MAX_DISTANCE 10
#define MAX_STEPS 100
#define EPSILON 0.01f
#define EPSILON_NORMAL 0.01f

struct Hittable {
    virtual float sdistance(const v4d_generic<float>& p) = 0;
};


struct Sphere:Hittable{
    v4d_generic<float> center;
    float radius;

    Sphere(v4d_generic<float> o, float r):center{o}, radius{r}{
        
    }

    float sdistance(const v4d_generic<float>& p) override{
        return (p-center).len() - radius;
    }
};

struct Segment:Hittable{
    v4d_generic<float> origin;
    v4d_generic<float> direction;
    float radius;

    Segment(const v4d_generic<float>& origin, const v4d_generic<float>& direction, float radius):origin{origin}, direction{direction}, radius{radius}{}

    float sdistance(const v4d_generic<float>& p) override{
        auto plen = direction.dot(p - origin)/direction.len2(); // (op . direction) / (direction . direction); len(op projected on direction)/ len(direction)
        plen = std::clamp(static_cast<double>(plen), 0.0, 1.0);
        
        return (origin + direction*plen - p).len() - radius;
    }
};

v4d_generic<float> Normal(v4d_generic<float> p, Hittable* scene){
    return v4d_generic<float>{
            scene->sdistance(v4d_generic<float>(p.x+EPSILON_NORMAL, p.y, p.z, p.t)) - scene->sdistance(v4d_generic<float>(p.x-EPSILON_NORMAL, p.y, p.z, p.t)),
            scene->sdistance(v4d_generic<float>(p.x+EPSILON_NORMAL, p.y, p.z, p.t)) - scene->sdistance(v4d_generic<float>(p.x-EPSILON_NORMAL, p.y, p.z, p.t)),
            scene->sdistance(v4d_generic<float>(p.x+EPSILON_NORMAL, p.y, p.z, p.t)) - scene->sdistance(v4d_generic<float>(p.x-EPSILON_NORMAL, p.y, p.z, p.t)),
            scene->sdistance(v4d_generic<float>(p.x+EPSILON_NORMAL, p.y, p.z, p.t)) - scene->sdistance(v4d_generic<float>(p.x-EPSILON_NORMAL, p.y, p.z, p.t))
                            };


}

float RayMarch(v4d_generic<float> origin, v4d_generic<float> direction, Hittable* scene, float epsilon=EPSILON) {

    float dist = 0;
    v4d_generic<float> p{origin};

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
    v4d_generic<float> top_left;
    float width;
    float height;
};

class Example : public olc::PixelGameEngine
{
private:
    tScreen screen;
    double speed;
    float x = 0;
    float y = 0;
    float z = 6;
    float t = 0;
    Hittable* scene;

public:
    Example()
    {
        // Name you application
        sAppName = "Example";
        speed = 20;
    }

    bool OnUserCreate() override
    {
        screen = tScreen{v4d_generic<float>{-1.5, -1, 1, 0}, 3.0, 2.0};


        return true;
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        // t += speed*fElapsedTime / 100;
        z -= speed*fElapsedTime / 100;
        v4d_generic<float> origin{0, 0, 0, 0};

        auto seg = Segment{v4d_generic<float>{x, y, z, t}, v4d_generic<float>{1, 0, 0, 0}, 0.1};
        auto sphere = Sphere{v4d_generic<float>{x, y, z, t}, 1};
        scene = &sphere;

        auto sphere_dist = scene->sdistance(origin);
        
        for (int p_x = 0; p_x < ScreenWidth(); p_x++)
        {
            for (int p_y = 0; p_y < ScreenHeight(); p_y++)
            {
                v4d_generic<float> direction = screen.top_left + v4d_generic<float>{screen.width*p_x/ScreenWidth(), screen.height*p_y/ScreenHeight(), 0, 0};
                Draw(p_x, p_y, olc::BLACK);
                auto dist = RayMarch(origin, direction, scene);
                if(dist< MAX_DISTANCE ){
                    olc::Pixel color = olc::Pixel(0, 255*(0.5f - (dist-sphere_dist)/MAX_DISTANCE*5), 0);
                    Draw(p_x, p_y, color);
                }
                
            }
        }
        return true;
    };


};

int main()
{
    v4d_generic<float> a{1,0,0,0};
    a = v4d_generic<v4d_generic<float>>{
                                        v4d_generic<float>{1, 0, 0, 0},
                                        v4d_generic<float>{0, 1, 0, 0},
                                        v4d_generic<float>{0, 0, 1, 0},
                                        v4d_generic<float>{0, 0, 0, 1}  } * a;

    Example demo;
    if (demo.Construct(400, 300, 1, 1, 0, 0)){
        std::cout<<"demo constructed"<<std::endl;
        demo.Start();
    }
    return 0;
};
