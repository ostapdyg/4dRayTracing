#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include <algorithm>
#include <vector>
#include <iostream>
#include <exception>

// template<class T, int N>
// struct vnd_generic
// {
//     T c[N];
//     inline vnd_generic()
//     {
//         for(int i==0; i<N; i++){
//             c[i] = 0;
//         }
//     }
//     inline vnd_generic()
//     {
//         for(int i==0; i<N; i++){
//             c[i] = 0;
//         }
//     }
// };

template <class T>
struct v3d_generic
{
    T x = 0;
    T y = 0;
    T z = 0;
    inline v3d_generic() : x(0), y(0), z(0) {}
    inline v3d_generic(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
    inline v3d_generic(const v3d_generic &v) : x(v.x), y(v.y), z(v.z) {}
    inline T len() const { return std::sqrt(x * x + y * y + z * z); }
    inline T len2() const { return x * x + y * y + z * z; }
    inline v3d_generic norm() const
    {
        T r = 1 / len();
        return v3d_generic(x * r, y * r, z * r);
    }
    inline v3d_generic perp(const v3d_generic &rhs)       const { return v2d_generic(-y, x); }
    inline T           dot(const v3d_generic &rhs)        const { return this->x * rhs.x + this->y * rhs.y + this->z * rhs.z; }
    inline v3d_generic cross(const v3d_generic &rhs)      const { return v3d_generic(this->y * rhs.z - this->z * rhs.y, this->z * rhs.x - this->x * rhs.z, this->x * rhs.y - this->y * rhs.x); }
    inline v3d_generic operator+(const v3d_generic &rhs)  const { return v3d_generic(this->x + rhs.x, this->y + rhs.y, this->z + rhs.z); }
    inline v3d_generic operator-(const v3d_generic &rhs)  const { return v3d_generic(this->x - rhs.x, this->y - rhs.y, this->z - rhs.z); }
    inline v3d_generic operator*(const T &rhs)            const { return v3d_generic(this->x * rhs, this->y * rhs, this->z * rhs); }
    inline v3d_generic operator*(const v3d_generic &rhs)  const { return v3d_generic(this->x * rhs.x, this->y * rhs.y, this->z * rhs.z); }
    inline v3d_generic operator/(const T &rhs)            const { return v3d_generic(this->x / rhs, this->y / rhs, this->z / rhs); }
    inline v3d_generic operator/(const v3d_generic &rhs)  const { return v3d_generic(this->x / rhs.x, this->y / rhs.y, this->z / rhs.z); }
    inline v3d_generic &operator+=(const v3d_generic &rhs)
    {
        this->x += rhs.x;
        this->y += rhs.y;
        this->z += rhs.z;
        return *this;
    }
    inline v3d_generic &operator-=(const v3d_generic &rhs)
    {
        this->x -= rhs.x;
        this->y -= rhs.y;
        this->z -= rhs.z;
        return *this;
    }
    inline v3d_generic &operator*=(const T &rhs)
    {
        this->x *= rhs;
        this->y *= rhs;
        this->z *= rhs;
        return *this;
    }
    inline v3d_generic &operator/=(const T &rhs)
    {
        this->x /= rhs;
        this->y /= rhs;
        this->z /= rhs;
        return *this;
    }
    inline operator v3d_generic<int32_t>() const { return {static_cast<int32_t>(this->x), static_cast<int32_t>(this->y), static_cast<int32_t>(this->z)}; }
    inline operator v3d_generic<float>()   const { return {static_cast<float>(this->x), static_cast<float>(this->y), static_cast<float>(this->z)}; }
    inline operator v3d_generic<double>()  const { return {static_cast<double>(this->x), static_cast<double>(this->y), static_cast<double>(this->z)}; }
};

typedef v3d_generic<int32_t> vi3d;
typedef v3d_generic<uint32_t> vu3d;
typedef v3d_generic<float> vf3d;
typedef v3d_generic<double> vd3d;

struct line3d
{
    vd3d origin{0, 0, 0};
    vd3d direction{1, 0, 0};
    double epsilon = 0;

    int intersect_ll(const line3d& rhs) const
    {
        auto n = direction.cross(rhs.direction);
        auto dist = (origin - rhs.origin).dot(n) / n.len();
        return dist < epsilon;
    }

    int intersect_ll(const line3d& rhs, double *s) const
    {
        auto n = direction.cross(rhs.direction);
        auto dist = (origin - rhs.origin).dot(n) / n.len();
        auto n2 = rhs.direction.cross(n);
        *s = (origin - rhs.origin).dot(n2) / direction.dot(n2);
        return dist < epsilon;
    }

    int intersect_ll(const line3d& rhs, double *s1, double *s2) const
    {
        // std::cout<<direction.x<<" "<<direction.y<<" "<<direction.z<<"--"<<rhs.direction.x<<" "<<rhs.direction.y<<" "<<rhs.direction.z<<std::endl;
        auto n = direction.cross(rhs.direction);
        // std::cout<<"n = "<<n.x<<" "<<n.y<<" "<<n.z<<std::endl;
        auto dist = n.dot(rhs.origin - origin)/ n.len();
        // std::cout << "dis = " << dist << std::endl;
        auto n1 = direction.cross(n);
        // std::cout<<"n1 = "<<n1.x<<" "<<n1.y<<" "<<n1.z<<std::endl;

        auto n2 = rhs.direction.cross(n);
        // std::cout<<"ndirection2 = "<<n2.x<<" "<<n2.y<<" "<<n2.z<<std::endl;
        // std::cout << s1 << *s1 << std::endl;

        *s1 = ((rhs.origin - origin).dot(n2) / direction.dot(n2))*direction.len();
        // auto dot_p = (rhs.origin - origin).dot(n2);
        // auto rest = direction.dot(n2)*direction.len();
        // *s1 = dot_p/rest;


        // if(*s1 == 0){
        //     std::cout<<"---------------------------------------------"<<std::endl;
        //     std::cout<<direction.x<<" "<<direction.y<<" "<<direction.z<<"--"<<rhs.direction.x<<" "<<rhs.direction.y<<" "<<rhs.direction.z<<std::endl;
        //     std::cout<<"n = "<<n.x<<" "<<n.y<<" "<<n.z<<std::endl;
        //     std::cout << "dis = " << dist << std::endl;
        //     std::cout<<"n1 = "<<n1.x<<" "<<n1.y<<" "<<n1.z<<std::endl;
        //     std::cout<<"n2 = "<<n2.x<<" "<<n2.y<<" "<<n2.z<<std::endl;
        // }
        // std::cout << "*s1 = " << *s1 << std::endl;

        *s2 = ((origin - rhs.origin).dot(n1) / rhs.direction.dot(n1))*rhs.direction.len();
        // auto dot_p = (origin- rhs.origin).dot(n1);
        // auto rest = rhs.direction.dot(n1)*rhs.direction.len();
        // *s2 = dot_p/rest;

        if(std::abs(dist) < epsilon+rhs.epsilon){
            // std::cout<<"n1: "<<n1.x<<" "<<n1.y<<" "<<n1.z<<std::endl;
            // std::cout<<"n2: "<<n2.x<<" "<<n2.y<<" "<<n2.z<<std::endl;
            // std::cout<<"dot: "<<dot_p<<" rest: "<<rest<<std::endl;

        }
        // std::cout << "*s2 = " << *s2 << std::endl;
        return std::abs(dist) < epsilon+rhs.epsilon;
    }

    // int intersect_rl(line3d rhs){
    // auto n = direction.cross(rhs.direction);
    // auto dist = (origin - rhs.origin).dot(n) / n.len();
    // auto n2 = rhs.direction.cross(n);
    // auto s = (origin - rhs.origin).dot(n2) / direction.dot(n2);
    // return (dist < epsilon) && (s > -epsilon) && (s < direction.len());
    //  }

    // int intersect_sl(line3d rhs)
    // {
    //     auto n = direction.cross(rhs.direction);
    //     auto dist = (origin - rhs.origin).dot(n) / n.len();
    //     auto n2 = rhs.direction.cross(n);
    //     auto s = (origin - rhs.origin).dot(n2) / direction.dot(n2);
    //     return (dist < epsilon) && (s > -epsilon) && (s < direction.len());
    // }
    
    inline int intersect_sl(const line3d& rhs) const { double *s=0;                  if (intersect_ll(rhs, s))       return (*s  > -epsilon) && (*s  < (direction.len() + epsilon));     return 0;}

    inline int intersect_sl(const line3d& rhs,  double* s) const {                 if (intersect_ll(rhs, s))       return (*s  > -epsilon) && (*s  < (direction.len() + epsilon));     return 0;}

    inline int intersect_sl(const line3d& rhs,  double* s1, double* s2) const {    if (intersect_ll(rhs, s1, s2))  return (*s1 > -epsilon) && (*s1 < (direction.len() + epsilon));     return 0;}

    inline int intersect_ls(const line3d& rhs) const{                                                            return rhs.intersect_sl(*this);                                              }

    inline int intersect_ls(const line3d& rhs,  double* s) const {                                                return rhs.intersect_sl(*this, s);                                           }

    inline int intersect_ls(const line3d& rhs,  double* s1, double* s2) const {                                   return rhs.intersect_sl(*this, s1, s2);                                      }

    // inline int intersect_rs(line3d rhs)
    inline int intersect_ss(const line3d& rhs) const { double* s1=0; double* s2=0;     if(intersect_ll(rhs, s1, s2))   return (*s1 > -epsilon) &&   //TODO: Fix bug with nullptr
                                                                                                            (*s1 < direction.len()) && 
                                                                                                            (*s2 > -rhs.epsilon) && 
                                                                                                            (*s2 < rhs.direction.len());            return 0;}

    inline int intersect_ss(const line3d& rhs,  double* s1){ double s2 = 0;    if(intersect_ll(rhs, s1, &s2))   return (*s1 > -epsilon) && 
                                                                                                            (*s1 < direction.len()) && 
                                                                                                            (s2 > -rhs.epsilon) && 
                                                                                                            (s2 < rhs.direction.len());            return 0;}

    inline int intersect_ss(const line3d& rhs,  double* s1,  double* s2){       if(intersect_ll(rhs, s1, s2))   return (*s1 > -epsilon) && 
                                                                                                            (*s1 < direction.len()) && 
                                                                                                            (*s2 > -rhs.epsilon) && 
                                                                                                            (*s2 < rhs.direction.len());            return 0;}

    // inline int intersect_ss(line3d rhs)
    // {
    //     auto n = direction.cross(rhs.direction);
    //     auto dist = (origin - rhs.origin).dot(n) / n.len();
    //     auto n1 = direction.cross(n);
    //     auto n2 = rhs.direction.cross(n);
    //     auto s1 = (origin - rhs.origin).dot(n2) / direction.dot(n2);
    //     auto s2 = (origin - rhs.origin).dot(n2) / direction.dot(n2);

    //     return (dist < epsilon) && (s1 > -epsilon) && (s1 < direction.len() && (s2 > -rhs.epsilon) && (s2 < rhs.direction.len()));
    // }



    float dist(line3d rhs)
    {
        auto n = direction.cross(rhs.direction);
        return (origin - rhs.origin).dot(n) / n.len();
    }
};


struct tScreen
{
    vf3d top_left;
    float width;
    float height;


};



class Example : public olc::PixelGameEngine
{
public:
    Example()
    {
        // Name you application
        sAppName = "Example";
    }

public:
    bool OnUserCreate() override
    {
        screen = tScreen{vf3d{-1.5, -1, 1}, 3.0, 2.0};
        lines.push_back(line3d{vf3d{0, 0, 2}, vf3d{1, 0, 0}, 0.01});
        lines.push_back(line3d{vf3d{1, 0, 2}, vf3d{0, 1, 0}, 0.01});
        lines.push_back(line3d{vf3d{1, 1, 2}, vf3d{-1, 0, 0}, 0.01});
        lines.push_back(line3d{vf3d{0, 1, 2}, vf3d{0, -1, 0}, 0.01});

        for (int p_x = 0; p_x < ScreenWidth(); p_x++)
        {
            for (int p_y = 0; p_y < ScreenHeight(); p_y++)
            {
                vf3d direction = screen.top_left + vf3d{screen.width*p_x/ScreenWidth(), screen.height*p_y/ScreenHeight(), 0};
                direction *= 10;

                vf3d origin{0, 0, 0};
                double s1=0;
                double s2=0;
                line3d ray{origin, direction, 0};
                Draw(p_x, p_y, olc::BLACK);
                for(auto& line: lines){
                    if(ray.intersect_ss(line, &s1, &s2)){
                        std::cout<<" "<<direction.x<<" "<<direction.y<<" "<<direction.z<<std::endl;
                        // std::cout<<"-------------------------------------------------------------------------------\n"
                        std::cout<<s1<<" "<<s2<<std::endl;
                        std::cout<<"-------------------------------------------------------------------------------\n";

                        auto color = static_cast<int>(255- (s1*255)/direction.len());
                        Draw(p_x, p_y, olc::Pixel(0, 0, color));

                        continue;
                    }
                }
            }

            // // lines[1].direction.x;
            // double angle = fElapsedTime*speed;
            // // std::cout<<angle<<std::endl;
            // vf3d new_dir{lines[0].direction};
            // new_dir.x = lines[0].direction.dot(vf3d{cos(angle), -sin(angle), 0});
            // new_dir.y = lines[0].direction.dot(vf3d{sin(angle), cos(angle),  0});
            // new_dir.z = lines[0].direction.dot(vf3d{0,             0,      1});
            // // std::cout<<new_dir.x<<" "<<new_dir.y<<" "<<new_dir.z<<std::endl;
            // lines[0].direction = new_dir;

        }
        // speed = 0.01;

        return true;
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        
        // for (int p_x = 0; p_x < ScreenWidth(); p_x++)
        // {
        //     for (int p_y = 0; p_y < ScreenHeight(); p_y++)
        //     {
        //         vf3d direction = screen.top_left + vf3d{screen.width*p_x/ScreenWidth(), screen.height*p_y/ScreenHeight(), 0};
        //         direction *= 10;

        //         vf3d origin{0, 0, 0};
        //         double s1=0;
        //         double s2=0;
        //         line3d ray{origin, direction, 0};
        //         Draw(p_x, p_y, olc::BLACK);
        //         for(auto& line: lines){
        //             if(ray.intersect_ss(line, &s1, &s2)){
        //                 std::cout<<" "<<direction.x<<" "<<direction.y<<" "<<direction.z<<std::endl;
        //                 std::cout<<s1<<" "<<s2<<std::endl;
        //                 auto color = static_cast<int>(255- (s1*255)/direction.len());
        //                 Draw(p_x, p_y, olc::Pixel(0, 0, color));

        //                 continue;
        //             }
        //         }
        //     }

        //     // lines[1].direction.x;
        //     double angle = fElapsedTime*speed;
        //     // std::cout<<angle<<std::endl;
        //     vf3d new_dir{lines[0].direction};
        //     new_dir.x = lines[0].direction.dot(vf3d{cos(angle), -sin(angle), 0});
        //     new_dir.y = lines[0].direction.dot(vf3d{sin(angle), cos(angle),  0});
        //     new_dir.z = lines[0].direction.dot(vf3d{0,             0,      1});
        //     // std::cout<<new_dir.x<<" "<<new_dir.y<<" "<<new_dir.z<<std::endl;
        //     lines[0].direction = new_dir;

        // }

        return true;
    }

private:
    tScreen screen;
    std::vector<line3d> lines;
    double speed;
};

int main()
{
    Example demo;
    if (demo.Construct(400, 300, 1, 1, 0, 0)){
        std::cout<<"demo constructed"<<std::endl;
        demo.Start();
    }
    return 0;
}