#ifndef VECTOR_4D_BASIC
#define VECTOR_4D_BASIC

template <class T>
struct v4d_generic
{
    T x = 0;
    T y = 0;
    T z = 0;
    T t = 0;
    inline v4d_generic() : x(0), y(0), z(0), t(0) {}
    inline v4d_generic(T _x, T _y, T _z, T _t) : x(_x), y(_y), z(_z), t(_t) {}
    inline v4d_generic(const v4d_generic &v) : x(v.x), y(v.y), z(v.z), t(v.t) {}
    inline T len() const { return std::sqrt(x * x + y * y + z * z + t * t); }
    inline T len2() const { return x * x + y * y + z * z + t * t; }
    inline v4d_generic norm() const
    {
        T r = 1 / len();
        return v4d_generic(x * r, y * r, z * r, t * r);
    }
    inline T           dot(const v4d_generic &rhs)        const { return this->x * rhs.x + this->y * rhs.y + this->z * rhs.z + this->t * rhs.t; }
    inline v4d_generic cross(const v4d_generic &rhs1, const v4d_generic &rhs2)      const { return v4d_generic(
                                                                                             this->y * rhs1.z*rhs2.t + this->z*rhs1.t*rhs2.y + this->t*rhs1.y*rhs2.z - this->t*rhs1.z*rhs2.y-this->z*rhs1.y*rhs2.t - this->y*rhs1.t*rhs2.z,
                                                                                             this->x * rhs1.z*rhs2.t + this->z*rhs1.t*rhs2.x + this->t*rhs1.x*rhs2.z - this->t*rhs1.z*rhs2.x-this->z*rhs1.x*rhs2.t - this->x*rhs1.t*rhs2.z,
                                                                                             this->x * rhs1.y*rhs2.t + this->y*rhs1.t*rhs2.x + this->t*rhs1.x*rhs2.y - this->t*rhs1.y*rhs2.x-this->y*rhs1.x*rhs2.t - this->x*rhs1.t*rhs2.y,
                                                                                             this->x * rhs1.y*rhs2.z + this->y*rhs1.z*rhs2.x + this->z*rhs1.x*rhs2.y - this->z*rhs1.y*rhs2.x-this->y*rhs1.x*rhs2.z - this->x*rhs1.z*rhs2.y
                                                                                        ); }
    inline v4d_generic operator+(const v4d_generic &rhs)  const { return v4d_generic(this->x + rhs.x, this->y + rhs.y, this->z + rhs.z, this->t + rhs.t); }
    inline v4d_generic operator-(const v4d_generic &rhs)  const { return v4d_generic(this->x - rhs.x, this->y - rhs.y, this->z - rhs.z, this->t - rhs.t); }
    inline v4d_generic operator*(const T &rhs)            const { return v4d_generic(this->x * rhs, this->y * rhs, this->z * rhs, this->t * rhs); }
    inline v4d_generic operator*(const v4d_generic &rhs)  const { return v4d_generic(this->x * rhs.x, this->y * rhs.y, this->z * rhs.z, this->t * rhs.t); }
    inline v4d_generic operator/(const T &rhs)            const { return v4d_generic(this->x / rhs, this->y / rhs, this->z / rhs, this->t / rhs); }
    inline v4d_generic operator/(const v4d_generic &rhs)  const { return v4d_generic(this->x / rhs.x, this->y / rhs.y, this->z / rhs.z, this->t / rhs.t); }
    inline v4d_generic &operator+=(const v4d_generic &rhs)
    {
        this->x += rhs.x;
        this->y += rhs.y;
        this->z += rhs.z;
        this->t += rhs.t;
        return *this;
    }
    inline v4d_generic &operator-=(const v4d_generic &rhs)
    {
        this->x -= rhs.x;
        this->y -= rhs.y;
        this->z -= rhs.z;
        this->t += rhs.t;
        return *this;
    }
    inline v4d_generic &operator*=(const T &rhs)
    {
        this->x *= rhs;
        this->y *= rhs;
        this->z *= rhs;
        this->t *= rhs;
        return *this;
    }
    inline v4d_generic &operator/=(const T &rhs)
    {
        this->x /= rhs;
        this->y /= rhs;
        this->z /= rhs;
        this->t /= rhs;
        return *this;
    }

    inline operator v4d_generic<int>()     const { return {static_cast<int>(this->x),     static_cast<int>(this->y),     static_cast<int>(this->z),     static_cast<int>(this->z)    }; }
    inline operator v4d_generic<float>()   const { return {static_cast<float>(this->x),   static_cast<float>(this->y),   static_cast<float>(this->z),   static_cast<float>(this->z)  }; }
    inline operator v4d_generic<double>()  const { return {static_cast<double>(this->x),  static_cast<double>(this->y),  static_cast<double>(this->z),  static_cast<double>(this->z) }; }
};

#endif