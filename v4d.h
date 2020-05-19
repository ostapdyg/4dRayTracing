#ifndef VECTOR_4D_BASIC
#define VECTOR_4D_BASIC

#include <cmath>

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
    // Length
    inline T len() const { return std::sqrt(x * x + y * y + z * z + t * t); }
    // Length squared
    inline T len2() const { return x * x + y * y + z * z + t * t; }
    // Get normalized vector
    inline v4d_generic norm() const
    {
        T r = 1 / len();
        return v4d_generic(x * r, y * r, z * r, t * r);
    }
    // Dot product
    inline T dot(const v4d_generic &rhs) const { return this->x * rhs.x + this->y * rhs.y + this->z * rhs.z + this->t * rhs.t; }
    // 4-d Cross product; set rhs2 = vf4d{0,0,0,1} for 3-d
    inline v4d_generic cross(const v4d_generic &rhs1, const v4d_generic &rhs2 = v4d_generic{0, 0, 0, 1}) const { return v4d_generic(
        this->y * rhs1.z * rhs2.t + this->z * rhs1.t * rhs2.y + this->t * rhs1.y * rhs2.z - this->t * rhs1.z * rhs2.y - this->z * rhs1.y * rhs2.t - this->y * rhs1.t * rhs2.z,
        this->x * rhs1.z * rhs2.t + this->z * rhs1.t * rhs2.x + this->t * rhs1.x * rhs2.z - this->t * rhs1.z * rhs2.x - this->z * rhs1.x * rhs2.t - this->x * rhs1.t * rhs2.z,
        this->x * rhs1.y * rhs2.t + this->y * rhs1.t * rhs2.x + this->t * rhs1.x * rhs2.y - this->t * rhs1.y * rhs2.x - this->y * rhs1.x * rhs2.t - this->x * rhs1.t * rhs2.y,
        this->x * rhs1.y * rhs2.z + this->y * rhs1.z * rhs2.x + this->z * rhs1.x * rhs2.y - this->z * rhs1.y * rhs2.x - this->y * rhs1.x * rhs2.z - this->x * rhs1.z * rhs2.y); }
    inline v4d_generic operator+(const v4d_generic &rhs) const { return v4d_generic(this->x + rhs.x, this->y + rhs.y, this->z + rhs.z, this->t + rhs.t); }
    inline v4d_generic operator-(const v4d_generic &rhs) const { return v4d_generic(this->x - rhs.x, this->y - rhs.y, this->z - rhs.z, this->t - rhs.t); }
    inline v4d_generic operator*(const T &rhs) const { return v4d_generic(this->x * rhs, this->y * rhs, this->z * rhs, this->t * rhs); }
    // Pairwise multiplication
    inline v4d_generic operator*(const v4d_generic &rhs) const { return v4d_generic(this->x * rhs.x, this->y * rhs.y, this->z * rhs.z, this->t * rhs.t); }
    inline v4d_generic operator/(const T &rhs) const { return v4d_generic(this->x / rhs, this->y / rhs, this->z / rhs, this->t / rhs); }
    // Pairwise Division
    inline v4d_generic operator/(const v4d_generic &rhs) const { return v4d_generic(this->x / rhs.x, this->y / rhs.y, this->z / rhs.z, this->t / rhs.t); }
    inline v4d_generic operator%(const T &rhs) const { return v4d_generic(std::fmod(this->x, rhs), this->y, this->z, this->t); }
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

    inline operator v4d_generic<int>() const { return {static_cast<int>(this->x), static_cast<int>(this->y), static_cast<int>(this->z), static_cast<int>(this->z)}; }
    inline operator v4d_generic<float>() const { return {static_cast<float>(this->x), static_cast<float>(this->y), static_cast<float>(this->z), static_cast<float>(this->z)}; }
    inline operator v4d_generic<double>() const { return {static_cast<double>(this->x), static_cast<double>(this->y), static_cast<double>(this->z), static_cast<double>(this->z)}; }
};
typedef v4d_generic<float> vf4d;

template <class T>
struct m4d_generic
{
    v4d_generic<v4d_generic<T>> m;
    inline m4d_generic(v4d_generic<T> _x, v4d_generic<T> _y, v4d_generic<T> _z, v4d_generic<T> _t) : m(_x, _y, _z, _t) {}
    inline m4d_generic(const m4d_generic &rhs) : m(rhs.m) {}
    // inline m4d_generic(m4d_generic rhs) : m(rhs.m) {}
    inline m4d_generic(T x1, T x2, T x3, T x4,
                       T y1, T y2, T y3, T y4,
                       T z1, T z2, T z3, T z4,
                       T t1, T t2, T t3, T t4) : m(v4d_generic{x1, x2, x3, x4},
                                                   v4d_generic{y1, y2, y3, y4},
                                                   v4d_generic{z1, z2, z3, z4},
                                                   v4d_generic{t1, t2, t3, t4}) {}

    inline T operator+(const T &rhs)
    {
        return m4d_generic(m.x + rhs, m.y + rhs, m.z + rhs, m.t + rhs);
    }

    inline T operator-(const T &rhs)
    {
        return m4d_generic(m.x - rhs, m.y - rhs, m.z - rhs, m.t - rhs);
    }

    inline T operator*(const T &rhs)
    {
        return m4d_generic(m.x * rhs, m.y * rhs, m.z * rhs, m.t * rhs);
    }

    inline T operator/(const T &rhs)
    {
        return m4d_generic(m.x / rhs, m.y / rhs, m.z / rhs, m.t / rhs);
    }
    inline m4d_generic &operator+=(const m4d_generic &rhs)
    {
        m.x += rhs.m.x;
        m.y += rhs.m.y;
        m.z += rhs.m.z;
        m.t += rhs.m.t;
        return *this;
    }
    inline m4d_generic &operator-=(const m4d_generic &rhs)
    {
        m.x -= rhs.m.x;
        m.y -= rhs.m.y;
        m.z -= rhs.m.z;
        m.t += rhs.m.t;
        return *this;
    }
    inline m4d_generic &operator*=(const T &rhs)
    {
        m.x *= rhs;
        m.y *= rhs;
        m.z *= rhs;
        m.t *= rhs;
        return *this;
    }
    inline m4d_generic &operator/=(const T &rhs)
    {
        m.x /= rhs;
        m.y /= rhs;
        m.z /= rhs;
        m.t /= rhs;
        return *this;
    }

    inline m4d_generic operator*(const m4d_generic &rhs)
    {
        m4d_generic tr = rhs.transposed();
        return m4d_generic(m.x.dot(tr.m.x), m.x.dot(tr.m.y), m.x.dot(tr.m.z), m.x.dot(tr.m.t),
                           m.y.dot(tr.m.x), m.y.dot(tr.m.y), m.y.dot(tr.m.z), m.y.dot(tr.m.t),
                           m.z.dot(tr.m.x), m.z.dot(tr.m.y), m.z.dot(tr.m.z), m.z.dot(tr.m.t),
                           m.t.dot(tr.m.x), m.t.dot(tr.m.y), m.t.dot(tr.m.z), m.t.dot(tr.m.t));
    }

    inline v4d_generic<T> operator*(const v4d_generic<T> &rhs)
    {
        return v4d_generic(m.x.dot(rhs), m.y.dot(rhs), m.z.dot(rhs), m.t.dot(rhs));
    }

    inline m4d_generic transposed() const
    {
        return m4d_generic(v4d_generic{m.x.x, m.y.x, m.z.x, m.t.x},
                           v4d_generic{m.x.y, m.y.y, m.z.y, m.t.y},
                           v4d_generic{m.x.z, m.y.z, m.z.z, m.t.z},
                           v4d_generic{m.x.t, m.y.t, m.z.t, m.t.t});
    }

    inline T det()
    {
        return (m.x.x * (m.y.y * (m.z.z * m.t.t - m.z.t * m.t.z) - m.y.z * (m.z.y * m.t.t - m.z.t * m.t.y) + m.y.t * (m.z.y * m.t.z - m.z.z * m.t.y)) -
                m.x.y * (m.y.x * (m.z.z * m.t.t - m.z.t * m.t.z) - m.y.z * (m.z.x * m.t.t - m.z.t * m.t.x) + m.y.t * (m.z.x * m.t.z - m.z.z * m.t.x)) +
                m.x.z * (m.y.x * (m.z.y * m.t.t - m.z.t * m.t.y) - m.y.y * (m.z.x * m.t.t - m.z.t * m.t.x) + m.y.t * (m.z.x * m.t.y - m.z.y * m.t.x)) -
                m.x.t * (m.y.x * (m.z.y * m.t.z - m.z.z * m.t.y) - m.y.y * (m.z.x * m.t.z - m.z.z * m.t.x) + m.y.z * (m.z.x * m.t.y - m.z.y * m.t.x)));
    }

    inline m4d_generic adjugate()
    {
        return m4d_generic(v4d_generic{(m.y.y * (m.z.z * m.t.t - m.z.t * m.t.z) - m.y.z * (m.z.y * m.t.t - m.z.t * m.t.y) + m.y.t * (m.z.y * m.t.z - m.z.z * m.t.y)),
                                       -(m.y.x * (m.z.z * m.t.t - m.z.t * m.t.z) - m.y.z * (m.z.x * m.t.t - m.z.t * m.t.x) + m.y.t * (m.z.x * m.t.z - m.z.z * m.t.x)),
                                       (m.y.x * (m.z.y * m.t.t - m.z.t * m.t.y) - m.y.y * (m.z.x * m.t.t - m.z.t * m.t.x) + m.y.t * (m.z.x * m.t.y - m.z.y * m.t.x)),
                                       -(m.y.x * (m.z.y * m.t.z - m.z.z * m.t.y) - m.y.y * (m.z.x * m.t.z - m.z.z * m.t.x) + m.y.z * (m.z.x * m.t.y - m.z.y * m.t.x))},
                           v4d_generic{-(m.x.y * (m.z.z * m.t.t - m.z.t * m.t.z) - m.x.z * (m.z.y * m.t.t - m.z.t * m.t.y) + m.x.t * (m.z.y * m.t.z - m.z.z * m.t.y)),
                                       (m.x.x * (m.z.z * m.t.t - m.z.t * m.t.z) - m.x.z * (m.z.x * m.t.t - m.z.t * m.t.x) + m.x.t * (m.z.x * m.t.z - m.z.z * m.t.x)),
                                       -(m.x.x * (m.z.y * m.t.t - m.z.t * m.t.y) - m.x.y * (m.z.x * m.t.t - m.z.t * m.t.x) + m.x.t * (m.z.x * m.t.y - m.z.y * m.t.x)),
                                       (m.x.x * (m.z.y * m.t.z - m.z.z * m.t.y) - m.x.y * (m.z.x * m.t.z - m.z.z * m.t.x) + m.x.z * (m.z.x * m.t.y - m.z.y * m.t.x))},
                           v4d_generic{(m.x.y * (m.y.z * m.t.t - m.y.t * m.t.z) - m.x.z * (m.y.t * m.t.y - m.y.y * m.t.t) + m.x.t * (m.y.y * m.t.z - m.y.z * m.t.y)),
                                       -(m.x.x * (m.y.z * m.t.t - m.y.t * m.t.x) - m.x.z * (m.y.t * m.t.x - m.y.x * m.t.t) + m.x.t * (m.y.x * m.t.z - m.y.z * m.x.x)),
                                       (m.x.x * (m.y.y * m.t.t - m.y.t * m.t.y) - m.x.y * (m.y.t * m.t.x - m.y.x * m.t.t) + m.x.t * (m.y.x * m.t.y - m.y.y * m.t.x)),
                                       -(m.x.x * (m.y.y * m.t.z - m.y.z * m.t.y) - m.x.y * (m.y.z * m.t.x - m.y.x * m.t.z) + m.x.z * (m.y.x * m.t.y - m.y.y * m.t.x))},
                           v4d_generic{-(m.x.y * (m.y.z * m.z.t - m.y.t * m.z.z) - m.x.y * (m.y.y * m.z.t - m.y.t * m.z.y) + m.x.t * (m.y.y * m.z.z - m.y.z * m.z.y)),
                                       (m.x.x * (m.y.z * m.z.t - m.y.t * m.z.z) - m.x.z * (m.y.x * m.z.t - m.y.t * m.z.x) + m.x.t * (m.y.x * m.z.z - m.y.z * m.z.x)),
                                       -(m.x.x * (m.y.y * m.z.t - m.y.t * m.z.y) - m.x.y * (m.y.x * m.z.t - m.y.t * m.z.x) + m.x.t * (m.y.x * m.z.y - m.y.y * m.z.x)),
                                       (m.x.x * (m.y.y * m.z.z - m.y.z * m.z.y) - m.x.y * (m.y.x * m.z.z - m.y.z * m.z.x) + m.x.z * (m.y.x * m.z.y - m.y.y * m.z.x))});
    }

    inline m4d_generic inverse()
    {
        T determinant = det();
        if (determinant != 0)
        {
            return adjugate() / determinant;
        }
        return m4d_generic(v4d_generic{0, 0, 0, 0}, v4d_generic{0, 0, 0, 0}, v4d_generic{0, 0, 0, 0}, v4d_generic{0, 0, 0, 0});
    }
};
typedef m4d_generic<float> mf4d;

const mf4d ZERO_M{0, 0, 0, 0,
                  0, 0, 0, 0,
                  0, 0, 0, 0,
                  0, 0, 0, 0};

const mf4d IDENTITY_M{1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1};

// mf4d rotation_matrix(vf4d ang){

// }
#endif