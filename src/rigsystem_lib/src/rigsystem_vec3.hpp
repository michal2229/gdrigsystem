#include <cmath>

namespace rigsystem {

struct alignas(32) vec3 {
    template <class T>
    vec3(T _x, T _y, T _z)
        : x(_x)
        , y(_y)
        , z(_z)
    {
    }
    vec3()
        : x(0.f)
        , y(0.f)
        , z(0.f)
    {
    }

    void zero()
    {
        x = 0.f;
        y = 0.f;
        z = 0.f;
    }

    vec3 operator+(const vec3& o) const
    {
        return vec3(x + o.x, y + o.y, z + o.z);
    }

    template <class T>
    vec3 operator+(const T& o) const
    {
        return vec3(x + o, y + o, z + o);
    }

    vec3 operator-(const vec3& o) const
    {
        return vec3(x - o.x, y - o.y, z - o.z);
    }

    template <class T>
    vec3 operator-(const T& o) const
    {
        return vec3(x - o, y - o, z - o);
    }

    vec3 operator*(const vec3& o) const
    {
        return vec3(x * o.x, y * o.y, z * o.z);
    }

    template <class T>
    vec3 operator*(const T& o) const
    {
        return vec3(x * o, y * o, z * o);
    }

    vec3 operator/(const vec3& o) const
    {
        return vec3(x / o.x, y / o.y, z / o.z);
    }

    template <class T>
    vec3 operator/(const T& o) const
    {
        return vec3(x / o, y / o, z / o);
    }

    vec3& operator+=(const vec3& o)
    {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }

    vec3& operator-=(const vec3& o)
    {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }

    vec3& operator*=(const vec3& o)
    {
        x *= o.x;
        y *= o.y;
        z *= o.z;
        return *this;
    }

    vec3& operator/=(const vec3& o)
    {
        x /= o.x;
        y /= o.y;
        z /= o.z;
        return *this;
    }

    template <class T>
    vec3& operator+=(const T& o)
    {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }

    template <class T>
    vec3& operator-=(const T& o)
    {
        x -= o;
        y -= o;
        z -= o;
        return *this;
    }

    template <class T>
    vec3& operator*=(const T& o)
    {
        x *= o;
        y *= o;
        z *= o;
        return *this;
    }

    template <class T>
    vec3& operator/=(const T& o)
    {
        x /= o;
        y /= o;
        z /= o;
        return *this;
    }

    inline float dot(const vec3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }

    inline vec3 cross(const vec3& o) const noexcept
    {
        return vec3(y * o.z - z * o.y, y * o.x - x * o.z, x * o.y - y * o.x);
    }

     float length2() const { return dot(*this); }

    inline float length() const noexcept { return std::sqrt(length2()); }

    inline float distance_to(const vec3& o) const noexcept { return (*this - o).length(); }

    inline vec3 normalized() const noexcept
    {
        float l = length();
        return (l > 0.0f) ? (*this / l) : vec3(0.f, 0.f, 0.f);
    }

    float x, y, z;
};

template <class T>
vec3 operator+(const T& l, const vec3& r)
{
    return vec3(l + r.x, l + r.y, l + r.z);
}

template <class T>
vec3 operator-(const T& l, const vec3& r)
{
    return vec3(l - r.x, l - r.y, l - r.z);
}

template <class T>
vec3 operator*(const T& l, const vec3& r)
{
    return vec3(l * r.x, l * r.y, l * r.z);
}

template <class T>
vec3 operator/(const T& l, const vec3& r)
{
    return vec3(l / r.x, l / r.y, l / r.z);
}

} // namespace rigsystem
