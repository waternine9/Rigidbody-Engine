#pragma once
#include <math.h>
class PhysVector2
{
public:
    double x, y;
    PhysVector2()
    {

    }
    PhysVector2(double x, double y)
    {
        this->x = x;
        this->y = y;
    }
    PhysVector2 operator+(const PhysVector2 other)
    {
        return { x + other.x, y + other.y };
    }
    PhysVector2 operator-(const PhysVector2 other)
    {
        return { x - other.x, y - other.y };
    }
    PhysVector2 operator*(const PhysVector2 other)
    {
        return { x * other.x, y * other.y };
    }
    PhysVector2 operator/(const PhysVector2 other)
    {
        return { x / other.x, y / other.y };
    }
};
class PhysVector3
{
public:
    double x, y, z;
    PhysVector3()
    {

    }
    PhysVector3(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    PhysVector3 operator+(const PhysVector3 other)
    {
        return { x + other.x, y + other.y, z + other.z };
    }
    PhysVector3 operator-(const PhysVector3 other)
    {
        return { x - other.x, y - other.y, z - other.z };
    }
    PhysVector3 operator*(const PhysVector3 other)
    {
        return { x * other.x, y * other.y, z * other.z };
    }
    PhysVector3 operator/(const PhysVector3 other)
    {
        return { x / other.x, y / other.y, z / other.z };
    }
};
class PhysVector4
{
public:
    double x, y, z, w;
    PhysVector4()
    {

    }
    PhysVector4(double x, double y, double z, double w)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }
    PhysVector4 operator+(const PhysVector4 other)
    {
        return { x + other.x, y + other.y, z + other.z, w + other.w };
    }
    PhysVector4 operator-(const PhysVector4 other)
    {
        return { x - other.x, y - other.y, z - other.z, w - other.w };
    }
    PhysVector4 operator*(const PhysVector4 other)
    {
        return { x * other.x, y * other.y, z * other.z, w * other.w };
    }
    PhysVector4 operator/(const PhysVector4 other)
    {
        return { x / other.x, y / other.y, z / other.z, w / other.w };
    }
};

class Matrix4x4
{
public:
    double data[4][4];

    Matrix4x4() {};
    Matrix4x4(PhysVector4 row0, PhysVector4 row1, PhysVector4 row2, PhysVector4 row3)
    {
        data[0][0] = row0.x;
        data[0][1] = row0.y;
        data[0][2] = row0.z;
        data[0][3] = row0.w;

        data[1][0] = row1.x;
        data[1][1] = row1.y;
        data[1][2] = row1.z;
        data[1][3] = row1.w;

        data[2][0] = row2.x;
        data[2][1] = row2.y;
        data[2][2] = row2.z;
        data[2][3] = row2.w;

        data[3][0] = row3.x;
        data[3][1] = row3.y;
        data[3][2] = row3.z;
        data[3][3] = row3.w;
    }

    double* operator[](const int& x)
    {
        return data[x];
    }
    Matrix4x4 operator*(Matrix4x4 mat2)
    {
        Matrix4x4 result;
        result[0][0] = data[0][0] * mat2[0][0] + data[0][1] * mat2[1][0] + data[0][2] * mat2[2][0] + data[0][3] * mat2[3][0];
        result[0][1] = data[0][0] * mat2[0][1] + data[0][1] * mat2[1][1] + data[0][2] * mat2[2][1] + data[0][3] * mat2[3][1];
        result[0][2] = data[0][0] * mat2[0][2] + data[0][1] * mat2[1][2] + data[0][2] * mat2[2][2] + data[0][3] * mat2[3][2];
        result[0][3] = data[0][0] * mat2[0][3] + data[0][1] * mat2[1][3] + data[0][2] * mat2[2][3] + data[0][3] * mat2[3][3];

        result[1][0] = data[1][0] * mat2[0][0] + data[1][1] * mat2[1][0] + data[1][2] * mat2[2][0] + data[1][3] * mat2[3][0];
        result[1][1] = data[1][0] * mat2[0][1] + data[1][1] * mat2[1][1] + data[1][2] * mat2[2][1] + data[1][3] * mat2[3][1];
        result[1][2] = data[1][0] * mat2[0][2] + data[1][1] * mat2[1][2] + data[1][2] * mat2[2][2] + data[1][3] * mat2[3][2];
        result[1][3] = data[1][0] * mat2[0][3] + data[1][1] * mat2[1][3] + data[1][2] * mat2[2][3] + data[1][3] * mat2[3][3];

        result[2][0] = data[2][0] * mat2[0][0] + data[2][1] * mat2[1][0] + data[2][2] * mat2[2][0] + data[2][3] * mat2[3][0];
        result[2][1] = data[2][0] * mat2[0][1] + data[2][1] * mat2[1][1] + data[2][2] * mat2[2][1] + data[2][3] * mat2[3][1];
        result[2][2] = data[2][0] * mat2[0][2] + data[2][1] * mat2[1][2] + data[2][2] * mat2[2][2] + data[2][3] * mat2[3][2];
        result[2][3] = data[2][0] * mat2[0][3] + data[2][1] * mat2[1][3] + data[2][2] * mat2[2][3] + data[2][3] * mat2[3][3];

        result[3][0] = data[3][0] * mat2[0][0] + data[3][1] * mat2[1][0] + data[3][2] * mat2[2][0] + data[3][3] * mat2[3][0];
        result[3][1] = data[3][0] * mat2[0][1] + data[3][1] * mat2[1][1] + data[3][2] * mat2[2][1] + data[3][3] * mat2[3][1];
        result[3][2] = data[3][0] * mat2[0][2] + data[3][1] * mat2[1][2] + data[3][2] * mat2[2][2] + data[3][3] * mat2[3][2];
        result[3][3] = data[3][0] * mat2[0][3] + data[3][1] * mat2[1][3] + data[3][2] * mat2[2][3] + data[3][3] * mat2[3][3];
        return result;
    }
    Matrix4x4 operator+(Matrix4x4 mat2)
    {
        Matrix4x4 result = *this;
        result[0][0] += mat2[0][0];
        result[0][1] += mat2[0][1];
        result[0][2] += mat2[0][2];
        result[0][3] += mat2[0][3];

        result[1][0] += mat2[1][0];
        result[1][1] += mat2[1][1];
        result[1][2] += mat2[1][2];
        result[1][3] += mat2[1][3];

        result[2][0] += mat2[2][0];
        result[2][1] += mat2[2][1];
        result[2][2] += mat2[2][2];
        result[2][3] += mat2[2][3];

        result[3][0] += mat2[3][0];
        result[3][1] += mat2[3][1];
        result[3][2] += mat2[3][2];
        result[3][3] += mat2[3][3];
        return result;
    }
    PhysVector4 operator*(const PhysVector4& vec)
    {
        PhysVector4 result;
        result.x = data[0][0] * vec.x + data[0][1] * vec.y + data[0][2] * vec.z + data[0][3] * vec.w;
        result.y = data[1][0] * vec.x + data[1][1] * vec.y + data[1][2] * vec.z + data[1][3] * vec.w;
        result.z = data[2][0] * vec.x + data[2][1] * vec.y + data[2][2] * vec.z + data[2][3] * vec.w;
        result.w = data[3][0] * vec.x + data[3][1] * vec.y + data[3][2] * vec.z + data[3][3] * vec.w;
        return result;
    }
    PhysVector3 operator*(const PhysVector3& vec)
    {
        PhysVector3 result;
        result.x = data[0][0] * vec.x + data[0][1] * vec.y + data[0][2] * vec.z;
        result.y = data[1][0] * vec.x + data[1][1] * vec.y + data[1][2] * vec.z;
        result.z = data[2][0] * vec.x + data[2][1] * vec.y + data[2][2] * vec.z;
        return result;
    }
};

double dot(PhysVector2 x, PhysVector2 y)
{
    return { x.x * y.x + x.y * y.y };
}

double dot(PhysVector3 x, PhysVector3 y)
{
    return { x.x * y.x + x.y * y.y + x.z * y.z };
}

double dot(PhysVector4 x, PhysVector4 y)
{
    return { x.x * y.x + x.y * y.y + x.z * y.z + x.w * y.w };
}

PhysVector3 cross(PhysVector3 x, PhysVector3 y)
{
    return {
        x.y * y.z - x.z * y.y,
        x.z * y.x - x.x * y.z,
        x.x * y.y - x.y * y.x
    };
}

PhysVector3 normalize(PhysVector3 x)
{

    double mag = 1.0f / sqrtf(x.x * x.x + x.y * x.y + x.z * x.z);

    x.x *= mag;
    x.y *= mag;
    x.z *= mag;
    return x;
}

double magnitude(PhysVector2 x)
{
    return sqrtf(x.x * x.x + x.y * x.y);
}

double magnitude(PhysVector3 x)
{
    return sqrtf(x.x * x.x + x.y * x.y + x.z * x.z);
}

PhysVector3 scale(PhysVector3 x, double y)
{
    return { x.x * y, x.y * y, x.z * y };
}
PhysVector2 scale(PhysVector2 x, double y)
{
    return { x.x * y, x.y * y };
}

PhysVector3 reflect(PhysVector3 x, PhysVector3 normal)
{
    return x - scale(normal, 2 * dot(x, normal));
}

PhysVector3 min(PhysVector3 x, PhysVector3 y)
{
    x.x = std::min(x.x, y.x);
    x.y = std::min(x.y, y.y);
    x.z = std::min(x.z, y.z);
    return x;
}

PhysVector3 max(PhysVector3 x, PhysVector3 y)
{
    x.x = std::max(x.x, y.x);
    x.y = std::max(x.y, y.y);
    x.z = std::max(x.z, y.z);
    return x;
}

Matrix4x4 identity()
{
    Matrix4x4 output;


    output[0][0] = 1;
    output[0][1] = 0;
    output[0][2] = 0;
    output[0][3] = 0;

    output[1][0] = 0;
    output[1][1] = 1;
    output[1][2] = 0;
    output[1][3] = 0;

    output[2][0] = 0;
    output[2][1] = 0;
    output[2][2] = 1;
    output[2][3] = 0;

    output[3][0] = 0;
    output[3][1] = 0;
    output[3][2] = 0;
    output[3][3] = 1;
    return output;
}

Matrix4x4 translate(PhysVector3 moveBy)
{
    Matrix4x4 output;


    output[0][0] = 1;
    output[0][1] = 0;
    output[0][2] = 0;
    output[0][3] = moveBy.x;

    output[1][0] = 0;
    output[1][1] = 1;
    output[1][2] = 0;
    output[1][3] = moveBy.y;

    output[2][0] = 0;
    output[2][1] = 0;
    output[2][2] = 1;
    output[2][3] = moveBy.z;

    output[3][0] = 0;
    output[3][1] = 0;
    output[3][2] = 0;
    output[3][3] = 1;
    return output;
}

#undef near
#undef far

Matrix4x4 perspective(double fov, double aspect, double near, double far)
{
    Matrix4x4 output;
    double yScale = 1.0f / tanf(fov / 2);
    double xScale = yScale / aspect;
    double neardif = near - far;

    output[0][0] = xScale;
    output[0][1] = 0;
    output[0][2] = 0;
    output[0][3] = 0;

    output[1][0] = 0;
    output[1][1] = yScale;
    output[1][2] = 0;
    output[1][3] = 0;

    output[2][0] = 0;
    output[2][1] = 0;
    output[2][2] = (far) / neardif;
    output[2][3] = -1;

    output[3][0] = 0;
    output[3][1] = 0;
    output[3][2] = far * near / neardif;
    output[3][3] = 0;
    return output;
}

Matrix4x4 lookat(PhysVector3 origin, PhysVector3 target, PhysVector3 up)
{

    PhysVector3 forward = normalize(target - origin);

    PhysVector3 right = normalize(cross(forward, up));

    up = normalize(cross(forward, right));

    Matrix4x4 output;
    output[0][0] = right.x;
    output[0][1] = up.x;
    output[0][2] = forward.x;
    output[0][3] = 0;

    output[1][0] = right.y;
    output[1][1] = up.y;
    output[1][2] = forward.y;
    output[1][3] = 0;

    output[2][0] = right.z;
    output[2][1] = up.z;
    output[2][2] = forward.z;
    output[2][3] = 0;

    output[3][0] = -dot(origin, right);
    output[3][1] = -dot(origin, up);
    output[3][2] = -dot(origin, forward);
    output[3][3] = 1;
    return output;
}

Matrix4x4 rotate(PhysVector3 axis)
{
    Matrix4x4 accum = identity();
    Matrix4x4 output;

    output[3][0] = 0;
    output[3][1] = 0;
    output[3][2] = 0;
    output[3][3] = 1;
    output[0][3] = 0;
    output[1][3] = 0;
    output[2][3] = 0;
    if (axis.x != 0.0f)
    {


        double cosr = cos(axis.x);
        double sinr = sin(axis.x);
        output[0][0] = 1;
        output[0][1] = 0;
        output[0][2] = 0;

        output[1][0] = 0;
        output[1][1] = cosr;
        output[1][2] = sinr;

        output[2][0] = 0;
        output[2][1] = -sinr;
        output[2][2] = cosr;

        accum = accum * output;
    }
    if (axis.y != 0.0f)
    {


        double cosr = cos(axis.y);
        double sinr = sin(axis.y);
        output[0][0] = cosr;
        output[0][1] = 0;
        output[0][2] = -sinr;

        output[1][0] = 0;
        output[1][1] = 1;
        output[1][2] = 0;

        output[2][0] = sinr;
        output[2][1] = 0;
        output[2][2] = cosr;

        accum = accum * output;
    }
    if (axis.z != 0.0f)
    {


        double cosr = cos(axis.z);
        double sinr = sin(axis.z);
        output[0][0] = cosr;
        output[0][1] = sinr;
        output[0][2] = 0;

        output[1][0] = -sinr;
        output[1][1] = cosr;
        output[1][2] = 0;

        output[2][0] = 0;
        output[2][1] = 0;
        output[2][2] = 1;

        accum = accum * output;
    }

    return accum;
}

class PhysQuaternion
{
public:
    double x, y, z, w;

    PhysQuaternion operator*(PhysQuaternion b)
    {
        return {
        
            w * b.x + x * b.w + y * b.z - z * b.y,  // i
            w * b.y - x * b.z + y * b.w + z * b.x,  // j
            w * b.z + x * b.y - y * b.x + z * b.w,   // k
            w * b.w - x * b.x - y * b.y - z * b.z  // 1
        };
    }
    Matrix4x4 ToRotationMatrix()
    {
        Matrix4x4 m = identity();
        m[0][0] = w * w + x * x - y * y - z * z;
        m[0][1] = 2 * x * y + 2 * z * w;
        m[0][2] = 2 * x * z - 2 * y * w;
        m[1][0] = 2 * x * y - 2 * z * w;
        m[1][1] = w * w - x * x + y * y - z * z;
        m[1][2] = 2 * y * z + 2 * x * w;
        m[2][0] = 2 * z * x + 2 * y * w;
        m[2][1] = 2 * z * y - 2 * x * w;
        m[2][2] = w * w - x * x - y * y + z * z;
        return m;
    }

    PhysVector3 operator*(PhysVector3 v) {
        PhysVector4 p(
            w * v.x + y * v.z - z * v.y,
            w * v.y + z * v.x - x * v.z,
            w * v.z + x * v.y - y * v.x,
            -x * v.x - y * v.y - z * v.z
        );

        return PhysVector3(
            w * p.x - p.y * z + p.z * y - p.w * x,
            w * p.y - p.z * x + p.x * z - p.w * y,
            w * p.z - p.x * y + p.y * x - p.w * z
        );
    }
    PhysQuaternion operator+(PhysQuaternion v) {
        return { x + v.x, y + v.y, z + v.z, w + v.w };
    }
    PhysQuaternion operator*(double v) {
        return { x * v, y * v, z * v, w * v };
    }
    PhysQuaternion normalize()
    {
        double _normal = 1.0 / sqrt(normal());
        PhysQuaternion q = *this;
        q.x *= _normal;
        q.y *= _normal;
        q.z *= _normal;
        return q;
    }
    double normal()
    {
        PhysVector4 tov4f(x, y, z, w);
        return dot(tov4f, tov4f);
    }
};

PhysQuaternion slerp(PhysQuaternion q1, PhysQuaternion q2, double t) {
    double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

    if (dot < 0.0) {
        dot = -dot;
        PhysQuaternion temp = { -q2.w, -q2.x, -q2.y, -q2.z };
        q2 = temp;
    }

    double angle = std::acos(dot);
    double sin_inv = 1.0 / std::sin(angle);

    double coef1 = std::sin((1.0 - t) * angle) * sin_inv;
    double coef2 = std::sin(t * angle) * sin_inv;

    PhysQuaternion result;
    result.w = coef1 * q1.w + coef2 * q2.w;
    result.x = coef1 * q1.x + coef2 * q2.x;
    result.y = coef1 * q1.y + coef2 * q2.y;
    result.z = coef1 * q1.z + coef2 * q2.z;

    return result;
}

PhysQuaternion inverse(PhysQuaternion q)
{
    PhysQuaternion conjugate = { -q.x, -q.y, -q.z, q.w };
    double normsq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return { conjugate.x / normsq, conjugate.y / normsq, conjugate.z / normsq, conjugate.w / normsq };
}