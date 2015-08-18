#ifndef _VEC3_H
#define _VEC3_H

#include <vector>
#include "Utils.h"

namespace YAX
{
    struct Matrix;
    struct Quaternion;
    struct Vector2;

    struct Vector3
    {
        static const Vector3 One, UnitX, UnitY, UnitZ, Zero, 
                             Backward, Down, Forward, Left,
                             Right, Up;

        float X, Y, Z;

        Vector3();
        Vector3(float val);
        Vector3(float x, float y, float z);
        Vector3(Vector2 xy, float z);

        /**
        * @brief Normalizes the vector, maintaining direction but reducing its length to 1
        */
        void Normalize();

        /**
        * @brief Gets the length of the vector
        */
        float Length() const;

        /**
        * @brief Gets the squared length of the vector
        */
        float LengthSquared() const;

        /**
        * @brief Performs a barycentric interpolation over a triangle defined by three Vector3s
        *
        * @param p1 The first point of the triangle
        * @param p2 The second point of the triangle
        * @param p3 The third point of the triangle
        * @param b2 The weight factor of the second point
        * @param b3 The weight factor of the third point
        * @return The calculated barycentric point
        */
        static Vector3 Barycentric(const Vector3& p1, const Vector3& p2, const Vector3& p3, float b2, float b3);

        /**
        * @brief Performs a Catmull-Rom interpolation between the second and third points of a spline
        *
        * @param p1 The first point of the spline
        * @param p2 The second point of the spline
        * @param p3 The third point of the spline
        * @param p4 The fourth point of the spline
        * @param t The interpolation weight
        * @return The calculated point on the spline
        */
        static Vector3 CatmullRom(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& p4, float t);

        /**
        * @brief Clamps a Vector3's components between a min and max value
        *
        * @param point The Vector3 to clamp
        * @param min The minimum values for each coordinate
        * @param max The maximum values for each coordinate
        * @return The clamped Vector3
        */
        static Vector3 Clamp(const Vector3& point, const Vector3& min, const Vector3& max);
        
        /**
        * @brief Calculates the cross product of two Vector3s
        */
        static Vector3 Cross(const Vector3& v1, const Vector3 v2);

        /**
        * @brief Calculates the signed distance between two points represented by Vector3s
        */
        static float Distance(const Vector3& p1, const Vector3& p2);

        /**
        * @brief Calculates the squared distance between two points represented by Vector3s
        */
        static float DistanceSquared(const Vector3& p1, const Vector3& p2);

        /**
        * @brief Calculates the dot product between two Vector3s
        */
        static float Dot(const Vector3& v1, const Vector3& v2);

        /**
        * @brief Performs a Hermite Spline interpolation between two points
        *
        * @param p1 The first point of the spline
        * @param t1 The slope of the spline at the first point
        * @param p2 The second point of the spline
        * @param t2 The slope of the spline at the second point
        * @param w The interpolation weight
        * @return The calculated point on the spline
        */
        static Vector3 Hermite(const Vector3& p1, const Vector3& t1, const Vector3& p2, const Vector3& t2, float w);
        
        /**
        * @brief Performs a linear interpolation between two points
        *
        * @param p1 The first point
        * @param p2 The second point
        * @param t The interpolation weight
        * @return The calculated point on the line between p1 and p2
        */
        static Vector3 Lerp(const Vector3& p1, const Vector3& p2, float t);

        /**
        * @brief Finds the maximum of each component between two Vector3s
        *
        * @return The Vector3 containing the maximum X coordinate and the maximum Y coordinate
        */
        static Vector3 Max(const Vector3& v1, const Vector3& v2);

        /**
        * @brief Finds the minimum of each component between two Vector3s
        *
        * @return The Vector3 containing the minimum X coordinate and the minimum Y coordinate
        */
        static Vector3 Min(const Vector3& v1, const Vector3& v2);

        /**
        * @brief Normalizes a given vector
        *
        * @return The normalized vector
        */
        static Vector3 Normalize(Vector3 vec);

        /**
        * @brief Reflects a Vector3 across a given normal;
        *
        * @param vec The vector to reflect
        * @param normal The normal to reflect across
        * @return The reflected Vector3
        */
        static Vector3 Reflect(const Vector3& vec, const Vector3& normal);

        /**
        * @brief Performs a Cubic Hermite interpolation between two Vector3s
        *
        * @param v1 The start (t = 0) Vector3
        * @param v2 The end (t = 1) Vector3
        * @param t The interpolation weight
        * @return The calculated Vector3 between v1 and v2
        */
        static Vector3 SmoothStep(const Vector3& v1, const Vector3& v2, float t);

        /**
        * @brief Transforms a Vector3 by a matrix
        *
        * @param v The vector to transform
        * @param m The transformation matrix
        * @return The transformed Vector3
        */
        static Vector3 Transform(const Vector3& v, const Matrix& m);

        /**
        * @brief Rotates a Vector3 by a quaternion
        *
        * @param v The Vector3 to rotate
        * @param q The quaternion to apply
        * @return The rotated Vector3
        */
        static Vector3 Transform(const Vector3& v, const Quaternion& q);

        /**
        * @brief Transforms a list of Vector3s by a matrix and stores them in another list
        *
        * @param source The list of Vector3s to transform
        * @param sourceIdx The index of the first Vector3 in source to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector3s into
        * @param destIdx The index of the first Vector3 in dest to replace
        * @param count The number of Vector3s to transform
        */
        static void Transform(const std::vector<Vector3>& source, ui32 sourceIdx, const Matrix& mat, std::vector<Vector3>& dest, ui32 destIdx, ui32 count);
        
        /**
        * @brief Rotates a list of Vector3s by a quaternion and stores them in another list
        *
        * @param source The list of Vector3s to rotate
        * @param sourceIdx The index of the first Vector3 in source to rotate
        * @param q The quaternion to apply
        * @param dest The list to insert the rotated Vector3s into
        * @param destIdx The index of the first Vector3 in dest to replace
        * @param count The number of Vector3s to transform
        */
        static void Transform(const std::vector<Vector3>& source, ui32 sourceIdx, const Quaternion& q, std::vector<Vector3>& dest, ui32 destIdx, ui32 count);

        /**
        * @brief Transforms all of the Vector3s in a list by a matrix
        *
        * @param source The list of Vector3s to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector3s into
        */
        static void Transform(const std::vector<Vector3>& source, const Matrix& mat, std::vector<Vector3>& dest);

        /**
        * @brief Rotates all of the Vector3s in a list by a quaternion and stores them in another list
        *
        * @param source The list of Vector3s to rotate
        * @param q The quaternion to apply
        * @param dest The list to insert the rotated Vector3s into
        */
        static void Transform(const std::vector<Vector3>& source, const Quaternion& q, std::vector<Vector3>& dest);

        /**
        * @brief Transforms a normal vector by a matrix without applying translation
        *
        * @param normal The Vector3 to transform
        * @param mat The transformation matrix
        * @return The transformed normal
        */
        static Vector3 TransformNormal(const Vector3& normal, const Matrix& mat);

        /**
        * @brief Transforms a list of normals by a matrix without applying translation
        *
        * @param source The list of Vector3s to transform
        * @param sourceIdx The index of the first Vector3 to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector3s into
        * @param destIdx The index of the first Vector3 in dest to replace
        * @param count The number of Vector3s to transform
        */
        static void TransformNormal(const std::vector<Vector3>& source, ui32 sourceIdx, const Matrix& mat, std::vector<Vector3>& dest, ui32 destIdx, ui32 count);

        /**
        * @brief Transforms all of the normals in a list by a matrix
        *
        * @param source The list of Vector3s to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector3s into
        */
        static void TransformNormal(const std::vector<Vector3>& source, const Matrix& mat, std::vector<Vector3>& dest);

        Vector3& operator+=(const Vector3&);
        Vector3& operator-=(const Vector3&);
        Vector3& operator*=(const Vector3&);
        Vector3& operator*=(float);
        Vector3& operator/=(const Vector3&);
        Vector3& operator/=(float);
    };

    Vector3 operator+(const Vector3&, const Vector3&);
    Vector3 operator-(const Vector3&, const Vector3&);
    Vector3 operator*(const Vector3&, const Vector3&);
    Vector3 operator*(const Vector3&, float);
    Vector3 operator*(float, const Vector3&);
    Vector3 operator/(const Vector3&, const Vector3&);
    Vector3 operator/(const Vector3&, float);
    Vector3 operator-(const Vector3&);

    bool operator==(const Vector3&, const Vector3&);
    bool operator!=(const Vector3&, const Vector3&);
    bool operator>(const Vector3&, const Vector3&);
    bool operator<(const Vector3&, const Vector3&);
    bool operator>=(const Vector3&, const Vector3&);
    bool operator<=(const Vector3&, const Vector3&);
}


#endif