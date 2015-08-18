#ifndef _VEC4_H
#define _VEC4_H

#include <vector>
#include "Utils.h"

namespace YAX
{
    struct Vector2;
    struct Vector3;
    struct Matrix;
    struct Quaternion;

    struct Vector4
    {
        static const Vector4 One, UnitX, UnitY, UnitZ, UnitW, Zero;
        
        float X, Y, Z, W;
        
        Vector4();
        Vector4(float val);
        Vector4(float x, float y, float z, float w);
        Vector4(Vector2 xy, float z, float w);
        Vector4(Vector3 xyz, float w);

        /**
        * @brief Normalizes the vector, maintaining direction but reducing its length to 1
        */
        void Normalize();

        /**
        * @brief Gets the length of the vector
        */
        float Length();

        /**
        * @brief Gets the squared length of the vector
        */
        float LengthSquared();


        /**
        * @brief Performs a barycentric interpolation over a triangle defined by three Vector4s
        *
        * @param p1 The first point of the triangle
        * @param p2 The second point of the triangle
        * @param p3 The third point of the triangle
        * @param b2 The weight factor of the second point
        * @param b3 The weight factor of the third point
        * @return The calculated barycentric point
        */
        static Vector4 Barycentric(const Vector4& p1, const Vector4& p2, const Vector4& p3, float b2, float b3);

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
        static Vector4 CatmullRom(const Vector4& p1, const Vector4& p2, const Vector4& p3, const Vector4& p4, float t);

        /**
        * @brief Clamps a Vector4's components between a min and max value
        *
        * @param point The Vector4 to clamp
        * @param min The minimum values for each coordinate
        * @param max The maximum values for each coordinate
        * @return The clamped Vector4
        */
        static Vector4 Clamp(const Vector4& point, const Vector4& min, const Vector4& max);

        /**
        * @brief Calculates the signed distance between two points represented by Vector4s
        */
        static float Distance(const Vector4& p1, const Vector4& p2);

        /**
        * @brief Calculates the squared distance between two points represented by Vector4s
        */
        static float DistanceSquared(const Vector4& p1, const Vector4& p2);

        /**
        * @brief Calculates the dot product between two Vector4s
        */
        static float Dot(const Vector4& v1, const Vector4& v2);

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
        static Vector4 Hermite(const Vector4& p1, const Vector4& t1, const Vector4& p2, const Vector4& t2, float w);

        /**
        * @brief Performs a linear interpolation between two points
        *
        * @param p1 The first point
        * @param p2 The second point
        * @param t The interpolation weight
        * @return The calculated point on the line between p1 and p2
        */
        static Vector4 Lerp(const Vector4& p1, const Vector4& p2, float t);

        /**
        * @brief Finds the maximum of each component between two Vector4s
        *
        * @return The Vector4 containing the maximum X coordinate and the maximum Y coordinate
        */
        static Vector4 Max(const Vector4& v1, const Vector4& v2);

        /**
        * @brief Finds the minimum of each component between two Vector4s
        *
        * @return The Vector4 containing the minimum X coordinate and the minimum Y coordinate
        */
        static Vector4 Min(const Vector4& v1, const Vector4& v2);

        /**
        * @brief Normalizes a given vector
        *
        * @return The normalized vector
        */
        static Vector4 Normalize(Vector4 v);

        /**
        * @brief Performs a Cubic Hermite interpolation between two Vector4s
        *
        * @param v1 The start (t = 0) Vector4
        * @param v2 The end (t = 1) Vector4
        * @param t The interpolation weight
        * @return The calculated Vector4 between v1 and v2
        */
        static Vector4 SmoothStep(const Vector4& v1, const Vector4& v2, float t);

        /**
        * @brief Transforms a Vector4 by a matrix
        *
        * @param v The vector to transform
        * @param m The transformation matrix
        * @return The transformed Vector4
        */
        static Vector4 Transform(const Vector4& v, const Matrix& m);

        /**
        * @brief Rotates a Vector4 by a quaternion
        *
        * @param v The Vector4 to rotate
        * @param q The quaternion to apply
        * @return The rotated Vector4
        */
        static Vector4 Transform(const Vector4& v, const Quaternion& q);

        /**
        * @brief Transforms a list of Vector4s by a matrix and stores them in another list
        *
        * @param source The list of Vector4s to transform
        * @param sourceIdx The index of the first Vector4 in source to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector4s into
        * @param destIdx The index of the first Vector4 in dest to replace
        * @param count The number of Vector4s to transform
        */
        static void Transform(const std::vector<Vector4>& source, ui32 sourceIdx, const Matrix& mat, std::vector<Vector4>& dest, ui32 destIdx, ui32 count);

        /**
        * @brief Rotates a list of Vector4s by a quaternion and stores them in another list
        *
        * @param source The list of Vector4s to rotate
        * @param sourceIdx The index of the first Vector4 in source to rotate
        * @param q The quaternion to apply
        * @param dest The list to insert the rotated Vector4s into
        * @param destIdx The index of the first Vector4 in dest to replace
        * @param count The number of Vector4s to transform
        */
        static void Transform(const std::vector<Vector4>& source, ui32 sourceIdx, const Quaternion& q, std::vector<Vector4>& dest, ui32 destIdx, ui32 count);

        /**
        * @brief Transforms all of the Vector4s in a list by a matrix
        *
        * @param source The list of Vector4s to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector4s into
        */
        static void Transform(const std::vector<Vector4>& source, const Matrix& mat, std::vector<Vector4>& dest);

        /**
        * @brief Rotates all of the Vector4s in a list by a quaternion and stores them in another list
        *
        * @param source The list of Vector4s to rotate
        * @param q The quaternion to apply
        * @param dest The list to insert the rotated Vector4s into
        */
        static void Transform(const std::vector<Vector4>& source, const Quaternion& q, std::vector<Vector4>& dest);

        /**
        * @brief Transforms a normal vector by a matrix without applying translation
        *
        * @param normal The Vector4 to transform
        * @param mat The transformation matrix
        * @return The transformed normal
        */
        static Vector4 TransformNormal(const Vector4& normal, const Matrix& mat);

        /**
        * @brief Transforms a list of normals by a matrix without applying translation
        *
        * @param source The list of Vector4s to transform
        * @param sourceIdx The index of the first Vector4 to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector4s into
        * @param destIdx The index of the first Vector4 in dest to replace
        * @param count The number of Vector4s to transform
        */
        static void TransformNormal(const std::vector<Vector4>& source, ui32 sourceIdx, const Matrix& mat, std::vector<Vector4>& dest, ui32 destIdx, ui32 count);

        /**
        * @brief Transforms all of the normals in a list by a matrix
        *
        * @param source The list of Vector4s to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector4s into
        */
        static void TransformNormal(const std::vector<Vector4>& source, const Matrix& mat, std::vector<Vector4>& dest);

        Vector4& operator+=(const Vector4&);
        Vector4& operator-=(const Vector4&);
        Vector4& operator*=(const Vector4&);
        Vector4& operator*=(float);
        Vector4& operator/=(const Vector4&);
        Vector4& operator/=(float);

    };

    Vector4 operator+(Vector4, const Vector4&);
    Vector4 operator-(Vector4, const Vector4&);
    Vector4 operator*(Vector4, const Vector4&);
    Vector4 operator*(float, Vector4);
    Vector4 operator*(Vector4, float);
    Vector4 operator/(Vector4, const Vector4&);
    Vector4 operator/(Vector4, float);
    Vector4 operator-(Vector4);

    bool operator==(const Vector4&, const Vector4&);
    bool operator!=(const Vector4&, const Vector4&);
}


#endif