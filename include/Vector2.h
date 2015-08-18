#ifndef _VEC2_H
#define _VEC2_H

#include <vector>
#include "Utils.h"

namespace YAX
{
    struct Matrix;
    struct Quaternion;

    struct Vector2
    {
        static const Vector2 One, UnitX, UnitY, Zero;

        float X, Y;
        
        Vector2(float val);
        Vector2(float x, float y);

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
        * @brief Performs a barycentric interpolation over a triangle defined by three Vector2s
        *
        * @param p1 The first point of the triangle
        * @param p2 The second point of the triangle
        * @param p3 The third point of the triangle
        * @param b2 The weight factor of the second point
        * @param b3 The weight factor of the third point
        * @return The calculated barycentric point
        */
        static Vector2 Barycentric(const Vector2& p1, const Vector2& p2, const Vector2& p3, float b2, float b3);
        
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
        static Vector2 CatmullRom(const Vector2& p1, const Vector2& p2, const Vector2& p3, const Vector2& p4, float t);
        
        /**
        * @brief Clamps a Vector2's components between a min and max value
        *
        * @param point The Vector2 to clamp
        * @param min The minimum values for each coordinate
        * @param max The maximum values for each coordinate
        * @return The clamped Vector2
        */
        static Vector2 Clamp(const Vector2& point, const Vector2& min, const Vector2& max);
        
        /**
        * @brief Calculates the signed distance between two points represented by Vector2s 
        */
        static float Distance(const Vector2& p1, const Vector2& p2);

        /**
        * @brief Calculates the squared distance between two points represented by Vector2s
        */
        static float DistanceSquared(const Vector2& p1, const Vector2& p2);

        /**
        * @brief Calculates the dot product between two Vector2s
        */
        static float Dot(const Vector2& v1, const Vector2& v2);

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
        static Vector2 Hermite(const Vector2& p1, const Vector2& t1, const Vector2& p2, const Vector2& t2, float w);
        
        /**
        * @brief Performs a linear interpolation between two points
        * 
        * @param p1 The first point
        * @param p2 The second point
        * @param t The interpolation weight
        * @return The calculated point on the line between p1 and p2
        */
        static Vector2 Lerp(const Vector2& p1, const Vector2& p2, float t);
        
        /**
        * @brief Finds the maximum of each component between two Vector2s
        * 
        * @return The Vector2 containing the maximum X coordinate and the maximum Y coordinate
        */
        static Vector2 Max(const Vector2& v1, const Vector2& v2);

        /**
        * @brief Finds the minimum of each component between two Vector2s
        *
        * @return The Vector2 containing the minimum X coordinate and the minimum Y coordinate
        */
        static Vector2 Min(const Vector2& v1, const Vector2& v2);

        /**
        * @brief Normalizes a given vector
        *
        * @return The normalized vector
        */
        static Vector2 Normalize(Vector2 vec);

        /**
        * @brief Reflects a Vector2 across a given normal; 
        *
        * @param vec The vector to reflect
        * @param normal The normal to reflect across
        * @return The reflected Vector2
        */
        static Vector2 Reflect(const Vector2& vec, const Vector2& normal);

        /**
        * @brief Performs a Cubic Hermite interpolation between two Vector2s
        *
        * @param v1 The start (t = 0) Vector2
        * @param v2 The end (t = 1) Vector2
        * @param t The interpolation weight
        * @return The calculated Vector2 between v1 and v2
        */
        static Vector2 SmoothStep(const Vector2& v1, const Vector2& v2, float t);

        /**
        * @brief Transforms a Vector2 by a matrix
        * 
        * @param v The vector to transform
        * @param m The transformation matrix
        * @return The transformed Vector2 
        */
        static Vector2 Transform(const Vector2& v, const Matrix& m);

        /**
        * @brief Rotates a Vector2 by a quaternion
        *
        * @param v The Vector2 to rotate
        * @param q The quaternion to apply
        * @return The rotated Vector2
        */
        static Vector2 Transform(const Vector2& v, const Quaternion& q);

        /**
        * @brief Transforms a list of Vector2s by a matrix and stores them in another list
        *
        * @param source The list of Vector2s to transform
        * @param sourceIdx The index of the first Vector2 in source to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector2s into
        * @param destIdx The index of the first Vector2 in dest to replace
        * @param count The number of Vector2s to transform
        */
        static void Transform(const std::vector<Vector2>& source, ui32 sourceIdx, const Matrix& mat, std::vector<Vector2>& dest, ui32 destIdx, ui32 count);
        
        /**
        * @brief Rotates a list of Vector2s by a quaternion and stores them in another list
        *
        * @param source The list of Vector2s to rotate
        * @param sourceIdx The index of the first Vector2 in source to rotate
        * @param q The quaternion to apply
        * @param dest The list to insert the rotated Vector2s into
        * @param destIdx The index of the first Vector2 in dest to replace
        * @param count The number of Vector2s to transform
        */
        static void Transform(const std::vector<Vector2>& source, ui32 sourceIdx, const Quaternion& q, std::vector<Vector2>& dest, ui32 destIdx, ui32 count);

        /**
        * @brief Transforms all of the Vector2s in a list by a matrix
        *
        * @param source The list of Vector2s to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector2s into
        */
        static void Transform(const std::vector<Vector2>& source, const Matrix& mat, std::vector<Vector2>& dest);
        
        /**
        * @brief Rotates all of the Vector2s in a list by a quaternion and stores them in another list
        *
        * @param source The list of Vector2s to rotate
        * @param q The quaternion to apply
        * @param dest The list to insert the rotated Vector2s into
        */
        static void Transform(const std::vector<Vector2>& source, const Quaternion& q, std::vector<Vector2>& dest);

        /**
        * @brief Transforms a normal vector by a matrix without applying translation
        *
        * @param normal The Vector2 to transform
        * @param mat The transformation matrix
        * @return The transformed normal
        */
        static Vector2 TransformNormal(const Vector2& normal, const Matrix& mat);
        
        /**
        * @brief Transforms a list of normals by a matrix without applying translation
        *
        * @param source The list of Vector2s to transform
        * @param sourceIdx The index of the first Vector2 to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector2s into
        * @param destIdx The index of the first Vector2 in dest to replace
        * @param count The number of Vector2s to transform
        */
        static void TransformNormal(const std::vector<Vector2>& source, ui32 sourceIdx, const Matrix& mat, std::vector<Vector2>& dest, ui32 destIdx, ui32 count);
        
        /**
        * @brief Transforms all of the normals in a list by a matrix
        * 
        * @param source The list of Vector2s to transform
        * @param mat The transformation matrix
        * @param dest The list to insert the transformed Vector2s into
        */
        static void TransformNormal(const std::vector<Vector2>& source, const Matrix& mat, std::vector<Vector2>& dest);
    
        Vector2& operator+=(const Vector2&); 
        Vector2& operator-=(const Vector2&);
        Vector2& operator*=(const Vector2&);
        Vector2& operator*=(float);
        Vector2& operator/=(const Vector2&);
        Vector2& operator/=(float);
    };

    Vector2 operator+(const Vector2&, const Vector2&);
    Vector2 operator-(const Vector2&, const Vector2&);
    Vector2 operator*(const Vector2&, const Vector2&);
    Vector2 operator*(const Vector2&, float);
    Vector2 operator*(float, const Vector2&);
    Vector2 operator/(const Vector2&, const Vector2&);
    Vector2 operator/(const Vector2&, float);
    Vector2 operator-(const Vector2&);

    bool operator==(const Vector2&, const Vector2&);
    bool operator!=(const Vector2&, const Vector2&);
}


#endif