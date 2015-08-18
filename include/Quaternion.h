#ifndef _QUATERNION_H
#define _QUATERNION_H

namespace YAX
{
    struct Vector3;
    struct Matrix;

    /** @brief A mathematical tool used to represent rotations */
    struct Quaternion
    {	
        float X, Y, Z, W;
        static const Quaternion Identity;

        /**
        * @brief Creates a quaternion from precalculated components
        *
        * @param x The x vector component; x = rotationAxis_x * sin(rotationAngle / 2)
        * @param y The y vector component; y = rotationAxis_y * sin(rotationAngle / 2)
        * @param z The z vector component; z = rotationAxis_z * sin(rotationAngle / 2)
        * @param w The real component; w = cos(rotationAngle / 2)
        */
        Quaternion(float x, float y, float z, float w);

        /**
        * @brief Creates a quaternion from precalculated components
        *
        * @param xyz The vector component of the quaternion; xyz = roationAxis * sin(rotationAngle / 2)
        * @param w The real component of the quaternion; w = cos(rotationAngle / 2)
        */
        Quaternion(const Vector3& xyz, float w);

        /**
        * @brief Find the conjugate of the quaternion in-place
        */
        void Conjugate();

        /**
        * @brief Finds the dot product between two quaternions
        *
        * @param q The quaternion to dot this one with
        * @return The dot product of the two quaternions
        */
        float Dot(const Quaternion& q) const;

        /**
        * @brief Gets the length of the quaternion
        * 
        * @return The length of the quaternion
        */
        float Length() const;

        /**
        * @brief Get the squared length of the quaternion
        *
        * @return The squared length of the quaternion
        */
        float LengthSquared() const;

        /**
        * @brief Normalizes the quaternion
        */
        void Normalize();
        
        /**
        * @brief Combines two quaternions into a single rotation
        *
        * @param first The first rotation
        * @param second The second rotation
        * @return The quaternion representing the first rotation followed by the second rotation
        */
        static Quaternion Concatenate(const Quaternion& first, const Quaternion& second);

        /**
        * @brief Find the conjugate of the given quaternion
        * 
        * @param q The quaternion to invert
        * @return The conjugate quaternion of q
        */
        static Quaternion Conjugate(Quaternion q);

        /**
        * @brief Creates a quaternion representing a rotation of angle radians about a given axis
        *
        * @param axis The axis of rotation
        * @param angle The angle of rotation in radians
        * @return The calculated quaternion
        */
        static Quaternion CreateFromAxisAngle(const Vector3& axis, float angle);
        
        /**
        * @brief Converts a rotation matrix into its equivalent quaternion representation
        *
        * @param mat The rotation matrix to convert
        * @return The quaternion representing the rotation 
        */
        static Quaternion CreateFromRotationMatrix(const Matrix& mat);

        /**
        * @brief Creates a quaternion from yaw (Y), pitch (X), and roll (Z), angles
        *
        * @param yaw The yaw angle in radians
        * @param pitch The pitch angle in radians
        * @param roll The roll angle in radians
        * @return The quaternion representing the rotation of yaw, followed by pitch, followed by roll
        */
        static Quaternion CreateFromYawPitchRoll(float yaw, float pitch, float roll);

        /**
        * @brief Find the dot product between two quaternions
        *
        * @param q1 The first quaternion
        * @param q2 The second quaternion
        * @return The dot product between q1 and q2
        */
        static float Dot(const Quaternion& q1, const Quaternion& q2);

        /**
        * @brief Find the inverse of a given quaternion
        *
        * @param q The quaternion to invert
        * @return The inverse quaternion of q
        */
        static Quaternion Inverse(Quaternion q);

        /**
        * @brief Performs a linear interpolation between two quaternions
        *
        * @param from The start (t = 0) quaternion
        * @param to The end (t = 1) quaternion
        * @param t The interpolation weight
        * @return The interpolated quaternion
        */
        static Quaternion Lerp(const Quaternion& from, const Quaternion& to, float t);

        /**
        * @brief Finds the normal of a given quaternion
        * 
        * @param q The quaternion to normalize
        * @return The normalized quaternion
        */
        static Quaternion Normalize(Quaternion q);

        /**
        * @brief Performs a spherical linear interpolation between two quaternions 
        *
        * @param from The start (t = 0) quaternion
        * @param to The end (t = 1) quaternion
        * @param t The interpolation weight
        */
        static Quaternion Slerp(const Quaternion& from, const Quaternion& to, float t);

        Quaternion& operator+=(const Quaternion&);
        Quaternion& operator-=(const Quaternion&);
        Quaternion& operator*=(const Quaternion&);
        Quaternion& operator*=(float);
        Quaternion& operator/=(const Quaternion&);
        Quaternion& operator/=(float);

    };

    Quaternion operator+(Quaternion, const Quaternion&);
    Quaternion operator-(Quaternion, const Quaternion&);
    Quaternion operator*(Quaternion, const Quaternion&);
    Quaternion operator*(Quaternion, float);
    Quaternion operator*(float, const Quaternion&);
    Quaternion operator/(Quaternion, const Quaternion&);
    Quaternion operator/(Quaternion, float);
    Quaternion operator-(Quaternion);

    bool operator==(const Quaternion&, const Quaternion&);
    bool operator!=(const Quaternion&, const Quaternion&);
}


#endif