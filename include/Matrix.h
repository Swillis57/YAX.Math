#ifndef _MATRIX_H
#define _MATRIX_H

namespace YAX
{
    struct Vector3;
    struct Quaternion;
    struct Plane;

    /**
    * @brief Represents a row-major up-to-4x4 matrix
    */
    struct Matrix
    {

        float M11, M12, M13, M14,
              M21, M22, M23, M24,
              M31, M32, M33, M34,
              M41, M42, M43, M44;
        
        /** @brief The identity matrix */
        static const Matrix Identity;
        
        /** @brief The Catmull-Rom interpolation matrix */
        static const Matrix CatmullRomMat;

        Matrix(
            float M11, float M12, float M13, float M14,
            float M21, float M22, float M23, float M24,
            float M31, float M32, float M33, float M34,
            float M41, float M42, float M43, float M44
        );

        Vector3 Backward() const;
        void Backward(const Vector3&);

        Vector3 Down() const;
        void Down(const Vector3&);

        Vector3 Forward() const;
        void Forward(const Vector3&);

        Vector3 Left() const;
        void Left(const Vector3&);

        Vector3 Right() const;
        void Right(const Vector3&);

        Vector3 Translation() const;
        void Translation(const Vector3&);

        Vector3 Up() const;
        void Up(const Vector3&);


        /**
        * @brief Decomposes the matrix into its scale, rotation, and translation components
        *
        * @param scale Output parameter for the scale component
        * @param rot Output parameter for the rotation component
        * @param trans Output parameter for the translation component
        * @return true if matrix can be successfully decomposed, false otherwise
        */
        bool Decompose(Vector3& scale, Quaternion& rot, Vector3& trans) const;

        /**
        * @brief Calculates the determinant of the matrix
        */
        float Determinant() const;

        /**
        * @brief Creates a spherical (rotates around all axes) billboard matrix 
        *
        * @param objectPos The position of the billboard
        * @param cameraPos The position of the camera
        * @param cameraUp The up vector of the camera
        * @param cameraForward Optional parameter used when objectPos and cameraPos are too close and would produce odd results; Pass nullptr if not needed
        * @return The billboard's world matrix
        */
        static Matrix CreateBillboard(const Vector3& objectPos, const Vector3& cameraPos, const Vector3& cameraUp, const Vector3* cameraForward);
        
        /**
        * @brief Creates a cylindrical (rotates around provided axis) billboard matrix
        *
        * @param objectPos The position of the billboard
        * @param cameraPos The position of the camera
        * @param rotAxis The axis the billboard will rotate around
        * @param cameraForward Optional parameter used when objectPos and cameraPos are too close and would produce odd results; Pass nullptr if not needed
        * @param objectForward Optional parameter used when the camera is looking down the rotation axis
        * @return The billboard's world matrix
        */
        static Matrix CreateConstrainedBillboard(const Vector3& objectPos, const Vector3& cameraPos, const Vector3& rotAxis, const Vector3* cameraForward, const Vector3* objectForward);
        
        /**
        * @brief Creates a rotation matrix that rotates about a given axis
        *
        * @param axis The axis to rotate around
        * @param angle The angle in radians 
        * @return The rotation matrix
        */
        static Matrix CreateFromAxisAngle(const Vector3& axis, float angle);
        
        /**
        * @brief Converts a given quaternion into a rotation matrix
        *
        * @param q The quaternion to convert
        * @return The rotation matrix
        */
        static Matrix CreateFromQuaternion(const Quaternion& q);
        
        /**
        * @brief Creates a rotation matrix from yaw (Y), pitch (X), and roll (Z) angles
        *
        * @param y Yaw angle in radians
        * @param p Pitch angle in radians
        * @param r Roll angle in radians
        * @return The rotation matrix
        */
        static Matrix CreateFromYawPitchRoll(float y, float p, float r);
        
        /**
        * @brief Creates a view matrix 
        * 
        * @param cameraPos The camera's position
        * @param cameraTarg The position of the camera's target
        * @param cameraUp The camera's up vector
        * @return The view matrix
        */
        static Matrix CreateLookAt(const Vector3& cameraPos, const Vector3& cameraTarg, const Vector3& cameraUp);
        
        /**
        * @brief Creates a centered orthographic projection matrix
        *
        * @param width The width of the view box
        * @param height The height of the view box
        * @param zNear The minimum z-value of the view box
        * @param zFar The maximum z-value of the view box
        * @return The orthographic projection matrix
        */
        static Matrix CreateOrthographic(float width, float height, float zNear, float zFar);
        
        /**
        * @brief Creates an off-center orthographic projection matrix
        *
        * @param left The minimum x-value of the view box
        * @param right The maximum x-value of the view box
        * @param bottom The minimum y-value of the view box
        * @param top The maximum y-value of the view box
        * @param zNear The minimum z-value of the view box
        * @param zFar The maximum z-value of the view box
        * @return The off-center orthographic projection matrix
        */
        static Matrix CreateOrthographicOffCenter(float left, float right, float bottom, float top, float zNear, float zFar);
        
        /**
        * @brief Creates a centered perspective projection matrix
        *
        * @param width The width of the near view plane
        * @param height The height of the near view plane
        * @param zNear The z-value at the near view plane
        * @param zFar The z-value at the far view plane
        * @return The projection matrix
        */
        static Matrix CreatePerspective(float width, float height, float zNear, float zFar);

        /**
        * @brief Creates a centered perspective projection matrix from a field-of-view angle and aspect ratio
        *
        * @param fieldOfView The horizontal field of view of the view frustum
        * @param aspectRatio The ratio of the near view plane's height to its width
        * @param zNear The z-value at the near view plane
        * @param zFar The z-value at the far view plane
        * @return The projection matrix
        */
        static Matrix CreatePerspectiveFieldOfView(float fieldOfView, float aspectRatio, float zNear, float zFar);

        /**
        * @brief Creates an off-center perspective projection matrix
        *
        * @param left The minimum x-value of the near view plane
        * @param right The maximum x-value of the near view plane
        * @param bottom The minimum y-value of the near view plane
        * @param top The maximum y-value of the near view plane
        * @param zNear The z-value at the near view plane
        * @param zFar the z-value at the far view plane
        * @return The off-center projection matrix
        */
        static Matrix CreatePerspectiveOffCenter(float left, float right, float bottom, float top, float zNear, float zFar);

        /**
        * @brief Creates a reflection matrix that reflects about the normal of a plane
        *
        * @param normal The normal of the reflection plane
        * @param dist The distance from the origin to the plane along the normal
        * @return The reflection matrix
        */
        static Matrix CreateReflection(const Vector3& normal, float dist);
        
        /**
        * @brief Creates a matrix that rotates around the x-axis
        *
        * @param angle The rotation angle in radians
        * @return The rotation matrix
        */
        static Matrix CreateRotationX(float angle);

        /**
        * @brief Creates a matrix that rotates around the y-axis
        *
        * @param angle The rotation angle in radians
        * @return The rotation matrix
        */
        static Matrix CreateRotationY(float angle);
        
        /**
        * @brief Creates a matrix that rotates around the z-axis
        *
        * @param angle The rotation angle in radians
        * @return The rotation matrix
        */
        static Matrix CreateRotationZ(float angle); 

        /**
        * @brief Creates a uniform scaling matrix
        *
        * @param scale The scale factor
        * @return The uniform scaling matrix
        */
        static Matrix CreateScale(float scale);

        /**
        * @brief Creates a non-uniform scaling matrix
        *
        * @param scaleX The scale factor for the x-axis
        * @param scaleY The scale factor for the y-axis
        * @param scaleZ The scale factor for the z-axis
        * @return The non-uniform scaling matrix
        */
        static Matrix CreateScale(float scaleX, float scaleY, float scaleZ);
        
        /**
        * @brief Creates a scaling matrix from a vector of scale factors
        *
        * @param scaleVec Vector of scale factors
        * @return The scaling matrix
        */
        static Matrix CreateScale(const Vector3& scaleVec);

        /**
        * @brief Creates a matrix that projects onto the specified plane
        *
        * @param lightDir The direction from which the light is coming
        * @param planeNormal The normal of the shadow plane
        * @param planeDist The distance along planeNormal from the origin to the plane
        * @return The shadow projection matrix
        */
        static Matrix CreateShadow(const Vector3& lightDir, const Vector3& planeNormal, float planeDist);
        
        /**
        * @brief Creates a translation matrix
        *
        * @param xT Translation along the x-axis
        * @param yT Translation along the y-axis
        * @param zT Translation along the z-axis
        * @return The translation matrix
        */
        static Matrix CreateTranslation(float xT, float yT, float zT);
        
        /**
        * @brief Creates a translation matrix from a Vector of distances
        *
        * @param vec The Vector of translation distances
        * @return The translation matrix
        */
        static Matrix CreateTranslation(const Vector3& vec);

        /**
        * @brief Creates a world matrix that projects a point into the world's coordinate system
        *
        * @param pos The position of the world's origin
        * @param fwd The forward direction of the world
        * @param up The up direction of the world
        */
        static Matrix CreateWorld(Vector3 pos, Vector3 fwd, Vector3 up);

#ifdef YAX_GEOMETRY

        /**
        * @brief Creates a reflection matrix that reflects about the given plane
        * 
        * @param The reflection plane
        * @return The reflection matrix
        */
        static Matrix CreateReflection(const Plane& plane);
        
        /**
        * @brief Creates a matrix that projects onto the specified plane
        *
        * @param lightDir The direction from which the light is coming
        * @param plane The plane to project the shadow onto
        * @return The shadow projection matrix
        */
        static Matrix CreateShadow(const Vector3& lightDir, const Plane& plane);
#endif

        /**
        * @brief Finds the inverse of a matrix
        *
        * @param mat The matrix to find the inverse of
        * @return The inverted matrix
        */
        static Matrix Invert(const Matrix& mat);

        /**
        * @brief Performs a component-wise linear interpolation between two matrices
        *
        * @param from The start (t = 0) matrix
        * @param to The end (t = 1) matrix
        * @param t The interpolation weight
        */
        static Matrix Lerp(const Matrix& from, const Matrix& to, float t);
        
        /**
        * Rotates the coordinate system represented by a matrix by a quaternion
        * 
        * @param mat The matrix to transform 
        * @param rotation The rotation to apply
        * @return The transformed matrix
        */
        static Matrix Transform(const Matrix& mat, const Quaternion& rotation);

        /**
        * @brief Transposes a given matrix
        *
        * @param mat The matrix to transpose
        * @return The transposed matrix
        */
        static Matrix Transpose(const Matrix& mat);

        Matrix& operator+=(const Matrix&);
        Matrix& operator-=(const Matrix&);
        Matrix& operator*=(const Matrix&);
        Matrix& operator*=(float);
        Matrix& operator/=(const Matrix&);
        Matrix& operator/=(float);
    };

    Matrix operator+(Matrix, const Matrix&);
    Matrix operator-(Matrix, const Matrix&);
    Matrix operator*(Matrix, const Matrix&);
    Matrix operator*(float, const Matrix&);
    Matrix operator*(Matrix, float);
    Matrix operator/(Matrix, const Matrix&);
    Matrix operator/(Matrix, float);
    Matrix operator-(Matrix);

    bool operator==(const Matrix&, const Matrix&);
    bool operator!=(const Matrix&, const Matrix&);
}


#endif