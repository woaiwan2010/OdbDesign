#pragma once

#include <vector>
#include <string>
#include <memory>
#include <map>

namespace Odb::Lib::Converter
{
    struct Vec3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Color
    {
        double r = 0.8;
        double g = 0.8;
        double b = 0.8;
    };

    struct Material
    {
        Color diffuseColor{0.8, 0.8, 0.8};
        Color specularColor{0.0, 0.0, 0.0};
        Color emissiveColor{0.0, 0.0, 0.0};
        double ambientIntensity = 0.2;
        double shininess = 0.2;
        double transparency = 0.0;
    };

    // Represents a VRML IndexedFaceSet geometry node.
    struct IndexedFaceSet
    {
        std::vector<Vec3> coords;
        std::vector<int> coordIndex;   // face vertex indices; -1 separates faces
        std::vector<Vec3> normals;
        std::vector<int> normalIndex;
        bool normalPerVertex = true;
        bool ccw = true;
        bool solid = true;
        bool convex = true;
    };

    // A VRML Shape node: one piece of geometry with its material.
    struct VrmlShape
    {
        Material material;
        bool hasMaterial = false;
        IndexedFaceSet geometry;
        bool hasGeometry = false;
    };

    // A 4x4 row-major transformation matrix.
    struct Matrix4
    {
        double m[4][4];

        Matrix4()
        {
            // identity
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    m[i][j] = (i == j) ? 1.0 : 0.0;
        }

        Vec3 transformPoint(const Vec3& p) const
        {
            double x = m[0][0]*p.x + m[0][1]*p.y + m[0][2]*p.z + m[0][3];
            double y = m[1][0]*p.x + m[1][1]*p.y + m[1][2]*p.z + m[1][3];
            double z = m[2][0]*p.x + m[2][1]*p.y + m[2][2]*p.z + m[2][3];
            return {x, y, z};
        }

        Vec3 transformDirection(const Vec3& d) const
        {
            double x = m[0][0]*d.x + m[0][1]*d.y + m[0][2]*d.z;
            double y = m[1][0]*d.x + m[1][1]*d.y + m[1][2]*d.z;
            double z = m[2][0]*d.x + m[2][1]*d.y + m[2][2]*d.z;
            return {x, y, z};
        }

        static Matrix4 multiply(const Matrix4& a, const Matrix4& b)
        {
            Matrix4 result;
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                {
                    result.m[i][j] = 0.0;
                    for (int k = 0; k < 4; ++k)
                        result.m[i][j] += a.m[i][k] * b.m[k][j];
                }
            return result;
        }
    };

    // A VRML Transform or Group node.
    struct VrmlTransform
    {
        Vec3 translation{0.0, 0.0, 0.0};
        Vec3 rotationAxis{0.0, 0.0, 1.0};
        double rotationAngle = 0.0;
        Vec3 scale{1.0, 1.0, 1.0};
        Vec3 center{0.0, 0.0, 0.0};
        Vec3 scaleOrientationAxis{0.0, 0.0, 1.0};
        double scaleOrientationAngle = 0.0;

        std::vector<VrmlShape> shapes;
        std::vector<std::shared_ptr<VrmlTransform>> children;
    };

    // Top-level scene holding root shapes and transform nodes.
    struct VrmlScene
    {
        std::vector<VrmlShape> shapes;
        std::vector<std::shared_ptr<VrmlTransform>> transforms;
    };

} // namespace Odb::Lib::Converter
