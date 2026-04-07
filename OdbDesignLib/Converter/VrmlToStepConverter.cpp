#include "VrmlToStepConverter.h"
#include "VrmlParser.h"
#include "StepWriter.h"
#include <cmath>

namespace Odb::Lib::Converter
{

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

VrmlToStepConverter::VrmlToStepConverter() = default;

const std::string& VrmlToStepConverter::GetErrorMessage() const { return m_error; }

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool VrmlToStepConverter::Convert(const std::filesystem::path& inputVrml,
                                   const std::filesystem::path& outputStep)
{
    VrmlParser parser;
    if (!parser.ParseFile(inputVrml))
    {
        m_error = "VRML parse error: " + parser.GetErrorMessage();
        return false;
    }

    std::vector<Triangle> triangles;
    flattenScene(parser.GetScene(), triangles);

    if (triangles.empty())
    {
        m_error = "No geometry found in VRML file";
        return false;
    }

    StepWriter writer;
    std::string partName = inputVrml.stem().string();
    if (!writer.Write(outputStep, triangles, partName))
    {
        m_error = "STEP write error: " + writer.GetErrorMessage();
        return false;
    }

    return true;
}

std::string VrmlToStepConverter::ConvertString(const std::string& vrmlText,
                                                const std::string& partName)
{
    VrmlParser parser;
    if (!parser.ParseString(vrmlText))
    {
        m_error = "VRML parse error: " + parser.GetErrorMessage();
        return {};
    }

    std::vector<Triangle> triangles;
    flattenScene(parser.GetScene(), triangles);

    if (triangles.empty())
    {
        m_error = "No geometry found in VRML text";
        return {};
    }

    StepWriter writer;
    std::string result = writer.WriteToString(triangles, partName);
    if (result.empty())
    {
        m_error = "STEP write error: " + writer.GetErrorMessage();
    }
    return result;
}

// ---------------------------------------------------------------------------
// Scene flattening
// ---------------------------------------------------------------------------

void VrmlToStepConverter::flattenScene(const VrmlScene& scene,
                                        std::vector<Triangle>& triangles) const
{
    Matrix4 identity;
    // Root-level shapes (no transform)
    for (const auto& shape : scene.shapes)
        flattenShape(shape, identity, triangles);

    // Root-level transforms
    for (const auto& xf : scene.transforms)
        flattenTransform(*xf, identity, triangles);
}

void VrmlToStepConverter::flattenTransform(const VrmlTransform& xf,
                                            const Matrix4& parentMatrix,
                                            std::vector<Triangle>& triangles) const
{
    Matrix4 local = buildMatrix(xf);
    Matrix4 world = Matrix4::multiply(parentMatrix, local);

    for (const auto& shape : xf.shapes)
        flattenShape(shape, world, triangles);

    for (const auto& child : xf.children)
        flattenTransform(*child, world, triangles);
}

void VrmlToStepConverter::flattenShape(const VrmlShape& shape,
                                        const Matrix4& worldMatrix,
                                        std::vector<Triangle>& triangles) const
{
    if (!shape.hasGeometry) return;

    const IndexedFaceSet& ifs = shape.geometry;
    if (ifs.coords.empty() || ifs.coordIndex.empty()) return;

    // Transform all coordinates to world space.
    std::vector<Vec3> worldCoords;
    worldCoords.reserve(ifs.coords.size());
    for (const auto& c : ifs.coords)
        worldCoords.push_back(worldMatrix.transformPoint(c));

    // Split coordIndex into individual faces (separated by -1).
    std::vector<int> face;
    for (int idx : ifs.coordIndex)
    {
        if (idx == -1)
        {
            if (face.size() >= 3)
                triangulateFace(face, worldCoords, shape.material, triangles);
            face.clear();
        }
        else
        {
            face.push_back(idx);
        }
    }
    // Handle last face if not terminated by -1
    if (face.size() >= 3)
        triangulateFace(face, worldCoords, shape.material, triangles);
}

void VrmlToStepConverter::triangulateFace(
        const std::vector<int>& faceIndices,
        const std::vector<Vec3>& worldCoords,
        const Material& mat,
        std::vector<Triangle>& output) const
{
    if (faceIndices.size() < 3) return;

    // Validate all indices are in range
    for (int idx : faceIndices)
    {
        if (idx < 0 || static_cast<size_t>(idx) >= worldCoords.size())
            return;
    }

    // Fan triangulation from vertex 0
    const Vec3& v0 = worldCoords[static_cast<size_t>(faceIndices[0])];
    for (size_t i = 1; i + 1 < faceIndices.size(); ++i)
    {
        const Vec3& v1 = worldCoords[static_cast<size_t>(faceIndices[i])];
        const Vec3& v2 = worldCoords[static_cast<size_t>(faceIndices[i + 1])];

        // Skip degenerate triangles (area ~= 0)
        auto cross = [](const Vec3& a, const Vec3& b, const Vec3& c) {
            Vec3 ab = {b.x - a.x, b.y - a.y, b.z - a.z};
            Vec3 ac = {c.x - a.x, c.y - a.y, c.z - a.z};
            double cx = ab.y * ac.z - ab.z * ac.y;
            double cy = ab.z * ac.x - ab.x * ac.z;
            double cz = ab.x * ac.y - ab.y * ac.x;
            return std::sqrt(cx*cx + cy*cy + cz*cz);
        };
        if (cross(v0, v1, v2) < 1e-15) continue;

        Triangle tri;
        tri.v0 = v0;
        tri.v1 = v1;
        tri.v2 = v2;
        tri.material = mat;
        output.push_back(tri);
    }
}

// ---------------------------------------------------------------------------
// Transform matrix construction
// ---------------------------------------------------------------------------

// Build the combined 4x4 transform matrix for a VRML Transform node.
// VRML 2.0 spec order:
//   T(translation) * T(center) * R(rotation) * R(scaleOrientation)
//   * S(scale) * R(-scaleOrientation) * T(-center)
Matrix4 VrmlToStepConverter::buildMatrix(const VrmlTransform& xf) const
{
    // Rotation matrix from axis-angle (Rodrigues' formula)
    auto makeRotation = [](const Vec3& axis, double angle) -> Matrix4 {
        Matrix4 r;
        double len = std::sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
        if (len < 1e-15) return r; // identity

        double x = axis.x / len;
        double y = axis.y / len;
        double z = axis.z / len;
        double c = std::cos(angle);
        double s = std::sin(angle);
        double t = 1.0 - c;

        r.m[0][0] = t*x*x + c;    r.m[0][1] = t*x*y - s*z; r.m[0][2] = t*x*z + s*y; r.m[0][3] = 0;
        r.m[1][0] = t*x*y + s*z;  r.m[1][1] = t*y*y + c;   r.m[1][2] = t*y*z - s*x; r.m[1][3] = 0;
        r.m[2][0] = t*x*z - s*y;  r.m[2][1] = t*y*z + s*x; r.m[2][2] = t*z*z + c;   r.m[2][3] = 0;
        r.m[3][0] = 0;             r.m[3][1] = 0;            r.m[3][2] = 0;            r.m[3][3] = 1;
        return r;
    };

    auto makeTranslation = [](const Vec3& t) -> Matrix4 {
        Matrix4 m;
        m.m[0][3] = t.x;
        m.m[1][3] = t.y;
        m.m[2][3] = t.z;
        return m;
    };

    auto makeScale = [](const Vec3& s) -> Matrix4 {
        Matrix4 m;
        m.m[0][0] = s.x;
        m.m[1][1] = s.y;
        m.m[2][2] = s.z;
        return m;
    };

    auto neg = [](const Vec3& v) -> Vec3 { return {-v.x, -v.y, -v.z}; };

    // T(translation)
    Matrix4 T   = makeTranslation(xf.translation);
    // T(center)
    Matrix4 TC  = makeTranslation(xf.center);
    // R(rotation)
    Matrix4 R   = makeRotation(xf.rotationAxis, xf.rotationAngle);
    // R(scaleOrientation)
    Matrix4 SR  = makeRotation(xf.scaleOrientationAxis, xf.scaleOrientationAngle);
    // S(scale)
    Matrix4 S   = makeScale(xf.scale);
    // R(-scaleOrientation)
    Matrix4 SRi = makeRotation(xf.scaleOrientationAxis, -xf.scaleOrientationAngle);
    // T(-center)
    Matrix4 TCi = makeTranslation(neg(xf.center));

    // Combined: T * TC * R * SR * S * SRi * TCi
    Matrix4 result = T;
    result = Matrix4::multiply(result, TC);
    result = Matrix4::multiply(result, R);
    result = Matrix4::multiply(result, SR);
    result = Matrix4::multiply(result, S);
    result = Matrix4::multiply(result, SRi);
    result = Matrix4::multiply(result, TCi);

    return result;
}

} // namespace Odb::Lib::Converter
