#pragma once

#include "VrmlTypes.h"
#include "../odbdesign_export.h"
#include <filesystem>
#include <string>
#include <vector>

namespace Odb::Lib::Converter
{
    struct Triangle; // forward from StepWriter.h

    // Converts a VRML 2.0 (.wrl) file to a STEP AP214 (.stp / .step) file.
    //
    // Supported VRML nodes:
    //   Transform, Group, Shape, Appearance, Material, IndexedFaceSet, Coordinate, Normal
    //
    // The converter:
    //  1. Parses the VRML scene.
    //  2. Recursively flattens Transform hierarchies, accumulating the world-space
    //     transformation matrix (translation, rotation-axis/angle, scale).
    //  3. Triangulates non-triangular polygon faces using fan triangulation.
    //  4. Writes a STEP AP214 FACETED_BREP file.
    //
    // Usage:
    //   VrmlToStepConverter conv;
    //   bool ok = conv.Convert("component.wrl", "component.stp");
    //   if (!ok) std::cerr << conv.GetErrorMessage();
    class ODBDESIGN_EXPORT VrmlToStepConverter
    {
    public:
        VrmlToStepConverter();

        // Convert inputVrml -> outputStep.  Returns false on error.
        bool Convert(const std::filesystem::path& inputVrml,
                     const std::filesystem::path& outputStep);

        // Convert from an in-memory VRML string to a STEP string.
        // partName is embedded in the STEP file header.
        std::string ConvertString(const std::string& vrmlText,
                                  const std::string& partName = "Part");

        const std::string& GetErrorMessage() const;

    private:
        // Flatten the scene into a list of world-space triangles.
        void flattenScene(const VrmlScene& scene,
                          std::vector<Triangle>& triangles) const;

        void flattenTransform(const VrmlTransform& xf,
                              const Matrix4& parentMatrix,
                              std::vector<Triangle>& triangles) const;

        void flattenShape(const VrmlShape& shape,
                          const Matrix4& worldMatrix,
                          std::vector<Triangle>& triangles) const;

        // Build the 4x4 matrix for a VrmlTransform node.
        Matrix4 buildMatrix(const VrmlTransform& xf) const;

        // Triangulate a face given as a sequence of indices.
        // Appends triangles to output.  Uses fan triangulation.
        void triangulateFace(const std::vector<int>& faceIndices,
                             const std::vector<Vec3>& transformedCoords,
                             const Material& mat,
                             std::vector<Triangle>& output) const;

        std::string m_error;
    };

} // namespace Odb::Lib::Converter
