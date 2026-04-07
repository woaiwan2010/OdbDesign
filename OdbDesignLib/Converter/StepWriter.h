#pragma once

#include "VrmlTypes.h"
#include "../odbdesign_export.h"
#include <filesystem>
#include <string>
#include <vector>

namespace Odb::Lib::Converter
{
    // A triangle ready for STEP output (world-space coordinates).
    struct Triangle
    {
        Vec3 v0, v1, v2;
        Material material;
    };

    // Writes STEP AP214 files from a list of triangles.
    // The output uses FACETED_BREP with ADVANCED_FACE entities,
    // which is the most widely supported STEP format for faceted geometry
    // (supported by Altium, KiCad, SolidWorks, CATIA, Fusion 360, etc.).
    class ODBDESIGN_EXPORT StepWriter
    {
    public:
        StepWriter();

        // Write triangles to a STEP AP214 file.
        // partName is embedded in the FILE_NAME header.
        bool Write(const std::filesystem::path& outputPath,
                   const std::vector<Triangle>& triangles,
                   const std::string& partName = "Part");

        // Write to a string (useful for testing).
        std::string WriteToString(const std::vector<Triangle>& triangles,
                                  const std::string& partName = "Part");

        const std::string& GetErrorMessage() const;

    private:
        std::string m_error;

        std::string generate(const std::vector<Triangle>& triangles,
                             const std::string& partName);
    };

} // namespace Odb::Lib::Converter
