#pragma once

#include "VrmlTypes.h"
#include "../odbdesign_export.h"
#include <string>
#include <vector>
#include <filesystem>
#include <map>
#include <memory>

namespace Odb::Lib::Converter
{
    // Parses VRML 2.0 (WRL) files into an in-memory VrmlScene.
    // Handles Transform, Group, Shape, Appearance, Material,
    // IndexedFaceSet, and Coordinate nodes.  DEF/USE references
    // are resolved for the supported node types.
    class ODBDESIGN_EXPORT VrmlParser
    {
    public:
        VrmlParser();

        // Parse a file from disk; returns false on failure.
        bool ParseFile(const std::filesystem::path& path);

        // Parse raw VRML text; returns false on failure.
        bool ParseString(const std::string& vrmlText);

        const VrmlScene& GetScene() const;

        const std::string& GetErrorMessage() const;

    private:
        // ---- Tokeniser ----
        void tokenize(const std::string& text);

        bool isSpecial(char c) const;

        // ---- Token access helpers ----
        const std::string& peek(int offset = 0) const;
        const std::string& consume();
        bool atEnd() const;
        bool expect(const std::string& token);

        // ---- Node parsers ----
        // Parse the top-level stream of nodes.
        bool parseScene();

        // Parse a single node (possibly prefixed with DEF name).
        // Returns false on hard error.
        bool parseNode(VrmlScene& scene,
                       const std::shared_ptr<VrmlTransform>& parent);

        bool parseTransformNode(const std::string& defName,
                                VrmlScene& scene,
                                const std::shared_ptr<VrmlTransform>& parent);

        bool parseGroupNode(const std::string& defName,
                            VrmlScene& scene,
                            const std::shared_ptr<VrmlTransform>& parent);

        bool parseShapeNode(VrmlShape& shape);

        bool parseAppearanceNode(VrmlShape& shape);

        bool parseMaterialNode(Material& material);

        bool parseIndexedFaceSetNode(IndexedFaceSet& ifs);

        bool parseCoordinateNode(std::vector<Vec3>& coords);

        bool parseNormalNode(std::vector<Vec3>& normals);

        // Skip an unknown node body (consuming until the matching closing brace).
        void skipNodeBody();

        // ---- Field value parsers ----
        bool parseSFFloat(double& val);
        bool parseSFVec3f(Vec3& v);
        bool parseSFRotation(Vec3& axis, double& angle);
        bool parseSFBool(bool& val);
        bool parseSFString(std::string& val);
        bool parseMFVec3f(std::vector<Vec3>& vals);
        bool parseMFInt32(std::vector<int>& vals);
        bool parseMFFloat(std::vector<double>& vals);

        // ---- State ----
        VrmlScene m_scene;
        std::vector<std::string> m_tokens;
        size_t m_pos = 0;
        std::string m_error;

        // DEF name -> transform  (for USE resolution)
        std::map<std::string, std::shared_ptr<VrmlTransform>> m_defTransforms;

        static constexpr const char* VRML_HEADER = "#VRML V2.0";
    };

} // namespace Odb::Lib::Converter
