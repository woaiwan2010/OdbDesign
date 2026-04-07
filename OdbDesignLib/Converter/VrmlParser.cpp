#include "VrmlParser.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <cctype>

namespace Odb::Lib::Converter
{

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

VrmlParser::VrmlParser() = default;

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

bool VrmlParser::ParseFile(const std::filesystem::path& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        m_error = "Cannot open file: " + path.string();
        return false;
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();
    return ParseString(ss.str());
}

bool VrmlParser::ParseString(const std::string& vrmlText)
{
    m_scene = {};
    m_tokens.clear();
    m_pos = 0;
    m_error.clear();
    m_defTransforms.clear();

    tokenize(vrmlText);

    if (m_tokens.empty())
    {
        m_error = "Empty VRML input";
        return false;
    }

    // The first non-comment token should start with "#VRML V2.0".
    // Our tokeniser skips comments, but the very first line is the header
    // comment and would have been skipped.  We just proceed without
    // enforcing the header so unit tests with partial snippets still work.

    return parseScene();
}

const VrmlScene& VrmlParser::GetScene() const { return m_scene; }
const std::string& VrmlParser::GetErrorMessage() const { return m_error; }

// ---------------------------------------------------------------------------
// Tokeniser
// ---------------------------------------------------------------------------

bool VrmlParser::isSpecial(char c) const
{
    return c == '{' || c == '}' || c == '[' || c == ']' || c == ',';
}

void VrmlParser::tokenize(const std::string& text)
{
    size_t i = 0;
    const size_t n = text.size();

    while (i < n)
    {
        char c = text[i];

        // Skip whitespace
        if (std::isspace(static_cast<unsigned char>(c)))
        {
            ++i;
            continue;
        }

        // Comment: skip to end of line
        if (c == '#')
        {
            while (i < n && text[i] != '\n')
                ++i;
            continue;
        }

        // Quoted string
        if (c == '"')
        {
            std::string tok;
            tok += c;
            ++i;
            while (i < n && text[i] != '"')
            {
                if (text[i] == '\\' && i + 1 < n)
                {
                    tok += text[i];
                    tok += text[i + 1];
                    i += 2;
                }
                else
                {
                    tok += text[i++];
                }
            }
            if (i < n) tok += text[i++]; // closing "
            m_tokens.push_back(tok);
            continue;
        }

        // Special single-char tokens
        if (isSpecial(c))
        {
            m_tokens.push_back(std::string(1, c));
            ++i;
            continue;
        }

        // Regular token (number, keyword, identifier)
        std::string tok;
        while (i < n && !std::isspace(static_cast<unsigned char>(text[i]))
               && !isSpecial(text[i]) && text[i] != '#' && text[i] != '"')
        {
            tok += text[i++];
        }
        if (!tok.empty())
            m_tokens.push_back(tok);
    }
}

// ---------------------------------------------------------------------------
// Token access helpers
// ---------------------------------------------------------------------------

const std::string& VrmlParser::peek(int offset) const
{
    static const std::string empty;
    size_t idx = m_pos + static_cast<size_t>(offset);
    if (idx >= m_tokens.size()) return empty;
    return m_tokens[idx];
}

const std::string& VrmlParser::consume()
{
    static const std::string empty;
    if (m_pos >= m_tokens.size()) return empty;
    return m_tokens[m_pos++];
}

bool VrmlParser::atEnd() const { return m_pos >= m_tokens.size(); }

bool VrmlParser::expect(const std::string& token)
{
    if (atEnd() || peek() != token)
    {
        m_error = "Expected '" + token + "' but got '" + peek() + "'";
        return false;
    }
    consume();
    return true;
}

// ---------------------------------------------------------------------------
// Scene / node parsers
// ---------------------------------------------------------------------------

bool VrmlParser::parseScene()
{
    while (!atEnd())
    {
        if (!parseNode(m_scene, nullptr))
        {
            // Soft errors: skip unknown top-level tokens and continue.
            if (!atEnd()) consume();
        }
    }
    return true;
}

bool VrmlParser::parseNode(VrmlScene& scene,
                            const std::shared_ptr<VrmlTransform>& parent)
{
    if (atEnd()) return false;

    std::string token = peek();

    // ---- DEF ----
    std::string defName;
    if (token == "DEF")
    {
        consume(); // DEF
        if (atEnd()) return false;
        defName = consume(); // name
        if (atEnd()) return false;
        token = peek();
    }

    // ---- USE ----
    if (token == "USE")
    {
        consume(); // USE
        if (atEnd()) return false;
        std::string useName = consume();
        auto it = m_defTransforms.find(useName);
        if (it != m_defTransforms.end())
        {
            if (parent)
                parent->children.push_back(it->second);
            else
                scene.transforms.push_back(it->second);
        }
        return true;
    }

    // ---- Transform / Group ----
    if (token == "Transform")
    {
        consume();
        return parseTransformNode(defName, scene, parent);
    }

    if (token == "Group")
    {
        consume();
        return parseGroupNode(defName, scene, parent);
    }

    // ---- Shape ----
    if (token == "Shape")
    {
        consume();
        VrmlShape shape;
        if (!parseShapeNode(shape)) return false;
        if (parent)
            parent->shapes.push_back(shape);
        else
            scene.shapes.push_back(shape);
        return true;
    }

    // ---- WorldInfo or other metadata nodes: just skip ----
    if (token == "WorldInfo" || token == "Background" || token == "NavigationInfo"
        || token == "Viewpoint" || token == "DirectionalLight"
        || token == "PointLight" || token == "SpotLight"
        || token == "Inline" || token == "Script"
        || token == "LOD" || token == "Switch")
    {
        consume(); // node type
        skipNodeBody();
        return true;
    }

    // Unknown token – return false so the caller can skip
    return false;
}

bool VrmlParser::parseTransformNode(const std::string& defName,
                                    VrmlScene& scene,
                                    const std::shared_ptr<VrmlTransform>& parent)
{
    if (!expect("{")) return false;

    auto xf = std::make_shared<VrmlTransform>();

    while (!atEnd() && peek() != "}")
    {
        std::string field = consume();

        if (field == "translation")
        {
            parseSFVec3f(xf->translation);
        }
        else if (field == "rotation")
        {
            parseSFRotation(xf->rotationAxis, xf->rotationAngle);
        }
        else if (field == "scale")
        {
            parseSFVec3f(xf->scale);
        }
        else if (field == "center")
        {
            parseSFVec3f(xf->center);
        }
        else if (field == "scaleOrientation")
        {
            parseSFRotation(xf->scaleOrientationAxis, xf->scaleOrientationAngle);
        }
        else if (field == "children")
        {
            // children can be a MFNode (array) or inline node
            if (peek() == "[")
            {
                consume(); // [
                while (!atEnd() && peek() != "]")
                {
                    parseNode(scene, xf);
                }
                if (!expect("]")) return false;
            }
            else
            {
                parseNode(scene, xf);
            }
        }
        else
        {
            // Unknown field – skip value
            skipNodeBody();
        }
    }

    if (!expect("}")) return false;

    if (!defName.empty())
        m_defTransforms[defName] = xf;

    if (parent)
        parent->children.push_back(xf);
    else
        scene.transforms.push_back(xf);

    return true;
}

bool VrmlParser::parseGroupNode(const std::string& defName,
                                VrmlScene& scene,
                                const std::shared_ptr<VrmlTransform>& parent)
{
    if (!expect("{")) return false;

    // A Group is just a Transform with identity transform values.
    auto grp = std::make_shared<VrmlTransform>();

    while (!atEnd() && peek() != "}")
    {
        std::string field = consume();

        if (field == "children")
        {
            if (peek() == "[")
            {
                consume(); // [
                while (!atEnd() && peek() != "]")
                    parseNode(scene, grp);
                if (!expect("]")) return false;
            }
            else
            {
                parseNode(scene, grp);
            }
        }
        else if (field == "bboxCenter" || field == "bboxSize")
        {
            Vec3 dummy;
            parseSFVec3f(dummy);
        }
        else
        {
            skipNodeBody();
        }
    }

    if (!expect("}")) return false;

    if (!defName.empty())
        m_defTransforms[defName] = grp;

    if (parent)
        parent->children.push_back(grp);
    else
        scene.transforms.push_back(grp);

    return true;
}

bool VrmlParser::parseShapeNode(VrmlShape& shape)
{
    if (!expect("{")) return false;

    while (!atEnd() && peek() != "}")
    {
        std::string field = consume();

        if (field == "appearance")
        {
            if (peek() == "Appearance")
            {
                consume(); // Appearance
                parseAppearanceNode(shape);
            }
            else if (peek() == "NULL")
            {
                consume();
            }
            else
            {
                // Could be a USE reference; skip for now
                skipNodeBody();
            }
        }
        else if (field == "geometry")
        {
            if (peek() == "IndexedFaceSet")
            {
                consume();
                shape.hasGeometry = true;
                parseIndexedFaceSetNode(shape.geometry);
            }
            else if (peek() == "NULL")
            {
                consume();
            }
            else
            {
                // Other geometry types (Box, Sphere, etc.) – skip
                std::string geomType = consume();
                skipNodeBody();
            }
        }
        else
        {
            skipNodeBody();
        }
    }

    if (!expect("}")) return false;
    return true;
}

bool VrmlParser::parseAppearanceNode(VrmlShape& shape)
{
    if (!expect("{")) return false;

    while (!atEnd() && peek() != "}")
    {
        std::string field = consume();

        if (field == "material")
        {
            if (peek() == "Material")
            {
                consume();
                shape.hasMaterial = true;
                parseMaterialNode(shape.material);
            }
            else if (peek() == "NULL")
            {
                consume();
            }
            else
            {
                skipNodeBody();
            }
        }
        else
        {
            // texture, textureTransform, etc.
            skipNodeBody();
        }
    }

    if (!expect("}")) return false;
    return true;
}

bool VrmlParser::parseMaterialNode(Material& mat)
{
    if (!expect("{")) return false;

    while (!atEnd() && peek() != "}")
    {
        std::string field = consume();

        if (field == "diffuseColor")
        {
            Vec3 v;
            parseSFVec3f(v);
            mat.diffuseColor = {v.x, v.y, v.z};
        }
        else if (field == "specularColor")
        {
            Vec3 v;
            parseSFVec3f(v);
            mat.specularColor = {v.x, v.y, v.z};
        }
        else if (field == "emissiveColor")
        {
            Vec3 v;
            parseSFVec3f(v);
            mat.emissiveColor = {v.x, v.y, v.z};
        }
        else if (field == "ambientIntensity")
        {
            parseSFFloat(mat.ambientIntensity);
        }
        else if (field == "shininess")
        {
            parseSFFloat(mat.shininess);
        }
        else if (field == "transparency")
        {
            parseSFFloat(mat.transparency);
        }
        else
        {
            skipNodeBody();
        }
    }

    if (!expect("}")) return false;
    return true;
}

bool VrmlParser::parseIndexedFaceSetNode(IndexedFaceSet& ifs)
{
    if (!expect("{")) return false;

    while (!atEnd() && peek() != "}")
    {
        std::string field = consume();

        if (field == "coord")
        {
            if (peek() == "Coordinate")
            {
                consume();
                parseCoordinateNode(ifs.coords);
            }
            else if (peek() == "NULL")
            {
                consume();
            }
            else
            {
                skipNodeBody();
            }
        }
        else if (field == "coordIndex")
        {
            parseMFInt32(ifs.coordIndex);
        }
        else if (field == "normal")
        {
            if (peek() == "Normal")
            {
                consume();
                parseNormalNode(ifs.normals);
            }
            else if (peek() == "NULL")
            {
                consume();
            }
            else
            {
                skipNodeBody();
            }
        }
        else if (field == "normalIndex")
        {
            parseMFInt32(ifs.normalIndex);
        }
        else if (field == "normalPerVertex")
        {
            parseSFBool(ifs.normalPerVertex);
        }
        else if (field == "ccw")
        {
            parseSFBool(ifs.ccw);
        }
        else if (field == "solid")
        {
            parseSFBool(ifs.solid);
        }
        else if (field == "convex")
        {
            parseSFBool(ifs.convex);
        }
        else if (field == "texCoord" || field == "texCoordIndex"
                 || field == "color" || field == "colorIndex"
                 || field == "colorPerVertex" || field == "creaseAngle")
        {
            // skip texture / color fields
            if (peek() == "TextureCoordinate" || peek() == "Color")
            {
                consume();
                skipNodeBody();
            }
            else if (peek() == "NULL")
            {
                consume();
            }
            else if (peek() == "[")
            {
                // MF field
                std::vector<int> dummy;
                parseMFInt32(dummy);
            }
            else
            {
                double dummy = 0.0;
                parseSFFloat(dummy);
            }
        }
        else
        {
            skipNodeBody();
        }
    }

    if (!expect("}")) return false;
    return true;
}

bool VrmlParser::parseCoordinateNode(std::vector<Vec3>& coords)
{
    if (!expect("{")) return false;

    while (!atEnd() && peek() != "}")
    {
        std::string field = consume();
        if (field == "point")
        {
            parseMFVec3f(coords);
        }
        else
        {
            skipNodeBody();
        }
    }

    if (!expect("}")) return false;
    return true;
}

bool VrmlParser::parseNormalNode(std::vector<Vec3>& normals)
{
    if (!expect("{")) return false;

    while (!atEnd() && peek() != "}")
    {
        std::string field = consume();
        if (field == "vector")
        {
            parseMFVec3f(normals);
        }
        else
        {
            skipNodeBody();
        }
    }

    if (!expect("}")) return false;
    return true;
}

// ---------------------------------------------------------------------------
// Skip unknown node body
// ---------------------------------------------------------------------------

void VrmlParser::skipNodeBody()
{
    // If the next token is a node-type or field value, consume it.
    // If it opens a brace, consume the entire block.
    // If it opens a bracket, consume the entire array.
    if (atEnd()) return;

    const std::string& tok = peek();

    if (tok == "{")
    {
        consume(); // {
        int depth = 1;
        while (!atEnd() && depth > 0)
        {
            const std::string& t = consume();
            if (t == "{") ++depth;
            else if (t == "}") --depth;
        }
    }
    else if (tok == "[")
    {
        consume(); // [
        int depth = 1;
        while (!atEnd() && depth > 0)
        {
            const std::string& t = consume();
            if (t == "[") ++depth;
            else if (t == "]") --depth;
        }
    }
    else
    {
        // Single value token, just consume it.
        consume();
    }
}

// ---------------------------------------------------------------------------
// Field value parsers
// ---------------------------------------------------------------------------

bool VrmlParser::parseSFFloat(double& val)
{
    if (atEnd()) return false;
    try { val = std::stod(consume()); return true; }
    catch (...) { return false; }
}

bool VrmlParser::parseSFVec3f(Vec3& v)
{
    return parseSFFloat(v.x) && parseSFFloat(v.y) && parseSFFloat(v.z);
}

bool VrmlParser::parseSFRotation(Vec3& axis, double& angle)
{
    return parseSFFloat(axis.x) && parseSFFloat(axis.y)
        && parseSFFloat(axis.z) && parseSFFloat(angle);
}

bool VrmlParser::parseSFBool(bool& val)
{
    if (atEnd()) return false;
    const std::string& tok = consume();
    if (tok == "TRUE" || tok == "true" || tok == "1")  { val = true;  return true; }
    if (tok == "FALSE" || tok == "false" || tok == "0") { val = false; return true; }
    return false;
}

bool VrmlParser::parseSFString(std::string& val)
{
    if (atEnd()) return false;
    const std::string& tok = consume();
    if (tok.size() >= 2 && tok.front() == '"' && tok.back() == '"')
        val = tok.substr(1, tok.size() - 2);
    else
        val = tok;
    return true;
}

bool VrmlParser::parseMFVec3f(std::vector<Vec3>& vals)
{
    bool hasBracket = (peek() == "[");
    if (hasBracket) consume();

    while (!atEnd())
    {
        const std::string& t = peek();
        if (t == "]" || t == "}" || t == "{") break;
        // comma-only token: skip
        if (t == ",") { consume(); continue; }

        Vec3 v;
        if (!parseSFVec3f(v)) break;
        vals.push_back(v);

        if (peek() == ",") consume(); // optional comma
    }

    if (hasBracket && peek() == "]") consume();
    return true;
}

bool VrmlParser::parseMFInt32(std::vector<int>& vals)
{
    bool hasBracket = (peek() == "[");
    if (hasBracket) consume();

    while (!atEnd())
    {
        const std::string& t = peek();
        if (t == "]" || t == "}" || t == "{") break;
        if (t == ",") { consume(); continue; }

        try
        {
            int v = std::stoi(t);
            consume();
            vals.push_back(v);
        }
        catch (...)
        {
            break;
        }

        if (peek() == ",") consume();
    }

    if (hasBracket && peek() == "]") consume();
    return true;
}

bool VrmlParser::parseMFFloat(std::vector<double>& vals)
{
    bool hasBracket = (peek() == "[");
    if (hasBracket) consume();

    while (!atEnd())
    {
        const std::string& t = peek();
        if (t == "]" || t == "}" || t == "{") break;
        if (t == ",") { consume(); continue; }

        try
        {
            double v = std::stod(t);
            consume();
            vals.push_back(v);
        }
        catch (...)
        {
            break;
        }

        if (peek() == ",") consume();
    }

    if (hasBracket && peek() == "]") consume();
    return true;
}

} // namespace Odb::Lib::Converter
