#include <gtest/gtest.h>
#include <Converter/VrmlParser.h>
#include <Converter/VrmlToStepConverter.h>
#include <Converter/StepWriter.h>
#include <string>
#include <vector>
#include <cstring>

using namespace Odb::Lib::Converter;

namespace Odb::Test
{

// ---------------------------------------------------------------------------
// VrmlParser tests
// ---------------------------------------------------------------------------

TEST(VrmlParserTests, ParseEmptyString_ReturnsFalse)
{
    VrmlParser parser;
    EXPECT_FALSE(parser.ParseString(""));
}

TEST(VrmlParserTests, ParseHeader_NoNodes)
{
    VrmlParser parser;
    EXPECT_TRUE(parser.ParseString("#VRML V2.0 utf8\n"));
    EXPECT_TRUE(parser.GetScene().shapes.empty());
    EXPECT_TRUE(parser.GetScene().transforms.empty());
}

TEST(VrmlParserTests, ParseSingleTriangle_ExtractsGeometry)
{
    const std::string vrml = R"(
#VRML V2.0 utf8
Shape {
  appearance Appearance {
    material Material {
      diffuseColor 1.0 0.0 0.0
    }
  }
  geometry IndexedFaceSet {
    coord Coordinate {
      point [
        0 0 0,
        1 0 0,
        0 1 0
      ]
    }
    coordIndex [ 0 1 2 -1 ]
  }
}
)";

    VrmlParser parser;
    ASSERT_TRUE(parser.ParseString(vrml));

    const auto& scene = parser.GetScene();
    ASSERT_EQ(scene.shapes.size(), 1u);
    EXPECT_TRUE(scene.shapes[0].hasGeometry);
    EXPECT_TRUE(scene.shapes[0].hasMaterial);

    const auto& coords = scene.shapes[0].geometry.coords;
    ASSERT_EQ(coords.size(), 3u);
    EXPECT_NEAR(coords[0].x, 0.0, 1e-9);
    EXPECT_NEAR(coords[1].x, 1.0, 1e-9);
    EXPECT_NEAR(coords[2].y, 1.0, 1e-9);

    const auto& ci = scene.shapes[0].geometry.coordIndex;
    ASSERT_EQ(ci.size(), 4u);
    EXPECT_EQ(ci[0], 0);
    EXPECT_EQ(ci[1], 1);
    EXPECT_EQ(ci[2], 2);
    EXPECT_EQ(ci[3], -1);
}

TEST(VrmlParserTests, ParseMaterial_ColorExtracted)
{
    const std::string vrml = R"(
Shape {
  appearance Appearance {
    material Material {
      diffuseColor 0.2 0.4 0.6
      transparency 0.5
    }
  }
  geometry IndexedFaceSet {
    coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
    coordIndex [ 0 1 2 -1 ]
  }
}
)";

    VrmlParser parser;
    ASSERT_TRUE(parser.ParseString(vrml));
    ASSERT_EQ(parser.GetScene().shapes.size(), 1u);
    const auto& mat = parser.GetScene().shapes[0].material;
    EXPECT_NEAR(mat.diffuseColor.r, 0.2, 1e-9);
    EXPECT_NEAR(mat.diffuseColor.g, 0.4, 1e-9);
    EXPECT_NEAR(mat.diffuseColor.b, 0.6, 1e-9);
    EXPECT_NEAR(mat.transparency, 0.5, 1e-9);
}

TEST(VrmlParserTests, ParseTransform_ExtractsShapesAndTranslation)
{
    const std::string vrml = R"(
Transform {
  translation 1.0 2.0 3.0
  children [
    Shape {
      geometry IndexedFaceSet {
        coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
        coordIndex [ 0 1 2 -1 ]
      }
    }
  ]
}
)";

    VrmlParser parser;
    ASSERT_TRUE(parser.ParseString(vrml));
    const auto& scene = parser.GetScene();
    ASSERT_EQ(scene.transforms.size(), 1u);
    const auto& xf = *scene.transforms[0];
    EXPECT_NEAR(xf.translation.x, 1.0, 1e-9);
    EXPECT_NEAR(xf.translation.y, 2.0, 1e-9);
    EXPECT_NEAR(xf.translation.z, 3.0, 1e-9);
    ASSERT_EQ(xf.shapes.size(), 1u);
    EXPECT_TRUE(xf.shapes[0].hasGeometry);
}

TEST(VrmlParserTests, ParseNestedTransforms)
{
    const std::string vrml = R"(
Transform {
  translation 1 0 0
  children [
    Transform {
      translation 0 2 0
      children [
        Shape {
          geometry IndexedFaceSet {
            coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
            coordIndex [ 0 1 2 -1 ]
          }
        }
      ]
    }
  ]
}
)";

    VrmlParser parser;
    ASSERT_TRUE(parser.ParseString(vrml));
    const auto& scene = parser.GetScene();
    ASSERT_EQ(scene.transforms.size(), 1u);
    ASSERT_EQ(scene.transforms[0]->children.size(), 1u);
    ASSERT_EQ(scene.transforms[0]->children[0]->shapes.size(), 1u);
}

TEST(VrmlParserTests, ParseGroupNode)
{
    const std::string vrml = R"(
Group {
  children [
    Shape {
      geometry IndexedFaceSet {
        coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
        coordIndex [ 0 1 2 -1 ]
      }
    }
  ]
}
)";

    VrmlParser parser;
    ASSERT_TRUE(parser.ParseString(vrml));
    const auto& scene = parser.GetScene();
    ASSERT_EQ(scene.transforms.size(), 1u);
    ASSERT_EQ(scene.transforms[0]->shapes.size(), 1u);
}

TEST(VrmlParserTests, ParseDefUse_ReusesTransform)
{
    const std::string vrml = R"(
DEF MyXf Transform {
  translation 5 0 0
  children [
    Shape {
      geometry IndexedFaceSet {
        coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
        coordIndex [ 0 1 2 -1 ]
      }
    }
  ]
}
USE MyXf
)";

    VrmlParser parser;
    ASSERT_TRUE(parser.ParseString(vrml));
    const auto& scene = parser.GetScene();
    // DEF adds to scene, USE also adds (pointing to same object)
    EXPECT_GE(scene.transforms.size(), 1u);
}

TEST(VrmlParserTests, ParseQuadFace_FourVertices)
{
    const std::string vrml = R"(
Shape {
  geometry IndexedFaceSet {
    coord Coordinate {
      point [ 0 0 0, 1 0 0, 1 1 0, 0 1 0 ]
    }
    coordIndex [ 0 1 2 3 -1 ]
  }
}
)";

    VrmlParser parser;
    ASSERT_TRUE(parser.ParseString(vrml));
    ASSERT_EQ(parser.GetScene().shapes.size(), 1u);
    const auto& ci = parser.GetScene().shapes[0].geometry.coordIndex;
    ASSERT_EQ(ci.size(), 5u);  // 4 indices + -1
    EXPECT_EQ(ci[3], 3);
    EXPECT_EQ(ci[4], -1);
}

TEST(VrmlParserTests, IgnoresUnknownNodes)
{
    const std::string vrml = R"(
WorldInfo { title "test" }
Background { skyColor 0.5 0.5 1.0 }
Shape {
  geometry IndexedFaceSet {
    coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
    coordIndex [ 0 1 2 -1 ]
  }
}
)";

    VrmlParser parser;
    ASSERT_TRUE(parser.ParseString(vrml));
    EXPECT_EQ(parser.GetScene().shapes.size(), 1u);
}

// ---------------------------------------------------------------------------
// VrmlToStepConverter tests
// ---------------------------------------------------------------------------

TEST(VrmlToStepConverterTests, ConvertSingleTriangle_ProducesStepContent)
{
    const std::string vrml = R"(
Shape {
  geometry IndexedFaceSet {
    coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
    coordIndex [ 0 1 2 -1 ]
  }
}
)";

    VrmlToStepConverter conv;
    std::string step = conv.ConvertString(vrml, "TestPart");
    ASSERT_FALSE(step.empty()) << conv.GetErrorMessage();

    // Validate STEP structure
    EXPECT_NE(step.find("ISO-10303-21;"), std::string::npos);
    EXPECT_NE(step.find("END-ISO-10303-21;"), std::string::npos);
    EXPECT_NE(step.find("AUTOMOTIVE_DESIGN"), std::string::npos);
    EXPECT_NE(step.find("FACETED_BREP"), std::string::npos);
    EXPECT_NE(step.find("ADVANCED_FACE"), std::string::npos);
    EXPECT_NE(step.find("CLOSED_SHELL"), std::string::npos);
    EXPECT_NE(step.find("TestPart"), std::string::npos);
}

TEST(VrmlToStepConverterTests, ConvertQuadFace_ProducesTwoTriangles)
{
    // A quad should be fan-triangulated into 2 triangles
    const std::string vrml = R"(
Shape {
  geometry IndexedFaceSet {
    coord Coordinate {
      point [ 0 0 0, 1 0 0, 1 1 0, 0 1 0 ]
    }
    coordIndex [ 0 1 2 3 -1 ]
  }
}
)";

    VrmlToStepConverter conv;
    std::string step = conv.ConvertString(vrml);
    ASSERT_FALSE(step.empty()) << conv.GetErrorMessage();

    // Count ADVANCED_FACE occurrences: 2 per quad
    size_t count = 0;
    size_t pos = 0;
    while ((pos = step.find("=ADVANCED_FACE(", pos)) != std::string::npos)
    {
        ++count;
        ++pos;
    }
    EXPECT_EQ(count, 2u);
}

TEST(VrmlToStepConverterTests, ConvertTransformedShape_AppliesTranslation)
{
    // Shape at origin, translated by (10, 0, 0)
    const std::string vrml = R"(
Transform {
  translation 10.0 0.0 0.0
  children [
    Shape {
      geometry IndexedFaceSet {
        coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
        coordIndex [ 0 1 2 -1 ]
      }
    }
  ]
}
)";

    VrmlToStepConverter conv;
    std::string step = conv.ConvertString(vrml);
    ASSERT_FALSE(step.empty()) << conv.GetErrorMessage();
    // The translated vertex (10,0,0) should appear in the STEP file
    EXPECT_NE(step.find("10"), std::string::npos);
}

TEST(VrmlToStepConverterTests, ConvertEmptyScene_ReturnsEmpty)
{
    const std::string vrml = "#VRML V2.0 utf8\n";
    VrmlToStepConverter conv;
    std::string step = conv.ConvertString(vrml);
    EXPECT_TRUE(step.empty());
    EXPECT_FALSE(conv.GetErrorMessage().empty());
}

TEST(VrmlToStepConverterTests, ConvertMultipleFaces_AllFacesPresent)
{
    const std::string vrml = R"(
Shape {
  geometry IndexedFaceSet {
    coord Coordinate {
      point [
        0 0 0, 1 0 0, 0.5 1 0,
        0 0 1, 1 0 1, 0.5 1 1
      ]
    }
    coordIndex [ 0 1 2 -1, 3 4 5 -1 ]
  }
}
)";

    VrmlToStepConverter conv;
    std::string step = conv.ConvertString(vrml);
    ASSERT_FALSE(step.empty()) << conv.GetErrorMessage();

    size_t count = 0;
    size_t pos = 0;
    while ((pos = step.find("=ADVANCED_FACE(", pos)) != std::string::npos)
    {
        ++count;
        ++pos;
    }
    EXPECT_EQ(count, 2u);
}

// ---------------------------------------------------------------------------
// StepWriter tests
// ---------------------------------------------------------------------------

TEST(StepWriterTests, WriteNoTriangles_ReturnsFalse)
{
    StepWriter writer;
    std::string result = writer.WriteToString({}, "Part");
    EXPECT_TRUE(result.empty());
    EXPECT_FALSE(writer.GetErrorMessage().empty());
}

TEST(StepWriterTests, WriteSingleTriangle_ValidStepHeader)
{
    Triangle tri;
    tri.v0 = {0, 0, 0};
    tri.v1 = {1, 0, 0};
    tri.v2 = {0, 1, 0};

    StepWriter writer;
    std::string result = writer.WriteToString({tri}, "TestPart");
    ASSERT_FALSE(result.empty());

    EXPECT_NE(result.find("ISO-10303-21;"), std::string::npos);
    EXPECT_NE(result.find("ENDSEC;"), std::string::npos);
    EXPECT_NE(result.find("END-ISO-10303-21;"), std::string::npos);
    EXPECT_NE(result.find("DATA;"), std::string::npos);
    EXPECT_NE(result.find("FILE_SCHEMA"), std::string::npos);
    EXPECT_NE(result.find("AUTOMOTIVE_DESIGN"), std::string::npos);
}

TEST(StepWriterTests, WriteSingleTriangle_HasRequiredEntities)
{
    Triangle tri;
    tri.v0 = {0, 0, 0};
    tri.v1 = {1, 0, 0};
    tri.v2 = {0, 1, 0};

    StepWriter writer;
    std::string result = writer.WriteToString({tri}, "Part");
    ASSERT_FALSE(result.empty());

    EXPECT_NE(result.find("CARTESIAN_POINT"), std::string::npos);
    EXPECT_NE(result.find("VERTEX_POINT"), std::string::npos);
    EXPECT_NE(result.find("EDGE_CURVE"), std::string::npos);
    EXPECT_NE(result.find("EDGE_LOOP"), std::string::npos);
    EXPECT_NE(result.find("FACE_OUTER_BOUND"), std::string::npos);
    EXPECT_NE(result.find("PLANE"), std::string::npos);
    EXPECT_NE(result.find("ADVANCED_FACE"), std::string::npos);
    EXPECT_NE(result.find("CLOSED_SHELL"), std::string::npos);
    EXPECT_NE(result.find("FACETED_BREP"), std::string::npos);
    EXPECT_NE(result.find("ADVANCED_BREP_SHAPE_REPRESENTATION"), std::string::npos);
    EXPECT_NE(result.find("SHAPE_DEFINITION_REPRESENTATION"), std::string::npos);
}

TEST(StepWriterTests, WriteMultipleTriangles_MultipleAdvancedFaces)
{
    std::vector<Triangle> tris(3);
    tris[0] = {{0,0,0}, {1,0,0}, {0,1,0}, {}};
    tris[1] = {{0,0,1}, {1,0,1}, {0,1,1}, {}};
    tris[2] = {{0,0,2}, {1,0,2}, {0,1,2}, {}};

    StepWriter writer;
    std::string result = writer.WriteToString(tris, "Part");
    ASSERT_FALSE(result.empty());

    size_t count = 0;
    size_t pos = 0;
    while ((pos = result.find("=ADVANCED_FACE(", pos)) != std::string::npos)
    {
        ++count;
        ++pos;
    }
    EXPECT_EQ(count, 3u);
}

TEST(StepWriterTests, PartNameEmbeddedInOutput)
{
    Triangle tri{{0,0,0},{1,0,0},{0,1,0},{}};
    StepWriter writer;
    std::string result = writer.WriteToString({tri}, "MyComponent");
    ASSERT_FALSE(result.empty());
    EXPECT_NE(result.find("MyComponent"), std::string::npos);
}

} // namespace Odb::Test
