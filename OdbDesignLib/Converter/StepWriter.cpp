#include "StepWriter.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <chrono>

namespace Odb::Lib::Converter
{

// ---------------------------------------------------------------------------
// Small math helpers
// ---------------------------------------------------------------------------

static Vec3 vecSub(const Vec3& a, const Vec3& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec3 vecCross(const Vec3& a, const Vec3& b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

static double vecLen(const Vec3& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

static Vec3 vecNorm(const Vec3& v)
{
    double len = vecLen(v);
    if (len < 1e-15) return {0.0, 0.0, 1.0};
    return {v.x / len, v.y / len, v.z / len};
}

// Choose a direction perpendicular to n (for AXIS2_PLACEMENT_3D x-dir).
static Vec3 perpendicular(const Vec3& n)
{
    // Use the axis least aligned with n
    Vec3 ref = (std::abs(n.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    Vec3 p = vecCross(n, ref);
    return vecNorm(p);
}

// ---------------------------------------------------------------------------
// StepWriter implementation
// ---------------------------------------------------------------------------

StepWriter::StepWriter() = default;

const std::string& StepWriter::GetErrorMessage() const { return m_error; }

bool StepWriter::Write(const std::filesystem::path& outputPath,
                       const std::vector<Triangle>& triangles,
                       const std::string& partName)
{
    std::string content = generate(triangles, partName);
    if (content.empty()) return false;

    std::ofstream ofs(outputPath);
    if (!ofs.is_open())
    {
        m_error = "Cannot write to file: " + outputPath.string();
        return false;
    }
    ofs << content;
    return true;
}

std::string StepWriter::WriteToString(const std::vector<Triangle>& triangles,
                                      const std::string& partName)
{
    return generate(triangles, partName);
}

// ---------------------------------------------------------------------------
// STEP generation
// ---------------------------------------------------------------------------

std::string StepWriter::generate(const std::vector<Triangle>& triangles,
                                  const std::string& partName)
{
    if (triangles.empty())
    {
        m_error = "No triangles to write";
        return {};
    }

    // Entity counter (1-based)
    int id = 0;
    auto nextId = [&]() { return ++id; };

    // Timestamp
    std::time_t now = std::time(nullptr);
    char timeBuf[64] = {};
    std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%dT%H:%M:%S", std::gmtime(&now));

    std::ostringstream s;
    s << std::fixed << std::setprecision(10);

    // ---- HEADER section ----
    s << "ISO-10303-21;\n";
    s << "HEADER;\n";
    s << "FILE_DESCRIPTION(('VRML to STEP Conversion'),'2;1');\n";
    s << "FILE_NAME('" << partName << ".stp','" << timeBuf
      << "',(''),(''),'OdbDesign VrmlToStep','OdbDesign VrmlToStep','Unknown');\n";
    s << "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n";
    s << "ENDSEC;\n";
    s << "DATA;\n";

    // ---- Product structure ----
    int idAppCtx        = nextId(); // #1
    int idProdCtx       = nextId(); // #2
    int idAppProtoDef   = nextId(); // #3
    int idProduct       = nextId(); // #4
    int idProdDefForm   = nextId(); // #5
    int idProdDefCtx    = nextId(); // #6
    int idProdDef       = nextId(); // #7
    int idProdDefShape  = nextId(); // #8

    s << "#" << idAppCtx      << "=APPLICATION_CONTEXT('automotive design');\n";
    s << "#" << idProdCtx     << "=PRODUCT_CONTEXT('',#" << idAppCtx << ",'mechanical');\n";
    s << "#" << idAppProtoDef << "=APPLICATION_PROTOCOL_DEFINITION('international standard'"
                               << ",'AUTOMOTIVE_DESIGN',2003,#" << idAppCtx << ");\n";
    s << "#" << idProduct     << "=PRODUCT('" << partName << "','" << partName
                               << "','',(#" << idProdCtx << "));\n";
    s << "#" << idProdDefForm << "=PRODUCT_DEFINITION_FORMATION('','',#" << idProduct << ");\n";
    s << "#" << idProdDefCtx  << "=PRODUCT_DEFINITION_CONTEXT('part definition',#"
                               << idAppCtx << ",'design');\n";
    s << "#" << idProdDef     << "=PRODUCT_DEFINITION('design','',#" << idProdDefForm
                               << ",#" << idProdDefCtx << ");\n";
    s << "#" << idProdDefShape << "=PRODUCT_DEFINITION_SHAPE('','',#" << idProdDef << ");\n";

    // ---- Geometry context ----
    int idUncertainty   = nextId();
    int idLenUnit       = nextId();
    int idAngleUnit     = nextId();
    int idSolidAngleUnit = nextId();
    int idGeomCtx       = nextId();

    s << "#" << idLenUnit       << "=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n";
    s << "#" << idAngleUnit     << "=( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n";
    s << "#" << idSolidAngleUnit<< "=( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n";
    s << "#" << idUncertainty   << "=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-7),#"
                                 << idLenUnit << ",'distance_accuracy_value','');\n";
    s << "#" << idGeomCtx       << "=( GEOMETRIC_REPRESENTATION_CONTEXT(3)"
                                 << " GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#" << idUncertainty << "))"
                                 << " GLOBAL_UNIT_ASSIGNED_CONTEXT((#" << idLenUnit
                                 << ",#" << idAngleUnit
                                 << ",#" << idSolidAngleUnit << "))"
                                 << " REPRESENTATION_CONTEXT('Context #1',"
                                 << "'3D Context with UNIT and UNCERTAINTY') );\n";

    // ---- Faces ----
    // For each triangle we emit:
    //   3 CARTESIAN_POINTs   (vertices)
    //   3 VERTEX_POINTs
    //   3 DIRECTIONs + 3 VECTORs + 3 LINEs (edge geometry)
    //   3 EDGE_CURVEs
    //   3 ORIENTED_EDGEs
    //   1 EDGE_LOOP
    //   1 FACE_OUTER_BOUND
    //   1 CARTESIAN_POINT (plane origin/centroid)
    //   2 DIRECTIONs (plane normal + x-axis)
    //   1 AXIS2_PLACEMENT_3D
    //   1 PLANE
    //   1 ADVANCED_FACE

    std::vector<int> faceIds;
    faceIds.reserve(triangles.size());

    auto fmtReal = [](double v) -> std::string {
        // Format a real number for STEP output (avoid trailing zeros, handle E notation)
        std::ostringstream os;
        os << std::setprecision(10) << v;
        std::string str = os.str();
        // STEP requires a decimal point for real literals
        if (str.find('.') == std::string::npos &&
            str.find('e') == std::string::npos &&
            str.find('E') == std::string::npos)
        {
            str += ".";
        }
        return str;
    };

    auto fmtPt = [&](const Vec3& p) -> std::string {
        return "(" + fmtReal(p.x) + "," + fmtReal(p.y) + "," + fmtReal(p.z) + ")";
    };

    for (const auto& tri : triangles)
    {
        // -- Vertices --
        int ptA = nextId();
        int ptB = nextId();
        int ptC = nextId();
        s << "#" << ptA << "=CARTESIAN_POINT(''," << fmtPt(tri.v0) << ");\n";
        s << "#" << ptB << "=CARTESIAN_POINT(''," << fmtPt(tri.v1) << ");\n";
        s << "#" << ptC << "=CARTESIAN_POINT(''," << fmtPt(tri.v2) << ");\n";

        int vpA = nextId();
        int vpB = nextId();
        int vpC = nextId();
        s << "#" << vpA << "=VERTEX_POINT('',#" << ptA << ");\n";
        s << "#" << vpB << "=VERTEX_POINT('',#" << ptB << ");\n";
        s << "#" << vpC << "=VERTEX_POINT('',#" << ptC << ");\n";

        // -- Edge AB (from A to B) --
        Vec3 edgeAB = vecSub(tri.v1, tri.v0);
        Vec3 dirAB  = vecNorm(edgeAB);
        double lenAB = vecLen(edgeAB);
        int dAB = nextId(); int vecAB = nextId(); int lineAB = nextId();
        s << "#" << dAB   << "=DIRECTION(''," << fmtPt(dirAB) << ");\n";
        s << "#" << vecAB << "=VECTOR('',#" << dAB << "," << fmtReal(lenAB) << ");\n";
        s << "#" << lineAB<< "=LINE('',#" << ptA << ",#" << vecAB << ");\n";

        // -- Edge BC (from B to C) --
        Vec3 edgeBC = vecSub(tri.v2, tri.v1);
        Vec3 dirBC  = vecNorm(edgeBC);
        double lenBC = vecLen(edgeBC);
        int dBC = nextId(); int vecBC = nextId(); int lineBC = nextId();
        s << "#" << dBC   << "=DIRECTION(''," << fmtPt(dirBC) << ");\n";
        s << "#" << vecBC << "=VECTOR('',#" << dBC << "," << fmtReal(lenBC) << ");\n";
        s << "#" << lineBC<< "=LINE('',#" << ptB << ",#" << vecBC << ");\n";

        // -- Edge CA (from C back to A) --
        Vec3 edgeCA = vecSub(tri.v0, tri.v2);
        Vec3 dirCA  = vecNorm(edgeCA);
        double lenCA = vecLen(edgeCA);
        int dCA = nextId(); int vecCA = nextId(); int lineCA = nextId();
        s << "#" << dCA   << "=DIRECTION(''," << fmtPt(dirCA) << ");\n";
        s << "#" << vecCA << "=VECTOR('',#" << dCA << "," << fmtReal(lenCA) << ");\n";
        s << "#" << lineCA<< "=LINE('',#" << ptC << ",#" << vecCA << ");\n";

        // -- Edge curves --
        int ecAB = nextId(); int ecBC = nextId(); int ecCA = nextId();
        s << "#" << ecAB << "=EDGE_CURVE('',#" << vpA << ",#" << vpB << ",#" << lineAB << ",.T.);\n";
        s << "#" << ecBC << "=EDGE_CURVE('',#" << vpB << ",#" << vpC << ",#" << lineBC << ",.T.);\n";
        s << "#" << ecCA << "=EDGE_CURVE('',#" << vpC << ",#" << vpA << ",#" << lineCA << ",.T.);\n";

        // -- Oriented edges (all forward direction) --
        int oeAB = nextId(); int oeBC = nextId(); int oeCA = nextId();
        s << "#" << oeAB << "=ORIENTED_EDGE('',*,*,#" << ecAB << ",.T.);\n";
        s << "#" << oeBC << "=ORIENTED_EDGE('',*,*,#" << ecBC << ",.T.);\n";
        s << "#" << oeCA << "=ORIENTED_EDGE('',*,*,#" << ecCA << ",.T.);\n";

        // -- Edge loop --
        int edgeLoop = nextId();
        s << "#" << edgeLoop << "=EDGE_LOOP('',(#" << oeAB
          << ",#" << oeBC << ",#" << oeCA << "));\n";

        // -- Face outer bound --
        int faceBound = nextId();
        s << "#" << faceBound << "=FACE_OUTER_BOUND('',#" << edgeLoop << ",.T.);\n";

        // -- Plane for the face --
        // Normal = (AB × AC) normalised
        Vec3 AB = vecSub(tri.v1, tri.v0);
        Vec3 AC = vecSub(tri.v2, tri.v0);
        Vec3 normal = vecNorm(vecCross(AB, AC));
        Vec3 xDir   = vecNorm(AB); // x-axis on the plane

        // Centroid as plane origin
        Vec3 origin = {
            (tri.v0.x + tri.v1.x + tri.v2.x) / 3.0,
            (tri.v0.y + tri.v1.y + tri.v2.y) / 3.0,
            (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0
        };

        int ptOrig = nextId(); int dNorm = nextId(); int dX = nextId();
        int axisPlace = nextId(); int plane = nextId();
        s << "#" << ptOrig   << "=CARTESIAN_POINT(''," << fmtPt(origin) << ");\n";
        s << "#" << dNorm    << "=DIRECTION(''," << fmtPt(normal) << ");\n";
        s << "#" << dX       << "=DIRECTION(''," << fmtPt(xDir)   << ");\n";
        s << "#" << axisPlace<< "=AXIS2_PLACEMENT_3D('',#" << ptOrig
                              << ",#" << dNorm << ",#" << dX << ");\n";
        s << "#" << plane    << "=PLANE('',#" << axisPlace << ");\n";

        // -- Advanced face --
        int faceId = nextId();
        s << "#" << faceId   << "=ADVANCED_FACE('',(#" << faceBound << "),#" << plane << ",.T.);\n";

        faceIds.push_back(faceId);
    }

    // ---- Closed shell ----
    int shellId = nextId();
    s << "#" << shellId << "=CLOSED_SHELL('',(";
    for (size_t i = 0; i < faceIds.size(); ++i)
    {
        if (i > 0) s << ",";
        s << "#" << faceIds[i];
    }
    s << "));\n";

    // ---- Faceted BREP ----
    int brepId = nextId();
    s << "#" << brepId << "=FACETED_BREP('',#" << shellId << ");\n";

    // ---- Shape representation ----
    int shapeReprId = nextId();
    s << "#" << shapeReprId << "=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#"
      << brepId << "),#" << idGeomCtx << ");\n";

    // ---- Shape definition representation ----
    int sdrId = nextId();
    s << "#" << sdrId << "=SHAPE_DEFINITION_REPRESENTATION(#"
      << idProdDefShape << ",#" << shapeReprId << ");\n";

    s << "ENDSEC;\n";
    s << "END-ISO-10303-21;\n";

    return s.str();
}

} // namespace Odb::Lib::Converter
