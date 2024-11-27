//
// Modo mesh helper class
// Generate triangle list and vertex positions from Modo mesh per
// connecting polygon group.
//
#pragma once

#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_select.hpp>
#include <lxsdk/lx_seltypes.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lx_log.hpp>

#include <Eigen/Dense>

#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>

//
// trick to add std::unordered_map<std::pair<VerxID,VerxID>, EdgeID>
//
template<class T> size_t HashCombine(const size_t seed,const T &v){
    return seed^(std::hash<T>()(v)+0x9e3779b9+(seed<<6)+(seed>>2));
}
template<class T,class S> struct std::hash<std::pair<T,S>>{
    size_t operator()(const std::pair<T,S> &keyval) const noexcept {
        return HashCombine(std::hash<T>()(keyval.first), keyval.second);
    }
};

struct MeshHelper
{
    struct Verx;
    struct Edge;
    struct Triangle;
    struct Group;

    typedef std::shared_ptr<Verx>     VerxID;
    typedef std::shared_ptr<Edge>     EdgeID;
    typedef std::shared_ptr<Triangle> TriangleID;
    typedef std::shared_ptr<Group>    GroupID;

    struct Verx
    {
        LXtPointID                  pnt;
        unsigned                    index;      // index in group
        LXtMarkMode                 marks;      // marks for working
        LXtVector                   pos;        // vertex corner position
        double                      w;          // falloff weight
        std::unordered_set<TriangleID> tris;       // connecting triangles
    };

    struct Edge
    {
        unsigned                    index;   // index in group
        LXtMarkMode                 marks;   // marks for working
        std::array<VerxID,2>        vrts;    // v0, v1
        std::unordered_set<TriangleID> tris;    // connecting triangles
    };

    struct Triangle
    {
        LXtPolygonID                pol;
        unsigned                    index;   // index
        LXtMarkMode                 marks;   // marks for working
        std::array<VerxID,3>        vrts;    // v0, v1, v2
        unsigned                    group;   // group index
        double                      area3D;
    };

    struct Face
    {
        unsigned                    index;
        unsigned                    group;  // group index
        std::vector<TriangleID>     tris;   // triangles of the face
    };

    struct Group
    {
        unsigned                    index;
        CLxBoundingBox              box3D;
        double                      area3D;
        double                      radius;
        bool                        flipped;
        std::vector<TriangleID>     tris;   // connecting triangles
        std::vector<VerxID>         vrts;   // vertex of the triangles
    };

    std::vector<GroupID>    m_groups;
    std::vector<EdgeID>     m_edges;
    std::vector<VerxID>     m_vertices;
    std::vector<TriangleID> m_triangles;

    std::unordered_map<LXtPolygonID, Face> m_faces;
    std::unordered_map<std::pair<VerxID,VerxID>, EdgeID> m_edgemap;

    LXtMatrix               mtx; /* mesh item transform */

    CLxUser_Mesh    m_mesh;
    CLxUser_MeshMap m_vmap;
    CLxUser_Polygon m_poly;
    CLxUser_Point   m_vert;
    unsigned int    m_flags;
    LXtID4          m_type;

    LXtMarkMode m_pick;
    LXtMarkMode m_mark_done;
    LXtMarkMode m_mark_seam;
    LXtMarkMode m_mark_hide;
    LXtMarkMode m_mark_lock;

    CLxUser_LogService   s_log;

    MeshHelper(CLxUser_Mesh& mesh);

    void Clear()
    {
        m_vertices.clear();
        m_edges.clear();
        m_triangles.clear();
        m_faces.clear();
        m_groups.clear();
        m_edgemap.clear();
    }

    // V : 3d vertex positions
    // F : vertex indices of triangles
    // V_o : output vertex positions
    LxResult EgenMatrix(GroupID grp, Eigen::MatrixXd& V, Eigen::MatrixXi& F);
    LxResult SetResult(GroupID grp, Eigen::MatrixXd& V_o);
    LxResult FitResult(GroupID grp, Eigen::MatrixXd& V_o);

    LxResult Apply(CLxUser_Mesh& edit_mesh);

    LxResult AddTriangle(LXtPolygonID pol, LXtPointID v0, LXtPointID v1, LXtPointID v2);
    LxResult AddPolygon(LXtPolygonID pol);
    VerxID   AddVertex(LXtPointID vrt, LXtPolygonID pol, TriangleID tri);
    EdgeID   AddEdge(VerxID v0, VerxID v1, TriangleID tri);
    EdgeID   GetEdge(VerxID v0, VerxID v1);
    bool     IsSeamEdge(CLxUser_Edge& edge);
    LXtPolygonID TracePolygon(LXtPointID vrt, LXtPolygonID pol, int shift);
    TriangleID  FetchTriangle(LXtPointID pnt, LXtPolygonID pol);
};
