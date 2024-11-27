//
// Modo mesh helper class
//

#include "mesh_helper.hxx"

#define MODE_M(c, s) ((LXtMarkMode) (((c) << 16) | s))

static double AreaTriangle(const LXtVector v0, const LXtVector v1, const LXtVector v2)
{
    LXtVector A, B;
    double    AB, AR;

    LXx_VSUB3(A, v1, v0);
    LXx_VSUB3(B, v2, v0);
    AB = LXx_VDOT(A, B);
    AR = LXx_VDOT(A, A) * LXx_VDOT(B, B) - AB * AB;
    if (AR > 0.0)
        return std::sqrt(AR) / 2;
    else
        return 0.0;
}

template <typename T>
static int VectorEqual(const T* a, const T* b, int n)
{
    while (n--)
        if (lx::Compare(a[n], b[n]))
            return 0;

    return 1;
}

class MarkEdgeVisitor : public CLxImpl_AbstractVisitor
{
public:
    LxResult Evaluate()
    {
        m_edge.SetMarks(MODE_M(m_context->m_mark_seam, 0));
        return LXe_OK;
    }

    CLxUser_Edge   m_edge;
    struct MeshHelper* m_context;
};

//
// Polygon visitor to add tringles of the polygon. This ignores non-surface polygons.
//
class TripleFaceVisitor : public CLxImpl_AbstractVisitor
{
public:
    LxResult Evaluate()
    {
        unsigned nvert;
        m_poly.VertexCount(&nvert);
        if (nvert < 3)
            return LXe_OK;

        m_poly.SetMarks(m_mark_done);

        LXtID4 type;
        m_poly.Type(&type);
        if ((type != LXiPTYP_FACE) && (type != LXiPTYP_PSUB) && (type != LXiPTYP_SUBD))
            return LXe_OK;

        m_context->AddPolygon(m_poly.ID());

        LXtPointID v0, v1, v2;
        if (nvert == 3)
        {
            m_poly.VertexByIndex(0, &v0);
            m_poly.VertexByIndex(1, &v1);
            m_poly.VertexByIndex(2, &v2);
            m_context->AddTriangle(m_poly.ID(), v0, v1, v2);
        }
        else
        {
            unsigned count;
            m_poly.GenerateTriangles(&count);
            for (auto i = 0u; i < count; i++)
            {
                m_poly.TriangleByIndex(i, &v0, &v1, &v2);
                m_context->AddTriangle(m_poly.ID(), v0, v1, v2);
            }
        }
        return LXe_OK;
    }

    CLxUser_Mesh    m_mesh;
    CLxUser_Polygon m_poly;
    CLxUser_Point   m_vert;
    CLxUser_MeshMap m_vmap;
    LXtMarkMode     m_mark_done;
    struct MeshHelper*  m_context;
};

//
// Polygon visitor to group connecting triangles as group.
//
class GroupFaceVisitor : public CLxImpl_AbstractVisitor
{
public:
    LxResult Evaluate()
    {
        unsigned nvert;
        m_poly.VertexCount(&nvert);
        if (nvert < 3)
            return LXe_OK;

        LXtID4 type;
        m_poly.Type(&type);
        if ((type != LXiPTYP_FACE) && (type != LXiPTYP_PSUB) && (type != LXiPTYP_SUBD))
            return LXe_OK;

        if (m_poly.TestMarks(m_mark_done) == LXe_TRUE)
            return LXe_OK;

        m_context->m_groups.push_back(std::make_shared<MeshHelper::Group>());
        MeshHelper::GroupID grp = m_context->m_groups.back();
        grp->index              = static_cast<unsigned>(m_context->m_groups.size() - 1);

        CLxUser_Polygon poly, poly1;
        poly.fromMesh(m_mesh);
        poly1.fromMesh(m_mesh);
        CLxUser_Edge edge;
        edge.fromMesh(m_mesh);

        std::vector<LXtPolygonID> stack;
        LXtPolygonID              pol = m_poly.ID();
        stack.push_back(pol);
        m_poly.SetMarks(m_context->m_mark_done);

        LXtVector axis, norm;
        LXx_VCLR(axis);

        while (!stack.empty())
        {
            pol = stack.back();
            stack.pop_back();
            poly.Select(pol);
            poly.Normal(norm);
            LXx_VADD(axis, norm);
            MeshHelper::Face& face = m_context->m_faces[pol];
            face.index             = static_cast<unsigned>(m_context->m_faces.size() - 1);
            face.group             = grp->index;
            for(auto& tri : face.tris)
            {
                tri->group = grp->index;
                grp->tris.push_back(tri);
                grp->area3D += tri->area3D;
            }
            unsigned int nvert = 0u, npol = 0u;
            poly.VertexCount(&nvert);
            for (auto i = 0u; i < nvert; i++)
            {
                LXtPointID v0{}, v1{};
                poly.VertexByIndex(i, &v0);
                poly.VertexByIndex((i + 1) % nvert, &v1);
                edge.SelectEndpoints(v0, v1);
                edge.PolygonCount(&npol);
                if (m_context->IsSeamEdge(edge))
                {
                    continue;
                }
                for (auto j = 0u; j < npol; j++)
                {
                    LXtPolygonID pol1;
                    edge.PolygonByIndex(j, &pol1);
                    poly1.Select(pol1);
                    if (poly1.TestMarks(m_mark_done) == LXe_TRUE)
                        continue;
                    poly1.SetMarks(m_mark_done);
                    if (poly1.TestMarks(m_context->m_mark_hide) == LXe_TRUE)
                        continue;
                    if (m_context->m_type == LXiSEL_POLYGON)
                    {
                        if (m_context->m_pick && poly1.TestMarks(m_context->m_pick) == LXe_FALSE)
                            continue;
                    }
                    stack.push_back(pol1);
                }
            }
        }

        return LXe_OK;
    }

    CLxUser_Mesh    m_mesh;
    CLxUser_Polygon m_poly;
    CLxUser_Point   m_vert;
    CLxUser_MeshMap m_vmap;
    LXtMarkMode     m_mark_done;
    struct MeshHelper*  m_context;
};

//
// Add polygon into face table.
//
LxResult MeshHelper::AddPolygon(LXtPolygonID pol)
{
    Face face;
    m_faces[pol] = face;
    return LXe_OK;
}

//
// Return edge with v0 and v1. 
//
MeshHelper::EdgeID MeshHelper::GetEdge(VerxID v0, VerxID v1)
{
    std::pair<VerxID,VerxID> key = std::make_pair(v0,v1);
    auto it = m_edgemap.find(key);
    if (it != m_edgemap.end())
        return it->second;

    key = std::make_pair(v1,v0);
    it = m_edgemap.find(key);
    if (it != m_edgemap.end())
        return it->second;
        
    return nullptr;
}

//
// Add new edge with v0 and v1. 
//
MeshHelper::EdgeID MeshHelper::AddEdge(VerxID v0, VerxID v1, TriangleID tri)
{
    auto edge = GetEdge(v0, v1);
    if (!edge)
    {
        m_edges.push_back(std::make_shared<Edge>());
        edge = m_edges.back();
        edge->index = m_edges.size() - 1;
        edge->vrts[0] = v0;
        edge->vrts[1] = v1;
        auto key = std::make_pair(v1,v0);
        m_edgemap.insert(std::make_pair(key,edge));
    }
    edge->tris.insert(tri);
    return edge;
}

//
// Add new triangle for the polygon.
//
LxResult MeshHelper::AddTriangle(LXtPolygonID pol, LXtPointID v0, LXtPointID v1, LXtPointID v2)
{
    m_triangles.push_back(std::make_shared<Triangle>());
    TriangleID tri = m_triangles.back();

    int index;
    unsigned index0, index1, index2;
    m_poly.Select(pol);
    m_poly.Index(&index);
    m_vert.Select(v0);
    m_vert.Index(&index0);
    m_vert.Select(v1);
    m_vert.Index(&index1);
    m_vert.Select(v2);
    m_vert.Index(&index2);

    VerxID vrts[3];
    vrts[0] = AddVertex(v0, pol, tri);
    vrts[1] = AddVertex(v1, pol, tri);
    vrts[2] = AddVertex(v2, pol, tri);

    Face& face = m_faces[pol];
    face.tris.push_back(tri);

    tri->index   = m_triangles.size() - 1;
    tri->area3D  = AreaTriangle(vrts[0]->pos, vrts[1]->pos, vrts[2]->pos);
    tri->vrts[0] = vrts[0];
    tri->vrts[1] = vrts[1];
    tri->vrts[2] = vrts[2];
    tri->pol     = pol;

    AddEdge(vrts[0], vrts[1], tri);
    AddEdge(vrts[1], vrts[2], tri);
    AddEdge(vrts[2], vrts[0], tri);
    return LXe_OK;
}

LXtPolygonID MeshHelper::TracePolygon(LXtPointID vrt, LXtPolygonID pol, int shift)
{
    CLxUser_Edge edge;
    edge.fromMesh(m_mesh);

    CLxUser_Polygon poly;
    poly.fromMesh(m_mesh);

    LXtPolygonID pol1;
    LXtPointID   vrt1;
    unsigned int npol, nvert;

    m_poly.Select(pol);
    m_poly.VertexCount(&nvert);

    for (auto i = 0u; i < nvert; i++)
    {
        m_poly.VertexByIndex(i, &vrt1);
        if (vrt1 == vrt)
        {
            m_poly.VertexByIndex((i + shift + nvert) % nvert, &vrt1);
            edge.SelectEndpoints(vrt, vrt1);
            if (IsSeamEdge(edge))
                return nullptr;
            edge.PolygonCount(&npol);
            for (auto j = 0u; j < npol; j++)
            {
                edge.PolygonByIndex(j, &pol1);
                poly.Select(pol1);
                if (poly.TestMarks(m_mark_hide) == LXe_TRUE)
                    continue;
                if (pol1 != pol)
                {
                    return pol1;
                }
            }
            return nullptr;
        }
    }
    return nullptr;
}

//
// Find a tringle with the given 'pnt' and 'polygon'.
//
MeshHelper::TriangleID MeshHelper::FetchTriangle(LXtPointID pnt, LXtPolygonID pol)
{
    LXtPolygonID pol0 = pol;
    unsigned int npol;

    m_vert.Select(pnt);
    m_vert.PolygonCount(&npol);

    if (m_faces.find(pol) != m_faces.end())
    {
        Face& face = m_faces[pol];
        for (auto& tri : face.tris)
        {
            for (auto& vrt : tri->vrts)
                if (vrt->pnt == pnt)
                    return tri;
        }
    }
    npol--;

    pol = pol0;
    while (npol > 0)
    {
        pol = TracePolygon(pnt, pol, +1);
        if (!pol)
            break;
        if (pol == pol0)
            break;
        if (m_faces.find(pol) != m_faces.end())
        {
            Face& face = m_faces[pol];
            for (auto& tri : face.tris)
            {
                for (auto& vrt : tri->vrts)
                    if (vrt->pnt == pnt)
                        return tri;
            }
        }
        npol--;
    }

    pol = pol0;
    while (npol > 0)
    {
        pol = TracePolygon(pnt, pol, -1);
        if (!pol)
            break;
        if (pol == pol0)
            break;
        if (m_faces.find(pol) != m_faces.end())
        {
            Face& face = m_faces[pol];
            for (auto& tri : face.tris)
            {
                for (auto& vrt : tri->vrts)
                    if (vrt->pnt == pnt)
                        return tri;
            }
        }
        npol--;
    }

    return nullptr;
}

//
// Add vertex into vertices vector table. If it is already added, it sets the given triangle
// into the trinagle list of the vertex.
//
MeshHelper::VerxID MeshHelper::AddVertex(LXtPointID pnt, LXtPolygonID pol, TriangleID tri)
{
    TriangleID ref = FetchTriangle(pnt, pol);
    if (ref)
    {
        for (auto& vrt : ref->vrts)
        {
            if (vrt->pnt == pnt)
            {
                auto it = std::find(vrt->tris.begin(), vrt->tris.end(), tri);
                if (it == vrt->tris.end())
                    vrt->tris.insert(tri);
                return vrt;
            }
        }
    }

    m_vertices.push_back(std::make_shared<Verx>());
    VerxID vrt = m_vertices.back();

    vrt->pnt   = pnt;
    vrt->tris.insert(tri);
    vrt->index = static_cast<unsigned>(m_vertices.size()-1);
    vrt->marks = LXiMARK_ANY;
    vrt->w     = 1.0;
    m_vert.Select(pnt);
    LXtFVector fv;
    m_vert.Pos(fv);
    LXx_VCPY(vrt->pos, fv);
    unsigned index;
    m_vert.Index(&index);
    int index1;
    m_poly.Select(pol);
    m_poly.Index(&index1);
    return vrt;
}

//
// Return true if the given edge is on boundary. If mark_seam is set,
// it also returns marked edge as seam edge.
//
bool MeshHelper::IsSeamEdge(CLxUser_Edge& edge)
{
    if (m_mark_seam)
    {
        if (edge.TestMarks(m_mark_seam) == LXe_TRUE)
            return true;
    }

    unsigned int npol;
    edge.PolygonCount(&npol);
    // edge is boundary
    if (npol == 1)
    {
        edge.SetMarks(m_mark_seam);
        return true;
    }

    return false;
}

//
// Apply the current positions to the given edit mesh.
//
LxResult MeshHelper::Apply(CLxUser_Mesh& edit_mesh)
{
    CLxUser_Point   vert;
    vert.fromMesh(edit_mesh);
    // Apply computed UVs into UV vertex map.
    for (auto& vrt : m_vertices)
    {
        vert.Select(vrt->pnt);
        vert.SetPos(vrt->pos);
    }
    return LXe_OK;
}


//
// Set face indices and position vectors to Eigen matrices.
//
LxResult MeshHelper::EgenMatrix(GroupID grp, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    unsigned n = 0;

    // V : 3d vertex positions
    V.resize(grp->vrts.size(), 3);
    n = 0;
    for(auto& vrt : grp->vrts)
    {
        V(n, 0) = vrt->pos[0];
        V(n, 1) = vrt->pos[1];
        V(n, 2) = vrt->pos[2];
        n ++;
    }
    // F : vertex indices of triangles
    F.resize(grp->tris.size(), 3);
    n = 0;
    if (grp->flipped)
    {
        for(auto& tri : grp->tris)
        {
            F(n, 0) = tri->vrts[0]->index;
            F(n, 1) = tri->vrts[2]->index;
            F(n, 2) = tri->vrts[1]->index;
            n ++;
        }
    }
    else
    {
        for(auto& tri : grp->tris)
        {
            F(n, 0) = tri->vrts[0]->index;
            F(n, 1) = tri->vrts[1]->index;
            F(n, 2) = tri->vrts[2]->index;
            n ++;
        }
    }
 //   printf("Eigen V (%zu) F (%zu))\n", V.size(), F.size());
 //   printf("Eigen F cols(%td) rows (%td) V cols(%td) rows (%td)\n", F.cols(), F.rows(), V.cols(), V.rows());
    return LXe_OK;
}


//
// Set the position vectors given by Eigen matrix into group vertices.
//
LxResult MeshHelper::SetResult(GroupID grp, Eigen::MatrixXd& V_o)
{
    if (V_o.size() != (grp->vrts.size() * 3))
        return LXe_FAILED;

    LXtVector pos;
    auto n = 0u;
    for(auto& vrt : grp->vrts)
    {
        pos[0] = V_o(n, 0);
        pos[1] = V_o(n, 1);
        pos[2] = V_o(n, 2);
        LXx_VLERP(vrt->pos, vrt->pos, pos, vrt->w);
        n ++;
    }
    return LXe_OK;
}

//
// Resize the position vectors to fit the group bounding box. 
//
LxResult MeshHelper::FitResult(GroupID grp, Eigen::MatrixXd& V_o)
{
    if (V_o.size() != (grp->vrts.size() * 3))
        return LXe_FAILED;

    LXtVector center, pos;

    auto n = 0u;
    CLxBoundingBox box3D;
    for(auto& vrt : grp->vrts)
    {
        pos[0] = V_o(n, 0);
        pos[1] = V_o(n, 1);
        pos[2] = V_o(n, 2);
        box3D.add(pos);
        n ++;
    }
    double radius = 0.0;
    box3D.center(center);
    n = 0u;
    for(auto& vrt : grp->vrts)
    {
        pos[0] = V_o(n, 0);
        pos[1] = V_o(n, 1);
        pos[2] = V_o(n, 2);
        double r = LXx_VDIST(center, pos);
        if (r > radius)
            radius = r;
        n ++;
    }
    double s = grp->radius / radius;
    n = 0u;
    for(auto& vrt : grp->vrts)
    {
        pos[0] = V_o(n, 0);
        pos[1] = V_o(n, 1);
        pos[2] = V_o(n, 2);
        LXx_VSUB(pos, center);
        LXx_VSCL(pos, s);
        LXx_VADD(pos, center);
        V_o(n, 0) = pos[0];
        V_o(n, 1) = pos[1];
        V_o(n, 2) = pos[2];
        n ++;
    }
    return LXe_OK;
}


MeshHelper::MeshHelper(CLxUser_Mesh& mesh)
{
    m_mesh.set(mesh);
    m_flags     = 0u;

    m_poly.fromMesh(m_mesh);
    m_vert.fromMesh(m_mesh);

    CLxUser_MeshService mesh_svc;
    m_pick      = mesh_svc.SetMode(LXsMARK_SELECT);
    m_mark_done = mesh_svc.SetMode(LXsMARK_USER_0);
    m_mark_seam = mesh_svc.SetMode(LXsMARK_USER_1);
    m_mark_hide = mesh_svc.SetMode(LXsMARK_HIDE);
    m_mark_lock = mesh_svc.SetMode(LXsMARK_LOCK);
    m_type  = LXiSEL_POLYGON;

    // generate triangles from surface polygons
    TripleFaceVisitor triFace;
    triFace.m_mesh = m_mesh;
    triFace.m_poly.fromMesh(m_mesh);
    triFace.m_vert.fromMesh(m_mesh);
    triFace.m_mark_done = mesh_svc.ClearMode(LXsMARK_USER_0);
    triFace.m_context = this;
    triFace.m_poly.Enum(&triFace, LXiMARK_ANY);

    // divides polygons by seam edges in groups.
    GroupFaceVisitor grpFace;
    grpFace.m_mesh = m_mesh;
    grpFace.m_vmap = m_vmap;
    grpFace.m_poly.fromMesh(m_mesh);
    grpFace.m_vert.fromMesh(m_mesh);
    grpFace.m_mark_done = mesh_svc.SetMode(LXsMARK_USER_0);
    grpFace.m_context = this;
    grpFace.m_poly.Enum(&grpFace, LXiMARK_ANY);

    // finalize group info
    for (auto& vrt : m_vertices)
    {
        auto it = vrt->tris.begin();
        auto& tri = *it;
        auto& grp = m_groups[tri->group];
        vrt->index = grp->vrts.size();
        grp->box3D.add(vrt->pos);
        grp->vrts.push_back(vrt);
    }
    for (auto& grp : m_groups)
    {
        LXtVector center;
        grp->box3D.center(center);
        grp->radius = 0.0;
        for (auto& vrt : grp->vrts)
        {
            double r = LXx_VDIST(center, vrt->pos);
            if (r > grp->radius)
                grp->radius = r;
        }
    }
#if 0
    printf("MeshHelper: groups (%zu) faces (%zu) triangles (%zu) edges (%zu) vertices (%zu)\n",
        m_groups.size(), m_faces.size(), m_triangles.size(), m_edges.size(), m_vertices.size());
    for(auto& grp : m_groups)
        printf("group [%u] area3D %f tris (%zu) vrts (%zu)\n", grp->index, grp->area3D, grp->tris.size(), grp->vrts.size());
    for(auto& tri : m_triangles)
        printf("tri (%u) v0 (%u) v1 (%u) v2 (%u) group (%u)\n", tri->index, 
            tri->vrts[0]->index, tri->vrts[1]->index, tri->vrts[2]->index, tri->group);
    for(auto& edge : m_edges)
        printf("edge (%u) v0 (%u) v1 (%u) tris (%zu)\n", edge->index, 
            edge->vrts[0]->index, edge->vrts[1]->index, edge->tris.size());
    for(auto& vrt : m_vertices)
        printf("vrt (%u) pos %f %f %f tris (%zu)\n", vrt->index, 
            vrt->pos[0], vrt->pos[1], vrt->pos[2], vrt->tris.size());
#endif
}




