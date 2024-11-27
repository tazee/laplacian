/*
 * TOOL.CXX  The Laplacian smoothing tool. 
 */
#include <lxsdk/lx_plugin.hpp>

#include <lxsdk/lx_layer.hpp>
#include <lxsdk/lx_log.hpp>
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_tool.hpp>
#include <lxsdk/lx_toolui.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lx_vector.hpp>
#include <lxsdk/lx_vmodel.hpp>
#include <lxsdk/lxu_attributes.hpp>
#include <lxsdk/lx_select.hpp>
#include <lxsdk/lx_seltypes.hpp>
#include <string>

#include "mesh_helper.hxx"

#include <igl/barycenter.h>
#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/grad.h>
#include <igl/jet.h>
#include <igl/massmatrix.h>
#include <igl/per_vertex_normals.h>

/*
 * The Laplacian smoothing tool. Basic tool and tool model methods are defined here. The
 * attributes interface is inherited from the utility class.
 */

class CTool : public CLxImpl_Tool, public CLxImpl_ToolModel, public CLxDynamicAttributes
{
public:
    CTool();

    void        tool_Reset() override;
    LXtObjectID tool_VectorType() override;
    const char* tool_Order() override;
    LXtID4      tool_Task() override;
    void        tool_Evaluate(ILxUnknownID vts) override;

    unsigned    tmod_Flags() override;
    void        tmod_Initialize(ILxUnknownID vts, ILxUnknownID adjust, unsigned flags) override;
    LxResult    tmod_Enable(ILxUnknownID obj) override;
    LxResult    tmod_Down(ILxUnknownID vts, ILxUnknownID adjust) override;
    void        tmod_Move(ILxUnknownID vts, ILxUnknownID adjust) override;

    using CLxDynamicAttributes::atrui_UIHints;  // to distinguish from the overloaded version in CLxImpl_AttributesUI

    void atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints) override;

    bool TestPolygon();

    bool LaplacianSmoothing(CLxUser_Mesh& base_mesh, CLxUser_Mesh& edit_mesh, int iter);

    CLxUser_LogService   s_log;
    CLxUser_LayerService s_layer;
    CLxUser_VectorType   v_type;
    CLxUser_SelectionService s_sel;
    CLxUser_FalloffPacket falloff;

    unsigned offset_view;
    unsigned offset_screen;
    unsigned offset_falloff;
    unsigned mode_select;

    int m_iter0;
};

#define ATTRs_ITERATION "iteration"

#define ATTRa_ITERATION   0

/*
 * On create we add our one tool attribute. We also allocate a vector type
 * and select mode mask.
 */
CTool::CTool()
{
    CLxUser_PacketService sPkt;
    CLxUser_MeshService   sMesh;

    dyna_Add(ATTRs_ITERATION, LXsTYPE_INTEGER);

    tool_Reset();

    sPkt.NewVectorType(LXsCATEGORY_TOOL, v_type);
    sPkt.AddPacket(v_type, LXsP_TOOL_VIEW_EVENT, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_SCREEN_EVENT, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_FALLOFF, LXfVT_GET);

    offset_view = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_VIEW_EVENT);
    offset_screen = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SCREEN_EVENT);
    offset_falloff = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_FALLOFF);
    mode_select = sMesh.SetMode("select");
}

/*
 * Reset sets the attributes back to defaults.
 */
void CTool::tool_Reset()
{
    dyna_Value(ATTRa_ITERATION).SetInt(0);
}

/*
 * Boilerplate methods that identify this as an action (state altering) tool.
 */
LXtObjectID CTool::tool_VectorType()
{
    return v_type.m_loc;  // peek method; does not add-ref
}

const char* CTool::tool_Order()
{
    return LXs_ORD_ACTR;
}

LXtID4 CTool::tool_Task()
{
    return LXi_TASK_ACTR;
}

/*
 * We employ the simplest possible tool model -- default hauling. We indicate
 * that we want to haul one attribute, we name the attribute, and we implement
 * Initialize() which is what to do when the tool activates or re-activates.
 * In this case set the axis to the current value.
 */
unsigned CTool::tmod_Flags()
{
    return LXfTMOD_I0_INPUT;
}

void CTool::tmod_Initialize(ILxUnknownID /*vts*/, ILxUnknownID adjust, unsigned int /*flags*/)
{
}

LxResult CTool::tmod_Enable(ILxUnknownID obj)
{
    CLxUser_Message msg(obj);

    if (TestPolygon() == false)
    {
        msg.SetCode(LXe_CMD_DISABLED);
        msg.SetMessage("mesh.seashell", "NoPolygon", 0);
        return LXe_DISABLED;
    }
    return LXe_OK;
}

LxResult CTool::tmod_Down(ILxUnknownID vts, ILxUnknownID adjust)
{
    dyna_Value(ATTRa_ITERATION).GetInt(&m_iter0);
    return LXe_TRUE;
}

void CTool::tmod_Move(ILxUnknownID vts, ILxUnknownID adjust)
{
    CLxUser_AdjustTool at(adjust);
    CLxUser_VectorStack vec(vts);
    LXpToolScreenEvent* spak = static_cast<LXpToolScreenEvent*>(vec.Read(offset_screen));

    int iter = m_iter0 + (spak->cx - spak->px) / 10;
    if (iter < 0)
        iter = 0;
    at.SetInt(ATTRa_ITERATION, iter);
}

void CTool::atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints)
{
    switch (index)
    {
        case ATTRa_ITERATION:
            hints.MinInt(0);
            break;
    }
}

bool CTool::TestPolygon()
{
    /*
     * Start the scan in read-only mode.
     */
    CLxUser_LayerScan scan;
    CLxUser_Mesh      mesh;
    unsigned          i, n, count;
    bool              ok = false;

    s_layer.BeginScan(LXf_LAYERSCAN_ACTIVE | LXf_LAYERSCAN_MARKPOLYS, scan);

    /*
     * Count the polygons in all mesh layers.
     */
    if (scan)
    {
        n = scan.NumLayers();
        for (i = 0; i < n; i++)
        {
            scan.BaseMeshByIndex(i, mesh);
            mesh.PolygonCount(&count);
            if (count > 0)
            {
                ok = true;
                break;
            }
        }
        scan.Apply();
    }

    /*
     * Return false if there is no polygons in any active layers.
     */
    return ok;
}

/*
 * Tool evaluation uses layer scan interface to walk through all the active
 * meshes and visit all the selected polygons.
 */
void CTool::tool_Evaluate(ILxUnknownID vts)
{
    CLxUser_VectorStack vec(vts);
    LXpToolViewEvent*   view;

    view = (LXpToolViewEvent*) vec.Read(offset_view);
    if (!view || view->type != LXi_VIEWTYPE_3D)
        return;

    LXtID4  vertex_type = s_sel.LookupType(LXsSELTYP_VERTEX);
    LXtID4  edge_type   = s_sel.LookupType(LXsSELTYP_EDGE);
    LXtID4  poly_type   = s_sel.LookupType(LXsSELTYP_POLYGON);
    LXtID4  item_type   = s_sel.LookupType(LXsSELTYP_ITEM);

    LXtID4  currentTypes[5];
    currentTypes[0] = vertex_type;
    currentTypes[1] = edge_type;
    currentTypes[2] = poly_type;
    currentTypes[3] = item_type;
    currentTypes[4] = 0;

    LXtID4 cur = s_sel.CurrentType(currentTypes);

    /*
     * Start the scan in edit-poly mode.
     */
    CLxUser_LayerScan  scan;
    CLxUser_Mesh       base_mesh, edit_mesh;

    s_layer.BeginScan(LXf_LAYERSCAN_EDIT_POLVRT, scan);

    int      ival, iter;

    dyna_Value(ATTRa_ITERATION).GetInt(&iter);
    if (iter == 0)
        return;

    CLxUser_VectorStack vectorStack(vts);
    vectorStack.ReadObject(offset_falloff, falloff);

    auto n = scan.NumLayers();
    for (auto i = 0u; i < n; i++)
    {
        scan.BaseMeshByIndex(i, base_mesh);
        scan.EditMeshByIndex(i, edit_mesh);

        // Laplacian Smoothing
        LaplacianSmoothing(base_mesh, edit_mesh, iter);

        scan.SetMeshChange(i, LXf_MESHEDIT_POSITION);
    }

    scan.Apply();
}


bool CTool::LaplacianSmoothing(CLxUser_Mesh& base_mesh, CLxUser_Mesh& edit_mesh, int iter)
{
    using namespace Eigen;

    MeshHelper helper(base_mesh);

    MatrixXd V, U;
    MatrixXi F;
    SparseMatrix<double> L;

    for (auto& grp : helper.m_groups)
    {
        helper.EgenMatrix(grp, V, F);

        // Compute Laplace-Beltrami operator: #V by #V
        igl::cotmatrix(V,F,L);
        // Alternative construction of same Laplacian
        SparseMatrix<double> G,K;
        // Gradient/Divergence
        igl::grad(V,F,G);
        // Diagonal per-triangle "mass matrix"
        VectorXd dblA;
        igl::doublearea(V,F,dblA);
        // Place areas along diagonal #dim times
        const auto & T = 1.*(dblA.replicate(3,1)*0.5).asDiagonal();
        // Laplacian K built as discrete divergence of gradient or equivalently
        // discrete Dirichelet energy Hessian
        K = -G.transpose() * T * G;
        U = V;

        // set falloff weights
        for (auto& vrt : grp->vrts)
        {
            LXtFVector fv;
            LXx_VCPY(fv, vrt->pos);
            vrt->w = falloff.Evaluate(fv, vrt->pnt, nullptr);
        }

        for (auto i = 0; i < iter; i++)
        {
            // Recompute just mass matrix on each step
            SparseMatrix<double> M;
            igl::massmatrix(U,F,igl::MASSMATRIX_TYPE_BARYCENTRIC,M);
            // Solve (M-delta*L) U = M*U
            const auto & S = (M - 0.001*L);
            Eigen::SimplicialLLT<Eigen::SparseMatrix<double > > solver(S);
            if (solver.info() != Eigen::Success)
                break;
            U = solver.solve(M*U).eval();

            helper.FitResult(grp, U);
        }

        helper.SetResult(grp, U);
    }
    helper.Apply(edit_mesh);
    return true;
}

/*
 * Export tool server.
 */
void initialize()
{
    CLxGenericPolymorph* srv;

    srv = new CLxPolymorph<CTool>;
    srv->AddInterface(new CLxIfc_Tool<CTool>);
    srv->AddInterface(new CLxIfc_ToolModel<CTool>);
    srv->AddInterface(new CLxIfc_Attributes<CTool>);
    srv->AddInterface(new CLxIfc_AttributesUI<CTool>);
    thisModule.AddServer("xfrm.laplacian", srv);
}
