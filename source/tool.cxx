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

#include <igl/cotmatrix.h>
#include <igl/grad.h>
#include <igl/massmatrix.h>

#ifndef LXx_OVERRIDE
#define LXx_OVERRIDE override
#endif

/*
 * The Tool Operation is evaluated by the procedural modeling system.
 */
class CToolOp : public CLxImpl_ToolOperation
{
	public:
		LxResult    top_Evaluate(ILxUnknownID vts)  LXx_OVERRIDE;

        bool LaplacianSmoothing(CLxUser_Mesh& base_mesh, CLxUser_Mesh& edit_mesh);

        CLxUser_FalloffPacket falloff;
        CLxUser_Subject2Packet subject;

        unsigned offset_view;
        unsigned offset_screen;
        unsigned offset_falloff;
        unsigned offset_subject;

        int m_iter;
};

/*
 * The Laplacian smoothing tool. Basic tool and tool model methods are defined here. The
 * attributes interface is inherited from the utility class.
 */

class CTool : public CLxImpl_Tool, public CLxImpl_ToolModel, public CLxDynamicAttributes
{
public:
    CTool();

    void        tool_Reset() LXx_OVERRIDE;
    LXtObjectID tool_VectorType() LXx_OVERRIDE;
    const char* tool_Order() LXx_OVERRIDE;
    LXtID4      tool_Task() LXx_OVERRIDE;
	LxResult	tool_GetOp(void **ppvObj, unsigned flags) LXx_OVERRIDE;

    unsigned    tmod_Flags() LXx_OVERRIDE;
    LxResult    tmod_Enable(ILxUnknownID obj) LXx_OVERRIDE;
    LxResult    tmod_Down(ILxUnknownID vts, ILxUnknownID adjust) LXx_OVERRIDE;
    void        tmod_Move(ILxUnknownID vts, ILxUnknownID adjust) LXx_OVERRIDE;

    using CLxDynamicAttributes::atrui_UIHints;  // to distinguish from the overloaded version in CLxImpl_AttributesUI

    void atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints) LXx_OVERRIDE;

	static LXtTagInfoDesc	 descInfo[];

    bool TestPolygon();

    CLxUser_LogService   s_log;
    CLxUser_LayerService s_layer;
    CLxUser_VectorType   v_type;
    CLxUser_SelectionService s_sel;

    unsigned offset_view;
    unsigned offset_screen;
    unsigned offset_falloff;
    unsigned offset_subject;
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
    sPkt.AddPacket(v_type, LXsP_TOOL_SUBJECT2, LXfVT_GET);

    offset_view = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_VIEW_EVENT);
    offset_screen = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SCREEN_EVENT);
    offset_falloff = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_FALLOFF);
    offset_subject = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SUBJECT2);
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

LxResult CTool::tool_GetOp(void** ppvObj, unsigned flags)
{
    CLxSpawner<CToolOp> spawner("xfrm.laplacian.toolop");
    CToolOp*            toolop = spawner.Alloc(ppvObj);

	if (!toolop)
	{
		return LXe_FAILED;
	}

    dyna_Value(ATTRa_ITERATION).GetInt(&toolop->m_iter);

    toolop->offset_view = offset_view;
    toolop->offset_screen = offset_screen;
    toolop->offset_falloff = offset_falloff;
    toolop->offset_subject = offset_subject;

	return LXe_OK;
}

LXtTagInfoDesc CTool::descInfo[] =
{
	{LXsTOOL_PMODEL, "."},
	{LXsTOOL_USETOOLOP, "."},
	{LXsPMODEL_SELECTIONTYPES, LXsSELOP_TYPE_VERTEX},
	{0}

};

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
LxResult CToolOp::top_Evaluate(ILxUnknownID vts)
{
    if (m_iter == 0)
        return LXe_OK;

    CLxUser_VectorStack vec(vts);

    /*
     * Start the scan in edit mode.
     */
    CLxUser_LayerScan  scan;
    CLxUser_Mesh       base_mesh, edit_mesh;

    if (vec.ReadObject(offset_subject, subject) == false)
        return LXe_FAILED;
    if (vec.ReadObject(offset_falloff, falloff) == false)
        return LXe_FAILED;

    subject.BeginScan(LXf_LAYERSCAN_EDIT_VERTS, scan);

    auto n = scan.NumLayers();
    for (auto i = 0u; i < n; i++)
    {
        scan.BaseMeshByIndex(i, base_mesh);
        scan.EditMeshByIndex(i, edit_mesh);

        // Laplacian Smoothing
        LaplacianSmoothing(base_mesh, edit_mesh);

        scan.SetMeshChange(i, LXf_MESHEDIT_POSITION);
    }

    scan.Apply();
    return LXe_OK;
}


bool CToolOp::LaplacianSmoothing(CLxUser_Mesh& base_mesh, CLxUser_Mesh& edit_mesh)
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

        for (auto i = 0; i < m_iter; i++)
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
    srv->AddInterface(new CLxIfc_StaticDesc<CTool>);
    thisModule.AddServer("xfrm.laplacian", srv);

    srv = new CLxPolymorph<CToolOp>;
    srv->AddInterface(new CLxIfc_ToolOperation<CToolOp>);
    lx::AddSpawner("xfrm.laplacian.toolop", srv);
}
