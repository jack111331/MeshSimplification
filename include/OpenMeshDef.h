//
// Created by Edge on 2021/11/6.
//

#ifndef RAY_OPENMESHDEF_H
#define RAY_OPENMESHDEF_H


#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Utils/getopt.h>
#include <wx/log.h>

struct MyTraits : OpenMesh::DefaultTraits
{
    // let Point and Normal be a vector made from doubles
    typedef OpenMesh::Vec3d Point;
    typedef OpenMesh::Vec3d Normal;

    // add normal property to vertices and faces
    VertexAttributes(OpenMesh::Attributes::Normal);
    FaceAttributes  (OpenMesh::Attributes::Normal);

    // Already defined in OpenMesh::DefaultTraits
    // HalfedgeAttributes( OpenMesh::Attributes::PrevHalfedge );

    // Uncomment next line to disable attribute PrevHalfedge
    // HalfedgeAttributes( OpenMesh::Attributes::None );
    //
    // or
    //
    // HalfedgeAttributes( 0 );
};

namespace OMT//OpenMesh Triangle mesh
{
    using namespace std;
    /*----------------------------------------------------------------------*/
//
//    /*定義使用的精準度和基本屬性*/
//    struct MyTraits : OpenMesh::DefaultTraits
//    {
//        // let Point and Normal be a vector made from doubles
//        typedef OpenMesh::Vec3d Point;
//        typedef OpenMesh::Vec3d Normal;
//
//        // add normal property to vertices and faces
//        VertexAttributes(OpenMesh::Attributes::Normal);
//        FaceAttributes  (OpenMesh::Attributes::Normal);
//
//        // Already defined in OpenMesh::DefaultTraits
//        // HalfedgeAttributes( OpenMesh::Attributes::PrevHalfedge );
//
//        // Uncomment next line to disable attribute PrevHalfedge
//        // HalfedgeAttributes( OpenMesh::Attributes::None );
//        //
//        // or
//        //
//        // HalfedgeAttributes( 0 );
//    };
//    /*----------------------------------------------------------------------*/
//
    /*定義常用type*/
    typedef OpenMesh::TriMesh_ArrayKernelT <MyTraits> MyMesh;
    typedef OpenMesh::Vec3d Vector3d;    //Vec3D type
    typedef MyMesh::Scalar									Scalar	;	//Scalar type
    typedef MyMesh::Point									Point	;	//Point type
    typedef MyMesh::Normal									Normal	;	//Normal type
    typedef MyMesh::VertexHandle							VHandle	;	//VertexHandle type
    typedef MyMesh::HalfedgeHandle							HEHandle;	//HalfedgeHandle type
    typedef MyMesh::EdgeHandle							    EHandle ;	//edgeHandle type
    typedef MyMesh::FaceHandle								FHandle	;	//FaceHandle type
    //-------------Vertex iterators & circulators-------------
    typedef MyMesh::VertexIter								VIter	;	//VertexIter type
    typedef MyMesh::VertexVertexIter						VVIter	;	//VertexVertexIter type
    typedef MyMesh::VertexEdgeIter							VEIter	;	//VertexEdgeIter type
    typedef MyMesh::VertexFaceIter							VFIter	;	//VertexFaceIter type
    typedef MyMesh::EdgeIter								EIter	;	//EdgeIterT	type
    typedef MyMesh::FaceIter								FIter	;	//FaceIter type
    typedef MyMesh::FaceVertexIter							FVIter	;	//FaceVertexIter type
    typedef MyMesh::FaceEdgeIter							FEIter	;	//FaceEdgeIter type
    typedef MyMesh::FaceHalfedgeIter						FHIter;	//FaceHalfedgeIter type
    typedef MyMesh::FaceFaceIter							FFIter	;	//FaceFaceIter type
    typedef MyMesh::VertexOHalfedgeIter						VOHEIter;	//VertexOutHalfEdge type
    typedef MyMesh::VertexIHalfedgeIter						VIHEIter;	//VertexInHalfEdge type
    typedef MyMesh::ConstVertexVertexIter					CVVIter	;	//ConstVertexVertexIter type
    /*----------------------------------------------------------------------*/

    /*定義額外資料結構*/
    using namespace OpenMesh;
    /*----------------------------------------------------------------------*/

    /*定義類別*/
    class Model:public MyMesh
    {
    public:
        Model();//constructor
        ~Model();//de-constructor
    };
}
class Tri_Mesh:public OMT::Model
{
public:
    Tri_Mesh()
    {


    }
    Tri_Mesh(const Tri_Mesh &other) : OMT::Model(other) {
        m_vao = other.m_vao;
        m_triFaceEbo = other.m_triFaceEbo;
        m_edgeEbo = other.m_edgeEbo;
        m_coordVbo = other.m_coordVbo;
    }
    //-------Edit Flag-------//

    void createVAO();
    void updateVAO();


    uint32_t m_vao, m_triFaceEbo, m_edgeEbo, m_coordVbo;
    int m_edgeIndices, m_faceIndices;
private:
};

#endif //RAY_OPENMESHDEF_H
