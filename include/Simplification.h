#pragma once

#include "OpenMeshDef.h"
#include <Eigen/Core>
#include "Matrix.h"
#include "Vec.h"

class ErrorMetric {
public:
    // TODO change to HEHandle
    OMT::EHandle m_edgeHandle;
    double m_metric;
    MeshSimplification::Vec<4, double> m_optimalVertexPos;

    bool operator < (const ErrorMetric &rhs) const {
        return m_metric < rhs.m_metric;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
};

class SuitableVertexExtraction {
public:
    SuitableVertexExtraction(int n) {
        m_pQueue = new ErrorMetric[n + 1];
    }

    ~SuitableVertexExtraction() {
        delete [] m_pQueue;
    }

    void initErrorMetric(Tri_Mesh *mesh);

    void propagateToTop(Tri_Mesh *mesh, int heapIdx);
    void propagateToBottom(Tri_Mesh *mesh, int heapIdx);

    void insertEdge(Tri_Mesh *mesh, OMT::EHandle edgeHandle);
    ErrorMetric getTopEdge(Tri_Mesh *mesh);
    ErrorMetric extractTopEdge(Tri_Mesh *mesh);
    void deleteEdge(Tri_Mesh *mesh, int heapIdx);
    void changeEdge(Tri_Mesh *mesh, int heapIdx);
    void updateVertexQ(Tri_Mesh *mesh, OMT::VHandle vertexHandle);

    void collapseEdge(Tri_Mesh *mesh, OMT::EHandle edgeHandle, MeshSimplification::Vec<4, double> targetVertex);
    void flushMesh(Tri_Mesh *mesh);

    ErrorMetric computeErrorMetric(Tri_Mesh *mesh, OMT::EHandle edgeHandle);
private:


    ErrorMetric *m_pQueue;

    int m_currentTailIdx = 0;

    OpenMesh::VPropHandleT<MeshSimplification::XForm<double>> m_Q;
    OpenMesh::EPropHandleT<int> m_heapIdxProp;

};


class MeshManager {
public:
    MeshManager(Tri_Mesh *mesh) : m_meshes{} {
        m_meshes[0] = new Tri_Mesh(*mesh);
        m_currentMeshIdx = 0;

        m_originalMesh = new Tri_Mesh(*mesh);
        size_t edgeCount = m_originalMesh->n_edges();
        m_collapsePerPortion = m_originalMesh->n_vertices() / 10;
        m_extractor = new SuitableVertexExtraction(edgeCount);
        m_extractor->initErrorMetric(m_originalMesh);
        for (size_t i = 0; i < edgeCount; ++i) {
            m_extractor->insertEdge(m_originalMesh, m_originalMesh->edge_handle(i));
        }

    }

    void lowerMeshLevel() {
        if (m_currentMeshIdx < MAX_MESH_LEVEL_COUNT - 1) {
            if (m_meshes[m_currentMeshIdx + 1] == nullptr) {
                for (size_t i = 0; i < m_collapsePerPortion; ++i) {
                    ErrorMetric em = m_extractor->getTopEdge(m_originalMesh);
                    m_extractor->collapseEdge(m_originalMesh, em.m_edgeHandle, em.m_optimalVertexPos);
                }
                Tri_Mesh *duplicateMesh = new Tri_Mesh(*m_originalMesh);
                m_meshes[m_currentMeshIdx + 1] = duplicateMesh;
            }
            m_currentMeshIdx++;
        }
    }

    void higherMeshLevel() {
        if (m_currentMeshIdx > 0) {
            m_currentMeshIdx--;
            std::cerr << "Currently at " << m_currentMeshIdx << std::endl;
        }
    }

    Tri_Mesh *getCurrentMesh() {
        return m_meshes[m_currentMeshIdx];
    }

    SuitableVertexExtraction *m_extractor;
    static const int MAX_MESH_LEVEL_COUNT = 11;
    Tri_Mesh *m_meshes[MAX_MESH_LEVEL_COUNT];
    Tri_Mesh *m_originalMesh;
    int m_currentMeshIdx = 0;

    int m_collapsePerPortion;
};
