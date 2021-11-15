#pragma once

#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

#include "Simplification.h"

#include "OpenMeshDef.h"
#include <vector>

class SKErrorMetric {
public:
    SKErrorMetric(OMT::VHandle fromVertexHandle, OMT::VHandle toVertexHandle, double metric) : m_from(fromVertexHandle),
                                                                                               m_to(toVertexHandle),
                                                                                               m_metric(metric) {

    }

    OMT::VHandle m_from;
    OMT::VHandle m_to;
    double m_metric;

    bool operator<(const SKErrorMetric &rhs) const {
        return m_metric < rhs.m_metric;
    }

private:
};

class SkeletonFace {
public:
    OMT::VHandle m_from;
    OMT::VHandle m_to[2];
};

class SkeletonExtraction {
public:
    SkeletonExtraction(Tri_Mesh *mesh) {
        m_originalMesh = new Tri_Mesh(*mesh);

        size_t edgeCount = mesh->n_edges();
//        m_pQueue = new SKErrorMetric[edgeCount + 1];
    }

    ~SkeletonExtraction() {
//        delete[] m_pQueue;
    }

    void calculateSkeleton();

    Tri_Mesh *getCurrentMesh() {
        if (m_operateMesh == nullptr) {
            return m_originalMesh;
        }
        return m_operateMesh;
    }

    // DegenerateMeshToLine

//    bool isCollapsable(Tri_Mesh *mesh, OMT::HEHandle halfedgeHandle);

    void initErrorMetric(Tri_Mesh *mesh, std::map<OMT::VHandle, std::vector<SKErrorMetric>> &outHalfedgeMap,
                         std::map<OMT::VHandle, std::vector<SkeletonFace>> &outFaceMap);

    void propagateToTop(Tri_Mesh *mesh, int heapIdx);
    void propagateToBottom(Tri_Mesh *mesh, int heapIdx);

    void insertEdge(Tri_Mesh *mesh, SKErrorMetric *skErrorMetric);
    SKErrorMetric *getTopEdge(Tri_Mesh *mesh);
    void deleteEdge(Tri_Mesh *mesh, SKErrorMetric *skErrorMetric);
    void changeEdge(Tri_Mesh *mesh, SKErrorMetric *skErrorMetric);

    bool collapseEdge(Tri_Mesh *mesh,
                      std::map<OMT::VHandle, std::vector<SKErrorMetric>> &outHalfedgeMap,
                      std::map<OMT::VHandle, std::vector<SkeletonFace>> &outFaceMap);

    void halfedgeCollapse(std::map<OMT::VHandle, std::vector<SKErrorMetric>> &outHalfedgeMap,
                          std::map<OMT::VHandle, std::vector<SkeletonFace>> &outFaceMap,
                          std::vector<SKErrorMetric>::iterator collapseEdgeIter);

    void computeVertexMetric(Tri_Mesh *mesh, std::map<OMT::VHandle, std::vector<SKErrorMetric>> &outHalfedgeMap, OMT::VHandle vHandle);

    void computeErrorMetric(Tri_Mesh *mesh, SKErrorMetric &em, std::map<OMT::VHandle, std::vector<SKErrorMetric>> &outHalfedgeMap);

    //
    void simplifyMesh();

    Tri_Mesh *m_operateMesh = nullptr;

    // Least square mesh contraction
    double m_wL;
    //4.5 213 is the plausible minimum
    double w_l_factor = 12.0;

    double m_initWH = 1.0;

    int m_k = 0;
    std::vector<double> m_eachVertexWH;
    double m_initialAverageFaceArea;

    // Degenerate to line
    std::vector<SKErrorMetric *> m_pQueue;
    std::map<SKErrorMetric *, int> m_pQueueHeapIdx;

    std::map<OMT::VHandle, std::vector<SKErrorMetric>> m_outHalfedgeMap;
    std::map<OMT::VHandle, std::vector<SkeletonFace>> m_outFaceMap;

    int m_currentTailIdx = 0;

    OpenMesh::VPropHandleT<Eigen::Matrix4d> m_K;
    OpenMesh::VPropHandleT<double> m_KTotal;
//    OpenMesh::EPropHandleT<int> m_heapIdxProp;

    double m_wa = 1.0, m_wb = 0.1;
    bool m_isInitializedQ = false;

    void updateSkeletonEBO(std::vector<uint32_t> &edgeSkeletonIndices);

private:
    double calculateInitialAverageFaceArea();

    double calculateCurrentFaceArea();

    double calculateAngle(const OMT::Point &a, const OMT::Point &b);

    double calcCot(const OMT::Point &a, const OMT::Point &b);

    Tri_Mesh *m_originalMesh = nullptr;

    std::vector<double> m_initialOneRingArea;

//	std::vector<SKErrorMetric> m_discardedErrorMetric;

    int m_totalFace = 0;
};
