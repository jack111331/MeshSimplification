#pragma once
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

#include "Simplification.h"

#include "OpenMeshDef.h"
#include <vector>

class SKErrorMetric {
public:
    // TODO change to HEHandle
    OMT::EHandle m_edgeHandle;
    double m_metric;
    bool operator < (const SKErrorMetric &rhs) const {
        return m_metric < rhs.m_metric;
    }
private:
};

class SkeletonExtraction {
public:
	SkeletonExtraction(Tri_Mesh* mesh) {
		m_originalMesh = new Tri_Mesh(*mesh);

		size_t edgeCount = mesh->n_edges();
        m_pQueue = new SKErrorMetric[edgeCount + 1];
	}
	~SkeletonExtraction() {
	    delete [] m_pQueue;
	}

	void calculateSkeleton();

	Tri_Mesh* getCurrentMesh() {
		if (m_operateMesh == nullptr) {
			return m_originalMesh;
		}
		return m_operateMesh;
	}

	// DegenerateMeshToLine
	void degenerateToLine();

	bool isCollapsable(Tri_Mesh *mesh, OMT::HEHandle halfedgeHandle);

    void initErrorMetric(Tri_Mesh *mesh);

    void propagateToTop(Tri_Mesh *mesh, int heapIdx);
    void propagateToBottom(Tri_Mesh *mesh, int heapIdx);

    void insertEdge(Tri_Mesh *mesh, OMT::EHandle edgeHandle);
    SKErrorMetric getTopEdge(Tri_Mesh *mesh);
    void deleteEdge(Tri_Mesh *mesh, int heapIdx);
    void changeEdge(Tri_Mesh *mesh, int heapIdx);
    void updateVertexK(Tri_Mesh *mesh, OMT::VHandle vertexHandle);

    bool collapseEdge(Tri_Mesh *mesh, OMT::HEHandle halfedgeHandle);

    void initEdgeMetric(Tri_Mesh *mesh);
    void easierCollapseEdge(Tri_Mesh *mesh);

    SKErrorMetric computeErrorMetric(Tri_Mesh *mesh, OMT::EHandle edgeHandle);

    //
	void simplifyMesh();

	Tri_Mesh* m_operateMesh = nullptr;

	// Least square mesh contraction
	double m_wL;
	//4.5 213 is the plausible minimum
	double w_l_factor = 12.0;

	double m_initWH = 1.0;

	int m_k = 0;
    std::vector<double> m_eachVertexWH;
    double m_initialAverageFaceArea;

    // Degenerate to line
    SKErrorMetric *m_pQueue;

    int m_currentTailIdx = 0;

    OpenMesh::VPropHandleT<Eigen::Matrix4d> m_K;
    OpenMesh::EPropHandleT<int> m_heapIdxProp;

    double m_wa = 1.0, m_wb = 0.1;
    bool m_isInitializedQ = false;

    int m_lastDiscardSize = -1;

private:
	double calculateInitialAverageFaceArea();
	double calculateCurrentFaceArea();
	double calculateAngle(const OMT::Point &a, const OMT::Point &b);

	double calcCot(const OMT::Point& a, const OMT::Point& b);

	Tri_Mesh* m_originalMesh = nullptr;

	std::vector<double> m_initialOneRingArea;

	std::vector<SKErrorMetric> m_discardedErrorMetric;

	std::vector<double> m_cost;
    std::vector<bool> m_isDiscard;

};
