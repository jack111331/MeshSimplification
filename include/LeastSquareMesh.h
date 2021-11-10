#pragma once
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

#include "OpenMeshDef.h"

#include <vector>
class LeastSquareMesh {
public:
	LeastSquareMesh(Tri_Mesh *mesh) {
		m_originalMesh = new Tri_Mesh(*mesh);
	}

	void calculateLeastSquare();
	Tri_Mesh* getCurrentMesh() {
		if (m_operateMesh == nullptr) {
			return m_originalMesh;
		}
		return m_operateMesh;
	}

	Tri_Mesh* m_operateMesh = nullptr;
	static const int CONTROL_POINT_AMOUNT = 2001;
	int m_controlPoint[CONTROL_POINT_AMOUNT];
	int m_controlPointCount = 0;

	std::vector<bool> m_nonselectedVertexIdx;

	double w_l = 1.0;
	double w_l_factor = 2.0;

	double w_h = 1.0;
	double w_h_factor = 1.0;

	const int EVERY_K = 31;

private:
	void searchLocalMaximum(int idx);
	double calculateError(OMT::Scalar* a, OMT::Scalar* b);

	Tri_Mesh* m_originalMesh = nullptr;

	std::vector<bool> m_visit;
	std::vector<double> m_error;
};

