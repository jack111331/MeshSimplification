#include "LeastSquareMesh.h"
#include <queue>

void LeastSquareMesh::calculateLeastSquare() {
	size_t vertexCount = m_originalMesh->n_vertices();

	m_nonselectedVertexIdx.resize(vertexCount, true);
	m_visit.resize(vertexCount);
	m_error.resize(vertexCount);

	m_nonselectedVertexIdx[0] = false;
	m_controlPoint[m_controlPointCount++] = 0;

	double maxError = 0;
	int maxErrorIdx = 0;
	//		w_l = 0.001 *;
			// FIXME CONTROL_POINT_AMOUNT
	int TIMES = 60;
	for (int k = 0; k < TIMES; ++k) {
		std::cout << k << std::endl;
		if (m_operateMesh != nullptr) {
			delete m_operateMesh;
		}
		m_operateMesh = new Tri_Mesh(*m_originalMesh);

		// TODO loop through all original vertex and previous vertex to find the maximum error diff one
		Eigen::SparseMatrix<double> A(vertexCount + m_controlPointCount, vertexCount);
		Eigen::SparseMatrix<double> B(vertexCount + m_controlPointCount, 3);

		std::vector<Eigen::Triplet<double>> coef;
		// Setup A_1 matrix
		for (int i = 0; i < vertexCount; ++i) {
			OMT::VHandle currentVertex = m_operateMesh->vertex_handle(i);
			int degree = 0;
			OMT::HEHandle h = m_operateMesh->voh_begin(currentVertex).handle();
			OMT::HEHandle hStop = h;
			std::vector<int> outgoingVertexIdx;
			do {
				degree++;
				outgoingVertexIdx.emplace_back(m_operateMesh->to_vertex_handle(h).idx());
				h = m_operateMesh->next_halfedge_handle(m_operateMesh->opposite_halfedge_handle(h));
			} while (h != hStop);
			double invDegree = 1 / (double)degree;
			coef.push_back(Eigen::Triplet<double>(i, i, 1.0));
			for (int j = 0; j < outgoingVertexIdx.size(); ++j) {
				coef.push_back(Eigen::Triplet<double>(i, outgoingVertexIdx[j], -invDegree));
			}
		}
		for (int i = 0; i < m_controlPointCount; ++i) {
			OMT::VHandle v = m_operateMesh->vertex_handle(m_controlPoint[i]);
			coef.push_back(Eigen::Triplet<double>(vertexCount + i, v.idx(), 1.0));
		}
		A.setFromTriplets(coef.begin(), coef.end());

		// Setup B matrix
		coef.clear();
		for (int i = 0; i < m_controlPointCount; ++i) {
			OMT::VHandle v = m_operateMesh->vertex_handle(m_controlPoint[i]);
			OMT::Scalar* pointCoord = m_operateMesh->point(v).data();
			coef.push_back(Eigen::Triplet<double>(vertexCount + i, 0, pointCoord[0]));
			coef.push_back(Eigen::Triplet<double>(vertexCount + i, 1, pointCoord[1]));
			coef.push_back(Eigen::Triplet<double>(vertexCount + i, 2, pointCoord[2]));
		}
		B.setFromTriplets(coef.begin(), coef.end());
		// Factorize to upper-triangle LL^T = U^T U form
		Eigen::SimplicialLLT < Eigen::SparseMatrix<double>> solver(A.transpose() * A);
		Eigen::SparseMatrix<double> X = solver.solve(A.transpose() * B);

		// new X [[v_1.x, v_2.y, v_3.z], ...]
		for (int i = 0; i < vertexCount; ++i) {
			OMT::VHandle v = m_operateMesh->vertex_handle(i);

			OMT::Vec3d pointCoord = { X.coeff(i, 0), X.coeff(i, 1), X.coeff(i, 2) };
			OMT::Scalar* OpointCoord = m_operateMesh->point(v).data();
			m_operateMesh->set_point(v, pointCoord);
		}
		maxError = 0.0;
		for (int i = 0; i < vertexCount; ++i) {
			OMT::Scalar* pointCoord = m_originalMesh->point(m_originalMesh->vertex_handle(i)).data();
			OMT::Scalar* OPointCoord = m_operateMesh->point(m_operateMesh->vertex_handle(i)).data();
			if (i == 0) {
				std::cout << "pointCoord =" << pointCoord[0] << " " << pointCoord[1] << " " << pointCoord[2] << std::endl;
				std::cout << "OPointCoord =" << OPointCoord[0] << " " << OPointCoord[1] << " " << OPointCoord[2] << std::endl;
			}
			m_error[i] = calculateError(pointCoord, OPointCoord);
			if (m_error[i] > maxError && m_nonselectedVertexIdx[i] == true) {
				maxError = m_error[i];
				maxErrorIdx = i;
			}
		}
		std::cout << "Max error, idx =" << maxError << " " << maxErrorIdx << std::endl;
		searchLocalMaximum(maxErrorIdx);
	}
}

void LeastSquareMesh::searchLocalMaximum(int idx) {
	std::queue<int> nextIdxQueue;
	nextIdxQueue.push(idx);
	std::fill(m_visit.begin(), m_visit.end(), false);

	int chosenK = 0;

	while (!nextIdxQueue.empty()) {
		int top = nextIdxQueue.front();
		nextIdxQueue.pop();
		if (m_visit[top] == true) {
			continue;
		}
		m_visit[top] = true;
		OMT::VHandle startV = m_operateMesh->vertex_handle(top);
		OMT::HEHandle h = m_operateMesh->voh_begin(startV).handle();
		OMT::HEHandle hStop = h;
		bool isLocalMaxima = true;
		bool isNearRootIdx = false;
		do {
			OMT::VHandle v = m_operateMesh->to_vertex_handle(h);
			OMT::HEHandle halfedgeHandle = m_operateMesh->find_halfedge(startV, v);
			if (halfedgeHandle.is_valid()) {
				if (m_visit[v.idx()] == false) {
					nextIdxQueue.push(v.idx());
				}
				if (v.idx() == idx) {
					isNearRootIdx = true;
				}
				if (m_error[startV.idx()] < m_error[v.idx()]) {
					isLocalMaxima = false;
				}
			}
			h = m_operateMesh->next_halfedge_handle(m_operateMesh->opposite_halfedge_handle(h));
		} while (h != hStop);
		if (isNearRootIdx == false && isLocalMaxima == true && m_nonselectedVertexIdx[startV.idx()] == true) {
			m_controlPoint[m_controlPointCount++] = startV.idx();
			m_nonselectedVertexIdx[startV.idx()] = false;
			chosenK++;
		}

		if (chosenK >= EVERY_K) {
			break;
		}

	}
}

double LeastSquareMesh::calculateError(OMT::Scalar* a, OMT::Scalar* b) {
	return sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}