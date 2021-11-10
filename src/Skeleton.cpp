#include "Skeleton.h"
#include <cmath>
#include "Eigen/Dense"

void SkeletonExtraction::calculateSkeleton() {
    size_t vertexCount;
    if(m_k == 0){
        m_originalMesh->garbage_collection();
        vertexCount = m_originalMesh->n_vertices();

        m_initialOneRingArea.resize(vertexCount, 0.0);
        if (m_operateMesh != nullptr) {
            delete m_operateMesh;
        }
        m_operateMesh = new Tri_Mesh(*m_originalMesh);
        m_eachVertexWH.resize(vertexCount);
        m_initialAverageFaceArea = calculateInitialAverageFaceArea();
        m_wL = 0.001 * sqrt(m_initialAverageFaceArea);
        m_initWH = 1.0;

    }
    vertexCount = m_originalMesh->n_vertices();

    wxLogMessage("%dth iteration: m_wL=%f", m_k, m_wL);
    std::cout << m_k << "th iteration" << std::endl;
    std::cerr << m_wL << std::endl;

    Eigen::SparseMatrix<double> A(2*vertexCount, vertexCount);
    Eigen::SparseMatrix<double> B(2*vertexCount, 3);

    std::vector<Eigen::Triplet<double>> coef;

    // Setup A_1 matrix
    for (int i = 0; i < vertexCount; ++i) {
        OMT::VHandle currentVertex = m_operateMesh->vertex_handle(i);

        OMT::HEHandle h = m_operateMesh->voh_begin(currentVertex).handle();
        OMT::HEHandle hStop = h;

        // halfedge index, angle
        std::vector<std::tuple<int, double, double>> cotPropList;
        do {
            OMT::HEHandle thirdH;
            OMT::FHandle face = m_operateMesh->face_handle(h);
            OMT::FHIter fh_it = m_operateMesh->fh_iter(face);
            while (fh_it.handle() != h) {
                ++fh_it;
            }
            ++fh_it;
            OMT::Normal edgeVector_1 = m_operateMesh->calc_edge_vector(m_operateMesh->opposite_halfedge_handle(fh_it.handle()));
            ++fh_it;
            OMT::Normal edgeVector_2 = m_operateMesh->calc_edge_vector(fh_it.handle());
            double cot_1 = calcCot(edgeVector_1.normalized(), edgeVector_2.normalized());
            OMT::HEHandle oppositeH = m_operateMesh->opposite_halfedge_handle(h);
            face = m_operateMesh->face_handle(oppositeH);
            fh_it = m_operateMesh->fh_iter(face);
            while (fh_it.handle() != oppositeH) {
                ++fh_it;
            }
            ++fh_it;
            edgeVector_1 = m_operateMesh->calc_edge_vector(m_operateMesh->opposite_halfedge_handle(fh_it.handle()));
            ++fh_it;
            edgeVector_2 = m_operateMesh->calc_edge_vector(fh_it.handle());
            double cot_2 = calcCot(edgeVector_1.normalized(), edgeVector_2.normalized());
            cotPropList.push_back(std::tuple<int, double, double>{h.idx(), cot_1, cot_2});
            h = m_operateMesh->next_halfedge_handle(m_operateMesh->opposite_halfedge_handle(h));
        } while (h != hStop);

        if (m_k == 0) {
            m_eachVertexWH[i] = m_initWH;
        }
        else {
            double currentOneRingArea = 0.0;
            for (OMT::VFIter vf_it = m_operateMesh->vf_begin(currentVertex); vf_it != m_operateMesh->vf_end(currentVertex); ++vf_it) {
                OMT::FHandle faceHandle = vf_it.handle();
                OMT::FVIter fv_it = m_operateMesh->fv_begin(faceHandle);
                const OMT::Point& P = m_operateMesh->point(fv_it); ++fv_it;
                const OMT::Point& Q = m_operateMesh->point(fv_it); ++fv_it;
                const OMT::Point& R = m_operateMesh->point(fv_it);
                double area = ((Q - P) % (R - P)).norm() * 0.5;
                currentOneRingArea += area;
            }
            m_eachVertexWH[i] = m_initWH * sqrt(m_initialOneRingArea[i] / currentOneRingArea);
        }
        double totalWeight = 0.0;
        std::vector<std::pair<int, double>> weightList;
        double normalizeWeight = 0.0;
        for (int j = 0; j < cotPropList.size(); ++j) {
            // normalize all row weight
            double weight = std::get<1>(cotPropList[j]) + std::get<2>(cotPropList[j]);
            // FIXME nan weight problem
            if (isinf(weight)) {
                std::cerr << weight << std::endl;
                weight = 8165619676597685;
            }
            else if (isnan(weight)) {
                std::cerr << weight << std::endl;
                weight = 0.0;
            }
            OMT::VHandle v = m_operateMesh->to_vertex_handle(m_operateMesh->halfedge_handle(std::get<0>(cotPropList[j])));
            if (isnan(m_wL * weight) || isinf(m_wL * weight)) {
                std::cerr << "Aw inf or nan" << " " << m_wL << " " << weight << std::endl;
            }
            weightList.push_back(std::pair<int, double>(v.idx(), weight));
            normalizeWeight += weight * weight;
            totalWeight += weight;
        }
        normalizeWeight += totalWeight * totalWeight;
        normalizeWeight = sqrt(normalizeWeight);
        for (auto w : weightList) {
//            coef.push_back(Eigen::Triplet<double>(i, w.first, m_wL * w.second / normalizeWeight));
            coef.push_back(Eigen::Triplet<double>(i, w.first, m_wL * w.second));
        }
//        coef.push_back(Eigen::Triplet<double>(i, i, m_wL * -totalWeight / normalizeWeight));
        coef.push_back(Eigen::Triplet<double>(i, i, m_wL * -totalWeight));
        coef.push_back(Eigen::Triplet<double>(vertexCount + i, i, m_eachVertexWH[i]));
    }
    A.setFromTriplets(coef.begin(), coef.end());
    // Setup B matrix
    coef.clear();
    for (int i = 0; i < vertexCount; ++i) {
        OMT::VHandle v = m_operateMesh->vertex_handle(i);
        OMT::Scalar* pointCoord = m_operateMesh->point(v).data();
        coef.push_back(Eigen::Triplet<double>(vertexCount + i, 0, m_eachVertexWH[i] * pointCoord[0]));
        coef.push_back(Eigen::Triplet<double>(vertexCount + i, 1, m_eachVertexWH[i] * pointCoord[1]));
        coef.push_back(Eigen::Triplet<double>(vertexCount + i, 2, m_eachVertexWH[i] * pointCoord[2]));
    }
    B.setFromTriplets(coef.begin(), coef.end());
    // Factorize to upper-triangle LL^T = U^T U form
    Eigen::SimplicialCholesky < Eigen::SparseMatrix<double>> solver(A.transpose() * A);
    Eigen::SparseMatrix<double> X = solver.solve(A.transpose() * B);


    double currentFaceArea = calculateCurrentFaceArea();
    std::cout << "Face area=" << currentFaceArea << std::endl;
    if (currentFaceArea / (m_initialAverageFaceArea * m_originalMesh->n_faces()) <= 1e-6) {
        wxLogMessage("Total face area lower than threshold: %d", m_k);
    }

    // new X [[v_1.x, v_2.y, v_3.z], ...]
    for (OMT::VIter v_it = m_operateMesh->vertices_begin(); v_it != m_operateMesh->vertices_end(); ++v_it) {
        int idx = v_it.handle().idx();
        OMT::Vec3d pointCoord = { X.coeff(idx, 0), X.coeff(idx, 1), X.coeff(idx, 2) };
        m_operateMesh->set_point(v_it.handle(), pointCoord);
        /*
        if (k == TIMES - 1) {
            std::cerr << pointCoord[0] << " " << pointCoord[1] << " " << pointCoord[2] << std::endl;
        }
        */
    }
    if (m_wL > 200) {
        m_wL *= 1.1;
    } else {
        m_wL *= w_l_factor;
    }
    /*
    if (m_refreshCallback != nullptr) {
        m_refreshCallback();
    }
    */
	m_k++;
}

double SkeletonExtraction::calculateInitialAverageFaceArea() {
	double averageArea = 0.0;
	for (int i = 0; i < m_originalMesh->n_faces(); ++i) {
		OMT::FHandle faceHandle = m_originalMesh->face_handle(i);
		OMT::FVIter fv_it = m_originalMesh->fv_begin(faceHandle);
		const OMT::Point& P = m_originalMesh->point(fv_it); ++fv_it;
		const OMT::Point& Q = m_originalMesh->point(fv_it); ++fv_it;
		const OMT::Point& R = m_originalMesh->point(fv_it);
		double area = ((Q - P) % (R - P)).norm() * 0.5;
		fv_it = m_originalMesh->fv_begin(faceHandle);
		m_initialOneRingArea[fv_it.handle().idx()] += area; ++fv_it;
		m_initialOneRingArea[fv_it.handle().idx()] += area; ++fv_it;
		m_initialOneRingArea[fv_it.handle().idx()] += area;
		averageArea += area;
	}
	return averageArea / m_originalMesh->n_faces();
}

double SkeletonExtraction::calculateCurrentFaceArea() {
	double averageArea = 0.0;
	for (int i = 0; i < m_operateMesh->n_faces(); ++i) {
		OMT::FHandle faceHandle = m_operateMesh->face_handle(i);
		OMT::FVIter fv_it = m_operateMesh->fv_begin(faceHandle);
		const OMT::Point& P = m_operateMesh->point(fv_it); ++fv_it;
		const OMT::Point& Q = m_operateMesh->point(fv_it); ++fv_it;
		const OMT::Point& R = m_operateMesh->point(fv_it);
		double area = ((Q - P) % (R - P)).norm() * 0.5;
		averageArea += area;
	}
	return averageArea;
}

double SkeletonExtraction::calculateAngle(const OMT::Point& a, const OMT::Point& b) {
	return acos(a.normalized() | b.normalized());
}

double SkeletonExtraction::calcCot(const OMT::Point& a, const OMT::Point& b) {
	if (a == b) {
		return std::numeric_limits<double>::infinity();
	}
	return((a | b) / (a % b).norm());
}

void SkeletonExtraction::degenerateToLine() {

}

bool SkeletonExtraction::isCollapsable(Tri_Mesh *mesh, OMT::HEHandle halfedgeHandle) {
//    std::set<OMT::VHandle> fromVertexOneRingVertices;
//    std::set<OMT::VHandle> toVertexOneRingVertices;
//    OMT::VHandle fromVertexHandle = mesh->from_vertex_handle(halfedgeHandle);
//    OMT::VHandle toVertexHandle = mesh->to_vertex_handle(halfedgeHandle);
//    for (OMT::VOHEIter voh_it = mesh->voh_begin(fromVertexHandle);voh_it != mesh->voh_end(fromVertexHandle); ++voh_it) {
//        fromVertexOneRingVertices.insert(mesh->to_vertex_handle(*voh_it));
//    }
//    for (OMT::VOHEIter voh_it = mesh->voh_begin(toVertexHandle);voh_it != mesh->voh_end(toVertexHandle); ++voh_it) {
//        toVertexOneRingVertices.insert(mesh->to_vertex_handle(*voh_it));
//    }
//    std::set<OMT::VHandle> commonVertices;
//    set_intersection(fromVertexOneRingVertices.begin(), fromVertexOneRingVertices.end(), toVertexOneRingVertices.begin(), toVertexOneRingVertices.end(),
//                     std::inserter(commonVertices, commonVertices.begin()));
//    std::set<OMT::VHandle> faceVertices;
//    OMT::FVIter fv_it = mesh->fv_iter(mesh->face_handle(halfedgeHandle));
//    while(true) {
//        if (*fv_it != fromVertexHandle && *fv_it != toVertexHandle) {
//            faceVertices.insert(*fv_it);
//            break;
//        }
//        ++fv_it;
//    }
//    fv_it = mesh->fv_iter(mesh->face_handle(mesh->opposite_halfedge_handle(halfedgeHandle)));
//    while(true) {
//        if (*fv_it != fromVertexHandle && *fv_it != toVertexHandle) {
//            faceVertices.insert(*fv_it);
//            break;
//        }
//        ++fv_it;
//    }
//    return faceVertices == commonVertices;
    return mesh->face_handle(halfedgeHandle).is_valid() && mesh->face_handle(mesh->opposite_halfedge_handle(halfedgeHandle)).is_valid();
}

void SkeletonExtraction::propagateToTop(Tri_Mesh *mesh, int heapIdx) {
    SKErrorMetric em = computeErrorMetric(mesh, m_pQueue[heapIdx].m_halfedgeHandle);
    while (heapIdx > 1) {
        if (m_pQueue[heapIdx] < m_pQueue[heapIdx / 2]) {
            SKErrorMetric temp = m_pQueue[heapIdx];
            m_pQueue[heapIdx] = m_pQueue[heapIdx / 2];
            mesh->property(m_heapIdxProp, m_pQueue[heapIdx].m_halfedgeHandle) = heapIdx;
            m_pQueue[heapIdx / 2] = temp;
            heapIdx /= 2;
        }
        else {
            break;
        }
    }
    mesh->property(m_heapIdxProp, em.m_halfedgeHandle) = heapIdx;
}
void SkeletonExtraction::propagateToBottom(Tri_Mesh *mesh, int heapIdx) {
    SKErrorMetric em = computeErrorMetric(mesh, m_pQueue[heapIdx].m_halfedgeHandle);
    while (heapIdx <= m_currentTailIdx) {
        int minIdx = -1;
        if (heapIdx * 2 + 1 <= m_currentTailIdx) {
            if (m_pQueue[heapIdx * 2 + 1] < m_pQueue[heapIdx * 2]) {
                minIdx = heapIdx * 2 + 1;
            }
            else {
                minIdx = heapIdx * 2;
            }
        }
        else if (heapIdx * 2 <= m_currentTailIdx) {
            minIdx = heapIdx * 2;
        }
        if (minIdx != -1) {
            if (m_pQueue[minIdx] < m_pQueue[heapIdx]) {
                mesh->property(m_heapIdxProp, m_pQueue[minIdx].m_halfedgeHandle) = heapIdx;
                SKErrorMetric temp = m_pQueue[heapIdx];
                m_pQueue[heapIdx] = m_pQueue[minIdx];
                m_pQueue[minIdx] = temp;
                heapIdx = minIdx;
            }
            else {
                break;
            }
        }
        else {
            break;
        }
    }
    mesh->property(m_heapIdxProp, em.m_halfedgeHandle) = heapIdx;
}

void SkeletonExtraction::insertEdge(Tri_Mesh *mesh, OMT::HEHandle halfedgeHandle) {
    SKErrorMetric em = computeErrorMetric(mesh, halfedgeHandle);
    m_pQueue[++m_currentTailIdx] = em;
    mesh->property(m_heapIdxProp, halfedgeHandle) = m_currentTailIdx;

    propagateToTop(mesh, m_currentTailIdx);
}

SKErrorMetric SkeletonExtraction::getTopEdge(Tri_Mesh *mesh) {
    SKErrorMetric ret = m_pQueue[1];

    return ret;
}

void SkeletonExtraction::deleteEdge(Tri_Mesh *mesh, int heapIdx) {
    m_pQueue[heapIdx] = m_pQueue[m_currentTailIdx--];
    mesh->property(m_heapIdxProp, m_pQueue[heapIdx].m_halfedgeHandle) = heapIdx;

    propagateToTop(mesh, heapIdx);
    propagateToBottom(mesh, heapIdx);
}

void SkeletonExtraction::changeEdge(Tri_Mesh *mesh, int heapIdx) {
    SKErrorMetric newErrorMetric = computeErrorMetric(mesh, m_pQueue[heapIdx].m_halfedgeHandle);
    if (m_pQueue[heapIdx].m_metric < newErrorMetric.m_metric) {
        m_pQueue[heapIdx] = newErrorMetric;

        propagateToTop(mesh, heapIdx);

    }
    else {
        m_pQueue[heapIdx] = newErrorMetric;

        propagateToBottom(mesh, heapIdx);
    }
}

void SkeletonExtraction::initErrorMetric(Tri_Mesh *mesh) {
    // Fix template type
    mesh->add_property(m_K);
    mesh->add_property(m_heapIdxProp);

    for (OMT::VIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
    {
        // initialize
        updateVertexK(mesh, v_it.handle());
    }
}

void SkeletonExtraction::updateVertexK(Tri_Mesh *mesh, OMT::VHandle vertexHandle) {
    OMT::Scalar *pointCoord = mesh->point(vertexHandle).data();
    OMT::Point vPoint = OMT::Point(pointCoord[0], pointCoord[1], pointCoord[2]);
    Eigen::Vector3d vP(vPoint[0], vPoint[1], vPoint[2]);
    mesh->property(m_K, vertexHandle).setZero();
    for (OMT::VOHEIter voh_it = mesh->voh_begin(vertexHandle); voh_it != mesh->voh_end(vertexHandle); ++voh_it) {
        OMT::VHandle toVertexHandle = mesh->to_vertex_handle(*voh_it);
        OMT::Point p = mesh->point(toVertexHandle);
        Eigen::Vector3d a(p[0]-vPoint[0], p[1]-vPoint[1], p[2]-vPoint[2]);
        a.normalize();
        Eigen::Vector3d b = a.cross(vP);
        Eigen::MatrixXd K(3, 4);
        K << 0, -a[2], a[1], -b[0],
        a[2], 0, -a[0], -b[1],
        -a[1], a[0], 0, -b[2];
        mesh->property(m_K, vertexHandle) += (K.transpose() * K);
    }
}

bool SkeletonExtraction::collapseEdge(Tri_Mesh *mesh, OMT::HEHandle halfedgeHandle) {
//    Timer timer;
//    timer.updateCurrentTime();
    wxLogMessage("Debug - 1");
    if(m_currentTailIdx < 1) {
        for(auto em : m_discardedErrorMetric) {
            if (mesh->edge_handle(em.m_halfedgeHandle).is_valid()) {
                insertEdge(mesh, em.m_halfedgeHandle);
            }
        }
        m_discardedErrorMetric.clear();
    }
    wxLogMessage("Debug - 2 : is_valid %d", halfedgeHandle.is_valid());

    if (isCollapsable(mesh, halfedgeHandle)) {
        wxLogMessage("Debug - 2-1");

        OMT::VHandle toEdgeVertexHandle = mesh->to_vertex_handle(halfedgeHandle);
        OMT::VHandle fromEdgeVertexHandle = mesh->from_vertex_handle(halfedgeHandle);
//        wxLogMessage("collapseEdge: vertexIdx=%d, vertexIdx=%d", fromEdgeVertexHandle.idx(), toEdgeVertexHandle.idx());

        std::set<OMT::VHandle> commonVertexSet;

        std::set<OMT::HEHandle> edgeFaceSet;
        std::set<OMT::EHandle> deleteEdgeSet;
        std::set<OMT::VHandle> modifiedVertexSet;

        // collect edges to form face
        std::set<OMT::VHandle> fromVertexOneRingSet;
        for (OMT::VOHEIter voh_it = mesh->voh_begin(fromEdgeVertexHandle); voh_it != mesh->voh_end(fromEdgeVertexHandle); ++voh_it) {
            fromVertexOneRingSet.insert(mesh->to_vertex_handle(*voh_it));
//            wxLogMessage("fromVertexOneRingSet: edgeIdx=%d, vertexIdx=%d", mesh->edge_handle(*voh_it).idx(), mesh->to_vertex_handle(*voh_it).idx());
        }
//        for (OMT::VFIter vf_it = mesh->vf_begin(fromEdgeVertexHandle); vf_it != mesh->vf_end(fromEdgeVertexHandle); ++vf_it) {
//            int idx[3];
//            OMT::FVIter fv_it = mesh->fv_iter(*vf_it);
//            idx[0] = fv_it->idx();++fv_it;
//            idx[1] = fv_it->idx();++fv_it;
//            idx[2] = fv_it->idx();
//            wxLogMessage("fromVertexOneRingSet Face: vertexIdx=%d, vertexIdx=%d, vertexIdx=%d", idx[0], idx[1], idx[2]);
//        }
        wxLogMessage("Debug - 2-2");

        std::set<OMT::VHandle> toVertexOneRingSet;
        for (OMT::VOHEIter voh_it = mesh->voh_begin(toEdgeVertexHandle); voh_it != mesh->voh_end(toEdgeVertexHandle); ++voh_it) {
            toVertexOneRingSet.insert(mesh->to_vertex_handle(*voh_it));
//            wxLogMessage("toVertexOneRingSet: edgeIdx=%d, vertexIdx=%d", mesh->edge_handle(*voh_it).idx(), mesh->to_vertex_handle(*voh_it).idx());
        }
//        for (OMT::VFIter vf_it = mesh->vf_begin(toEdgeVertexHandle); vf_it != mesh->vf_end(toEdgeVertexHandle); ++vf_it) {
//            int idx[3];
//            OMT::FVIter fv_it = mesh->fv_iter(*vf_it);
//            idx[0] = fv_it->idx();++fv_it;
//            idx[1] = fv_it->idx();++fv_it;
//            idx[2] = fv_it->idx();
//            wxLogMessage("toVertexOneRingSet Face: vertexIdx=%d, vertexIdx=%d, vertexIdx=%d", idx[0], idx[1], idx[2]);
//        }
        set_intersection(fromVertexOneRingSet.begin(), fromVertexOneRingSet.end(), toVertexOneRingSet.begin(), toVertexOneRingSet.end(),
                         std::inserter(commonVertexSet, commonVertexSet.begin()));

        for (OMT::VHandle commomVertexHandle: commonVertexSet) {
            OMT::HEHandle fromVertexToCommonHalfedge = mesh->find_halfedge(fromEdgeVertexHandle, commomVertexHandle);
            OMT::HEHandle commonVertexToFromHalfedge = mesh->opposite_halfedge_handle(fromVertexToCommonHalfedge);
            deleteEdge(mesh, mesh->property(m_heapIdxProp, fromVertexToCommonHalfedge));
            deleteEdge(mesh, mesh->property(m_heapIdxProp, commonVertexToFromHalfedge));
        }
        deleteEdge(mesh, mesh->property(m_heapIdxProp, halfedgeHandle));
        deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->opposite_halfedge_handle(halfedgeHandle)));
        wxLogMessage("Debug - 2-3");


//        mesh->request_face_status();
//        mesh->request_edge_status();
//        mesh->request_vertex_status();

//        mesh->collapse(halfedgeHandle);
        std::vector<OMT::HEHandle> fromEdgeVertexOneRingList;
        OMT::VOHEIter vohFrom_it = mesh->voh_iter(fromEdgeVertexHandle);
        while(mesh->to_vertex_handle(*vohFrom_it) != toEdgeVertexHandle) {
            ++vohFrom_it;
        }
        do {
            OMT::VHandle currentVertexHandle = mesh->to_vertex_handle(*vohFrom_it);
            ++vohFrom_it;
            OMT::VHandle nextVertexHandle = mesh->to_vertex_handle(*vohFrom_it);
            fromEdgeVertexOneRingList.push_back(mesh->find_halfedge(currentVertexHandle, nextVertexHandle));
        }while(mesh->to_vertex_handle(*vohFrom_it) != toEdgeVertexHandle);

        mesh->delete_vertex(fromEdgeVertexHandle);
        for (int i = 0; i < fromEdgeVertexOneRingList.size()-1; ++i) {
            int idx = i+1;
            if (mesh->to_vertex_handle(fromEdgeVertexOneRingList[idx]) == toEdgeVertexHandle) {
                continue;
            }
            if (!mesh->find_halfedge(toEdgeVertexHandle, mesh->to_vertex_handle(fromEdgeVertexOneRingList[idx])).is_valid() || !mesh->find_halfedge(toEdgeVertexHandle, mesh->to_vertex_handle(fromEdgeVertexOneRingList[i])).is_valid()) {
                OMT::FHandle fHandle = mesh->add_face(toEdgeVertexHandle, mesh->to_vertex_handle(fromEdgeVertexOneRingList[i]), mesh->to_vertex_handle(fromEdgeVertexOneRingList[idx]));
                if(!fHandle.is_valid()) {
                    fHandle = mesh->add_face(toEdgeVertexHandle, mesh->to_vertex_handle(fromEdgeVertexOneRingList[idx]), mesh->to_vertex_handle(fromEdgeVertexOneRingList[i]));
                    if (!fHandle.is_valid()) {
                        wxLogMessage("Create Face failed");
                        return false;
                    }
                }
                wxLogMessage("Create Face State %d", fHandle.is_valid());
            }
        }
        wxLogMessage("Debug - 2-4");


        for (OMT::VOHEIter voh_it = mesh->voh_begin(toEdgeVertexHandle); voh_it != mesh->voh_end(toEdgeVertexHandle); ++voh_it) {
            updateVertexK(mesh, mesh->to_vertex_handle(*voh_it));
//            wxLogMessage("toVertexOneRingSet 1: edgeIdx=%d, vertexIdx=%d", mesh->edge_handle(*voh_it).idx(), mesh->to_vertex_handle(*voh_it).idx());
        }
        wxLogMessage("Debug - 2-4-1");
        updateVertexK(mesh, toEdgeVertexHandle);
//        for (OMT::VFIter vf_it = mesh->vf_begin(toEdgeVertexHandle); vf_it != mesh->vf_end(toEdgeVertexHandle); ++vf_it) {
//            int idx[3];
//            OMT::FVIter fv_it = mesh->fv_iter(*vf_it);
//            idx[0] = fv_it->idx();++fv_it;
//            idx[1] = fv_it->idx();++fv_it;
//            idx[2] = fv_it->idx();
//            wxLogMessage("toVertexOneRingSet Face 1: vertexIdx=%d, vertexIdx=%d, vertexIdx=%d", idx[0], idx[1], idx[2]);
//        }
        wxLogMessage("Debug - 2-4-2");
        for (OMT::VOHEIter voh_it = mesh->voh_begin(toEdgeVertexHandle); voh_it != mesh->voh_end(toEdgeVertexHandle); ++voh_it) {
            for (OMT::VOHEIter outer_voh_it = mesh->voh_begin(mesh->to_vertex_handle(*voh_it)); outer_voh_it != mesh->voh_end(mesh->to_vertex_handle(*voh_it)); ++outer_voh_it) {
                changeEdge(mesh, mesh->property(m_heapIdxProp, *outer_voh_it));
                changeEdge(mesh, mesh->property(m_heapIdxProp, mesh->opposite_halfedge_handle(*outer_voh_it)));
            }
            changeEdge(mesh, mesh->property(m_heapIdxProp, *voh_it));
        }
        wxLogMessage("Debug - 2-5");

    }
    else {
        // temporary
        wxLogMessage("Not collapsable");
        m_discardedErrorMetric.push_back(m_pQueue[mesh->property(m_heapIdxProp, halfedgeHandle)]);
        m_discardedErrorMetric.push_back(m_pQueue[mesh->property(m_heapIdxProp, mesh->opposite_halfedge_handle(halfedgeHandle))]);
        deleteEdge(mesh, mesh->property(m_heapIdxProp, halfedgeHandle));
        deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->opposite_halfedge_handle(halfedgeHandle)));
    }
    wxLogMessage("Debug - 3");
    return true;

//    wxLogMessage("Elapsed: %f", timer.calculateDiffMilliSecondTime());
}

void SkeletonExtraction::initEdgeMetric(Tri_Mesh *mesh) {
    m_cost.resize(mesh->n_halfedges());
    for (OMT::EIter e_it = mesh->edges_begin(); e_it != mesh->edges_end(); ++e_it) {
        m_cost[mesh->halfedge_handle(*e_it, 0).idx()] = computeErrorMetric(mesh, mesh->halfedge_handle(*e_it, 0)).m_metric;
        m_cost[mesh->halfedge_handle(*e_it, 1).idx()] = computeErrorMetric(mesh, mesh->halfedge_handle(*e_it, 1)).m_metric;
    }
    m_isDiscard.resize(mesh->n_halfedges(), false);
}
void SkeletonExtraction::easierCollapseEdge(Tri_Mesh *mesh) {
//    Timer timer;
//    timer.updateCurrentTime();
    double minK = 1e10;
    OMT::HEHandle optimalHE;
    for (OMT::EIter e_it = mesh->edges_begin();e_it != mesh->edges_end();++e_it) {
        if (!m_isDiscard[mesh->halfedge_handle(*e_it, 0).idx()] && isCollapsable(mesh, mesh->halfedge_handle(*e_it, 0))) {
            double em = m_cost[mesh->halfedge_handle(*e_it, 0).idx()];
            if (em < minK) {
                minK = em;
                optimalHE = mesh->halfedge_handle(*e_it, 0);
            }
        }
    }
    OMT::VHandle toEdgeVertexHandle = mesh->to_vertex_handle(optimalHE);
    OMT::VHandle fromEdgeVertexHandle = mesh->from_vertex_handle(optimalHE);

    std::set<OMT::HEHandle> oneRingSet;
    std::vector<OMT::HEHandle> fromEdgeVertexOneRingList;
    for (OMT::VOHEIter voh_it = mesh->voh_begin(fromEdgeVertexHandle); voh_it != mesh->voh_end(fromEdgeVertexHandle); ++voh_it) {
        oneRingSet.insert(*voh_it);
        fromEdgeVertexOneRingList.push_back(*voh_it);
    }
    for (OMT::VOHEIter voh_it = mesh->voh_begin(toEdgeVertexHandle); voh_it != mesh->voh_end(toEdgeVertexHandle); ++voh_it) {
        oneRingSet.insert(*voh_it);
    }

//    mesh->collapse(optimalHE);
    mesh->delete_vertex(fromEdgeVertexHandle);
    for (int i = 0; i < fromEdgeVertexOneRingList.size(); ++i) {
        OMT::HEHandle he = fromEdgeVertexOneRingList[i];
        if (mesh->to_vertex_handle(he) != toEdgeVertexHandle) {
            if (!mesh->find_halfedge(toEdgeVertexHandle, mesh->to_vertex_handle(he)).is_valid()) {
                int idx = i==0?-1:i-1;
                OMT::FHandle fHandle = mesh->add_face(toEdgeVertexHandle, mesh->to_vertex_handle(he), mesh->to_vertex_handle(fromEdgeVertexOneRingList[idx]));
                if(!fHandle.is_valid()) {
                    wxLogMessage("Create Face failed");
                }
            }
        }
    }


    for (OMT::VOHEIter voh_it = mesh->voh_begin(toEdgeVertexHandle); voh_it != mesh->voh_end(toEdgeVertexHandle); ++voh_it) {
        updateVertexK(mesh, mesh->to_vertex_handle(*voh_it));
//            wxLogMessage("toVertexOneRingSet 1: edgeIdx=%d, vertexIdx=%d", mesh->edge_handle(*voh_it).idx(), mesh->to_vertex_handle(*voh_it).idx());
    }
    updateVertexK(mesh, toEdgeVertexHandle);

    for (auto he : oneRingSet) {
        if (mesh->find_halfedge(toEdgeVertexHandle, mesh->to_vertex_handle(he)).is_valid()) {
            m_cost[he.idx()] = computeErrorMetric(mesh, he).m_metric;
            m_cost[mesh->opposite_halfedge_handle(he).idx()] = computeErrorMetric(mesh, he).m_metric;
        } else {
            m_isDiscard[he.idx()] = true;
            m_isDiscard[mesh->opposite_halfedge_handle(he).idx()] = true;
        }
    }

}

SKErrorMetric SkeletonExtraction::computeErrorMetric(Tri_Mesh *mesh, OMT::HEHandle halfedgeHandle) {
    // get one-ring
    OMT::VHandle toEdgeVertexHandle = mesh->to_vertex_handle(halfedgeHandle);
    OMT::Point toVertexPoint = mesh->point(toEdgeVertexHandle);
    Eigen::Vector4d toP(toVertexPoint[0], toVertexPoint[1], toVertexPoint[2], 1.0);
    OMT::VHandle fromEdgeVertexHandle = mesh->from_vertex_handle(halfedgeHandle);
    OMT::Point fromVertexPoint = mesh->point(fromEdgeVertexHandle);
    Eigen::Vector4d fromP(fromVertexPoint[0], fromVertexPoint[1], fromVertexPoint[2], 1.0);
//    double F = (fromP.transpose() * mesh->property(m_K, toEdgeVertexHandle) * fromP) + (toP.transpose() * mesh->property(m_K, fromEdgeVertexHandle) * toP);
    double Fa = (fromP.transpose() * mesh->property(m_K, toEdgeVertexHandle) * fromP);
    Fa += (toP.transpose() * mesh->property(m_K, fromEdgeVertexHandle) * toP);

    double Fb = 0.0;
    for (OMT::VOHEIter voh_it = mesh->voh_begin(fromEdgeVertexHandle); voh_it !=mesh->voh_end(fromEdgeVertexHandle); ++voh_it) {
        OMT::VHandle kVertexHandle = mesh->to_vertex_handle(*voh_it);
        OMT::Point kVertexPoint = mesh->point(kVertexHandle);
        Eigen::Vector4d kP(kVertexPoint[0], kVertexPoint[1], kVertexPoint[2], 1.0);
        Fb += (kP - fromP).norm();
    }
    Fb *= (fromP - toP).norm();

    double F = m_wa * Fa + m_wb * Fb;
    return { halfedgeHandle, F };
}

void SkeletonExtraction::simplifyMesh() {
	int collapsePerPortion = m_operateMesh->n_vertices() / 10 * 2;
	size_t halfedgeCount = m_operateMesh->n_halfedges();
	if (!m_isInitializedQ) {
        initErrorMetric(m_operateMesh);
        for (OMT::EIter e_it = m_operateMesh->edges_sbegin(); e_it != m_operateMesh->edges_end(); ++e_it) {
            insertEdge(m_operateMesh, m_operateMesh->halfedge_handle(*e_it, 0));
            insertEdge(m_operateMesh, m_operateMesh->halfedge_handle(*e_it, 1));
        }
        m_isInitializedQ = true;
	}
//    initEdgeMetric(m_operateMesh);
	// FIXME 1 collapsePerPortion
	for (size_t i = 0; i < 10; ++i) {
	    if (m_currentTailIdx + m_discardedErrorMetric.size() <= 200) {
	        break;
	    }
	    wxLogMessage("Collapsed to %u", i);
        wxLogMessage("Current priority queue %d", m_currentTailIdx);
		SKErrorMetric em = getTopEdge(m_operateMesh);
		if (!collapseEdge(m_operateMesh, em.m_halfedgeHandle)) {
		    break;
		}
//        easierCollapseEdge(m_operateMesh);
	}
}