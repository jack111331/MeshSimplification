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
        m_wL = 1.1;
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
    return !mesh->status(mesh->face_handle(halfedgeHandle)).deleted() && !mesh->status(mesh->face_handle(mesh->opposite_halfedge_handle(halfedgeHandle))).deleted();
}

void SkeletonExtraction::propagateToTop(Tri_Mesh *mesh, int heapIdx) {
    SKErrorMetric em = computeErrorMetric(mesh, m_pQueue[heapIdx].m_edgeHandle);
    while (heapIdx > 1) {
        if (m_pQueue[heapIdx] < m_pQueue[heapIdx / 2]) {
            SKErrorMetric temp = m_pQueue[heapIdx];
            m_pQueue[heapIdx] = m_pQueue[heapIdx / 2];
            mesh->property(m_heapIdxProp, m_pQueue[heapIdx].m_edgeHandle) = heapIdx;
            m_pQueue[heapIdx / 2] = temp;
            heapIdx /= 2;
        }
        else {
            break;
        }
    }
    mesh->property(m_heapIdxProp, em.m_edgeHandle) = heapIdx;
}
void SkeletonExtraction::propagateToBottom(Tri_Mesh *mesh, int heapIdx) {
    SKErrorMetric em = computeErrorMetric(mesh, m_pQueue[heapIdx].m_edgeHandle);
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
                mesh->property(m_heapIdxProp, m_pQueue[minIdx].m_edgeHandle) = heapIdx;
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
    mesh->property(m_heapIdxProp, em.m_edgeHandle) = heapIdx;
}

void SkeletonExtraction::insertEdge(Tri_Mesh *mesh, OMT::EHandle edgeHandle) {
    SKErrorMetric em = computeErrorMetric(mesh, edgeHandle);
    m_pQueue[++m_currentTailIdx] = em;
    mesh->property(m_heapIdxProp, edgeHandle) = m_currentTailIdx;

    propagateToTop(mesh, m_currentTailIdx);
}

SKErrorMetric SkeletonExtraction::getTopEdge(Tri_Mesh *mesh) {
    SKErrorMetric ret = m_pQueue[1];

    return ret;
}

void SkeletonExtraction::deleteEdge(Tri_Mesh *mesh, int heapIdx) {
    if (heapIdx == -1) {
        return;
    }
    mesh->property(m_heapIdxProp, m_pQueue[heapIdx].m_edgeHandle) = -1;

    m_pQueue[heapIdx] = m_pQueue[m_currentTailIdx--];
    mesh->property(m_heapIdxProp, m_pQueue[heapIdx].m_edgeHandle) = heapIdx;

    propagateToTop(mesh, heapIdx);
    propagateToBottom(mesh, heapIdx);
}

void SkeletonExtraction::changeEdge(Tri_Mesh *mesh, int heapIdx) {
    if(heapIdx == -1) {
        return;
    }
    SKErrorMetric newErrorMetric = computeErrorMetric(mesh, m_pQueue[heapIdx].m_edgeHandle);
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
        updateVertexK(mesh, *v_it);
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
//    wxLogMessage("Debug - 1");
//    wxLogMessage("Debug - 2 : is_valid %d", halfedgeHandle.is_valid());
//    bool toLoop = mesh->next_halfedge_handle(mesh->next_halfedge_handle(halfedgeHandle)) == halfedgeHandle;
//    OMT::HEHandle oppositeHalfedgeHandle = mesh->opposite_halfedge_handle(halfedgeHandle);
//    bool fromLoop = mesh->next_halfedge_handle(mesh->next_halfedge_handle(oppositeHalfedgeHandle)) == oppositeHalfedgeHandle;
//    if (toLoop) {
//        // this infer that current from vertex's connectivity is 2, dont delete them
//        deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(mesh->next_halfedge_handle(halfedgeHandle))));
//        deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(halfedgeHandle)));
//    } else if (fromLoop) {
//        // this infer that current from vertex's connectivity is 2, dont delete them
//        deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(mesh->next_halfedge_handle(oppositeHalfedgeHandle))));
//        deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(oppositeHalfedgeHandle)));
//    } else if (isCollapsable(mesh, halfedgeHandle)) {
//    if (isCollapsable(mesh, halfedgeHandle)) {
    bool isNoFaceAround = false;
    if (mesh->is_collapse_ok(halfedgeHandle)) {
        if (isCollapsable(m_operateMesh, halfedgeHandle)) {
            // FIXME it may collapse to itself.... means from and to vertex is both itself
//        wxLogMessage("Debug - 2-1");

            OMT::VHandle toEdgeVertexHandle = mesh->to_vertex_handle(halfedgeHandle);
            OMT::VHandle fromEdgeVertexHandle = mesh->from_vertex_handle(halfedgeHandle);
            if (fromEdgeVertexHandle == toEdgeVertexHandle) {
                deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(halfedgeHandle)));
                mesh->delete_edge(mesh->edge_handle(halfedgeHandle));
                return true;
            }
//        wxLogMessage("collapseEdge: vertexIdx=%d, vertexIdx=%d", fromEdgeVertexHandle.idx(), toEdgeVertexHandle.idx());

            std::set<OMT::VHandle> commonVertexSet;

            std::set<OMT::HEHandle> edgeFaceSet;
            std::set<OMT::EHandle> deleteEdgeSet;
            std::set<OMT::VHandle> modifiedVertexSet;

            std::set<OMT::EHandle> allEdgeSet;


            // collect edges to form face
            std::set<OMT::VHandle> fromVertexOneRingSet;
            for (OMT::VOHEIter voh_it = mesh->voh_begin(fromEdgeVertexHandle); voh_it != mesh->voh_end(fromEdgeVertexHandle); ++voh_it) {
                fromVertexOneRingSet.insert(mesh->to_vertex_handle(*voh_it));
                allEdgeSet.insert(mesh->edge_handle(*voh_it));
//            wxLogMessage("fromVertexOneRingSet: edgeIdx=%d, vertexIdx=%d", mesh->edge_handle(*voh_it).idx(), mesh->to_vertex_handle(*voh_it).idx());
//            OMT::FHandle fHandle = mesh->face_handle(*voh_it);
//            for (OMT::FEIter fe_it = mesh->fe_begin(fHandle); fe_it != mesh->fe_end(fHandle); ++fe_it) {
//                wxLogMessage("fromVertexOneRingSet face: edgeIdx=%d, vertexIdx=%d, %d", fe_it->idx(), mesh->from_vertex_handle(mesh->halfedge_handle(*fe_it, 0)).idx(), mesh->to_vertex_handle(mesh->halfedge_handle(*fe_it, 0)).idx());
//            }
//            fHandle = mesh->face_handle(mesh->opposite_halfedge_handle(*voh_it));
//            for (OMT::FEIter fe_it = mesh->fe_begin(fHandle); fe_it != mesh->fe_end(fHandle); ++fe_it) {
//                wxLogMessage("fromVertexOneRingSet face opposite: edgeIdx=%d, vertexIdx=%d, %d", fe_it->idx(), mesh->from_vertex_handle(mesh->halfedge_handle(*fe_it, 0)).idx(), mesh->to_vertex_handle(mesh->halfedge_handle(*fe_it, 0)).idx());
//            }
            }
//        wxLogMessage("Debug - 2-2");

            std::set<OMT::VHandle> toVertexOneRingSet;
            for (OMT::VOHEIter voh_it = mesh->voh_begin(toEdgeVertexHandle); voh_it != mesh->voh_end(toEdgeVertexHandle); ++voh_it) {
                toVertexOneRingSet.insert(mesh->to_vertex_handle(*voh_it));
                allEdgeSet.insert(mesh->edge_handle(*voh_it));
//            wxLogMessage("toVertexOneRingSet: edgeIdx=%d, vertexIdx=%d", mesh->edge_handle(*voh_it).idx(), mesh->to_vertex_handle(*voh_it).idx());
//            OMT::FHandle fHandle = mesh->face_handle(*voh_it);
//            if (fHandle.is_valid()) {
//                for (OMT::FEIter fe_it = mesh->fe_begin(fHandle); fe_it != mesh->fe_end(fHandle); ++fe_it) {
//                    wxLogMessage("toVertexOneRingSet face: edgeIdx=%d, vertexIdx=%d, %d", fe_it->idx(), mesh->from_vertex_handle(mesh->halfedge_handle(*fe_it, 0)).idx(), mesh->to_vertex_handle(mesh->halfedge_handle(*fe_it, 0)).idx());
//                }
//            }
//            fHandle = mesh->face_handle(mesh->opposite_halfedge_handle(*voh_it));
//            if (fHandle.is_valid()) {
//                for (OMT::FEIter fe_it = mesh->fe_begin(fHandle); fe_it != mesh->fe_end(fHandle); ++fe_it) {
//                    wxLogMessage("toVertexOneRingSet face opposite: edgeIdx=%d, vertexIdx=%d, %d", fe_it->idx(), mesh->from_vertex_handle(mesh->halfedge_handle(*fe_it, 0)).idx(), mesh->to_vertex_handle(mesh->halfedge_handle(*fe_it, 0)).idx());
//                }
//            }
//            else {
//                fHandle = mesh->face_handle(mesh->opposite_halfedge_handle(*voh_it));
//                for (OMT::FEIter fe_it = mesh->fe_begin(fHandle); fe_it != mesh->fe_end(fHandle); ++fe_it) {
//                    wxLogMessage("toVertexOneRingSet face: edgeIdx=%d, vertexIdx=%d, %d", fe_it->idx(), mesh->from_vertex_handle(mesh->halfedge_handle(*fe_it, 0)).idx(), mesh->to_vertex_handle(mesh->halfedge_handle(*fe_it, 0)).idx());
//                }
//            }
            }
            set_intersection(fromVertexOneRingSet.begin(), fromVertexOneRingSet.end(), toVertexOneRingSet.begin(), toVertexOneRingSet.end(),
                             std::inserter(commonVertexSet, commonVertexSet.begin()));

            //        for (OMT::VHandle fromVertexOneRingHandle: fromVertexOneRingSet) {
            //            OMT::HEHandle fromVertexToCommonHalfedge = mesh->find_halfedge(fromEdgeVertexHandle, fromVertexOneRingHandle);
            //            wxLogMessage("Delete edge %d", mesh->edge_handle(fromVertexToCommonHalfedge).idx());
            //            deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(fromVertexToCommonHalfedge)));
            //        }
//        std::set<OMT::EHandle> deletedEdgeSet;
//        for (OMT::VHandle commonVertex: commonVertexSet) {
//            OMT::HEHandle fromVertexToCommonHalfedge = mesh->find_halfedge(fromEdgeVertexHandle, commonVertex);
////            wxLogMessage("Delete edge %d", mesh->edge_handle(fromVertexToCommonHalfedge).idx());
//            deletedEdgeSet.insert(mesh->edge_handle(fromVertexToCommonHalfedge));
//            deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(fromVertexToCommonHalfedge)));
//        }
//        deletedEdgeSet.insert(mesh->edge_handle(halfedgeHandle));
//        deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(halfedgeHandle)));
//        wxLogMessage("Debug - 2-3");
            // FIXME next_halfedge bug here


            //        mesh->request_face_status();
            //        mesh->request_edge_status();
            //        mesh->request_vertex_status();

            mesh->collapse(halfedgeHandle);
            // FIXME need post process? delete edge connected backward, edge connect twice

            //        std::vector<OMT::HEHandle> fromEdgeVertexOneRingList;
            //        OMT::VOHEIter vohFrom_it = mesh->voh_iter(fromEdgeVertexHandle);
            //        while(mesh->to_vertex_handle(*vohFrom_it) != toEdgeVertexHandle) {
            //            ++vohFrom_it;
            //        }
            //        do {
            //            OMT::VHandle currentVertexHandle = mesh->to_vertex_handle(*vohFrom_it);
            //            ++vohFrom_it;
            //            OMT::VHandle nextVertexHandle = mesh->to_vertex_handle(*vohFrom_it);
            //            fromEdgeVertexOneRingList.push_back(mesh->find_halfedge(currentVertexHandle, nextVertexHandle));
            //        }while(mesh->to_vertex_handle(*vohFrom_it) != toEdgeVertexHandle);
            //
            //        mesh->delete_vertex(fromEdgeVertexHandle);
            //        for (int i = 0; i < fromEdgeVertexOneRingList.size()-1; ++i) {
            //            int idx = i+1;
            //            if (mesh->to_vertex_handle(fromEdgeVertexOneRingList[idx]) == toEdgeVertexHandle) {
            //                continue;
            //            }
            //            OMT::FHandle fHandle = mesh->add_face(toEdgeVertexHandle, mesh->to_vertex_handle(fromEdgeVertexOneRingList[i]), mesh->to_vertex_handle(fromEdgeVertexOneRingList[idx]));
            //            if(!fHandle.is_valid()) {
            //                fHandle = mesh->add_face(toEdgeVertexHandle, mesh->to_vertex_handle(fromEdgeVertexOneRingList[idx]), mesh->to_vertex_handle(fromEdgeVertexOneRingList[i]));
            //                if (!fHandle.is_valid()) {
            //                    wxLogMessage("Create Face failed");
            //                    return false;
            //                } else {
            //                    OMT::FEIter fe_it = mesh->fe_iter(fHandle);
            //                    while(*fe_it == mesh->edge_handle(fromEdgeVertexOneRingList[idx]) || *fe_it == mesh->edge_handle(fromEdgeVertexOneRingList[i])) {
            //                        ++fe_it;
            //                    }
            //                    wxLogMessage("insert edge %d", fe_it->idx());
            //                    insertEdge(mesh, *fe_it);
            //                }
            //            } else {
            //                OMT::FEIter fe_it = mesh->fe_iter(fHandle);
            //                while(*fe_it == mesh->edge_handle(fromEdgeVertexOneRingList[idx]) || *fe_it == mesh->edge_handle(fromEdgeVertexOneRingList[i])) {
            //                    ++fe_it;
            //                }
            //                wxLogMessage("insert edge %d", fe_it->idx());
            //
            //                insertEdge(mesh, *fe_it);
            //            }
            ////            wxLogMessage("Create Face State %d", fHandle.is_valid());
            //        }
//        wxLogMessage("Debug - 2-4");
//        for(auto deletedEdge : deletedEdgeSet) {
//            wxLogMessage("Supposed to Deleted Edge idx=%d, valid=%d, isDeleted=%d", deletedEdge.idx(), deletedEdge.is_valid(), mesh->status(deletedEdge).deleted());
//        }
//        int deletedEdgeInAllEdge = 0;
// Delete selfedge
//        for (OMT::VOHEIter voh_it = mesh->voh_begin(toEdgeVertexHandle); voh_it != mesh->voh_end(toEdgeVertexHandle); ++voh_it) {
//            if (mesh->from_vertex_handle(*voh_it) == mesh->to_vertex_handle(*voh_it)) {
//                mesh->delete_edge(mesh->edge_handle(*voh_it));
//            }
//        }
// Delete multiedge
//        for(auto allOneRingEdge : allEdgeSet) {
//            if (!mesh->face_handle(mesh->halfedge_handle(allOneRingEdge, 0)).is_valid()) {
//                mesh->delete_edge(allOneRingEdge);
//            }
//        }

            for(auto allOneRingEdge : allEdgeSet) {
//            wxLogMessage("All Edge idx=%d, valid=%d, isDeleted=%d", allOneRingEdge.idx(), allOneRingEdge.is_valid(), mesh->status(allOneRingEdge).deleted());
                if (mesh->status(allOneRingEdge).deleted()) {
                    deleteEdge(mesh, mesh->property(m_heapIdxProp, allOneRingEdge));
//                deletedEdgeInAllEdge++;
                }
            }
//        if(deletedEdgeInAllEdge != deletedEdgeSet.size()) {
//            wxLogMessage("Deleted edge inconsistent=%d, %d", deletedEdgeInAllEdge, deletedEdgeSet.size());
//        }
            for (OMT::VOHEIter voh_it = mesh->voh_begin(toEdgeVertexHandle); voh_it != mesh->voh_end(toEdgeVertexHandle); ++voh_it) {
                updateVertexK(mesh, mesh->to_vertex_handle(*voh_it));
            }
            updateVertexK(mesh, toEdgeVertexHandle);
            std::set<OMT::EHandle> changeEdgeSet;
//        for (OMT::VOHEIter voh_it = mesh->voh_begin(toEdgeVertexHandle); voh_it != mesh->voh_end(toEdgeVertexHandle); ++voh_it) {
//            for (OMT::VOHEIter outer_voh_it = mesh->voh_begin(mesh->to_vertex_handle(*voh_it)); outer_voh_it != mesh->voh_end(mesh->to_vertex_handle(*voh_it)); ++outer_voh_it) {
////                wxLogMessage("Debug - 2-4-3");
//                changeEdgeSet.insert(mesh->edge_handle(*outer_voh_it));
////                wxLogMessage("Debug - 2-4-4");
//            }
//            changeEdgeSet.insert(mesh->edge_handle(*voh_it));
////            wxLogMessage("Debug - 2-4-5");
//        }
//        for(auto eHandle: changeEdgeSet) {
//            changeEdge(mesh, mesh->property(m_heapIdxProp, eHandle));
//            wxLogMessage("change edge %d", eHandle.idx());
//        }
            for (OMT::VOHEIter voh_it = mesh->voh_begin(toEdgeVertexHandle); voh_it != mesh->voh_end(toEdgeVertexHandle); ++voh_it) {
                for (OMT::VOHEIter outer_voh_it = mesh->voh_begin(mesh->to_vertex_handle(*voh_it)); outer_voh_it != mesh->voh_end(mesh->to_vertex_handle(*voh_it)); ++outer_voh_it) {
                    changeEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(*outer_voh_it)));
                }
                changeEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(*voh_it)));
            }
//        wxLogMessage("Debug - 2-5");
        } else {
            deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(halfedgeHandle)));
            isNoFaceAround = true;
        }
    } else {
        // temporary
        wxLogMessage("Not collapsable");
        m_discardedErrorMetric.push_back(m_pQueue[mesh->property(m_heapIdxProp, mesh->edge_handle(halfedgeHandle))]);
        deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(halfedgeHandle)));
    }
    if (isNoFaceAround && m_discardedErrorMetric.empty() && m_currentTailIdx == 0) {
        return false;
    }
//    wxLogMessage("Debug - 3");
    return true;

//    wxLogMessage("Elapsed: %f", timer.calculateDiffMilliSecondTime());
}

void SkeletonExtraction::initEdgeMetric(Tri_Mesh *mesh) {
    m_cost.resize(mesh->n_edges());
    for (OMT::EIter e_it = mesh->edges_begin(); e_it != mesh->edges_end(); ++e_it) {
        m_cost[e_it->idx()] = computeErrorMetric(mesh, *e_it).m_metric;
    }
    m_isDiscard.resize(mesh->n_edges(), false);
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
            m_cost[he.idx()] = computeErrorMetric(mesh, mesh->edge_handle(he)).m_metric;
        } else {
            m_isDiscard[he.idx()] = true;
        }
    }

}

SKErrorMetric SkeletonExtraction::computeErrorMetric(Tri_Mesh *mesh, OMT::EHandle edgeHandle) {
    // get one-ring
    OMT::HEHandle halfedgeHandle = mesh->halfedge_handle(edgeHandle, 0);
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
    return { edgeHandle, F };
}

void SkeletonExtraction::simplifyMesh() {
	int collapsePerPortion = m_operateMesh->n_vertices() / 10 * 2;
	size_t halfedgeCount = m_operateMesh->n_halfedges();
	if (!m_isInitializedQ) {
        initErrorMetric(m_operateMesh);
        for (OMT::EIter e_it = m_operateMesh->edges_sbegin(); e_it != m_operateMesh->edges_end(); ++e_it) {
            insertEdge(m_operateMesh, *e_it);
        }
        m_isInitializedQ = true;
	}
//    initEdgeMetric(m_operateMesh);
	// FIXME 1 collapsePerPortion
	int i = 0;
	while(true) {
	    wxLogMessage("Collapsed to %u", i++);
        wxLogMessage("Current priority queue %d", m_currentTailIdx);
        if(m_currentTailIdx < 1) {
//        wxLogMessage("Here?");
            if(m_lastDiscardSize == m_discardedErrorMetric.size()) {
                break;
            }
            m_lastDiscardSize = m_discardedErrorMetric.size();
            for(auto em : m_discardedErrorMetric) {
                if (!m_operateMesh->status(em.m_edgeHandle).deleted()) {
                    insertEdge(m_operateMesh, em.m_edgeHandle);
                }
            }
            m_discardedErrorMetric.clear();
        }
        if(m_currentTailIdx == 0) {
            break;
        }
		SKErrorMetric em = getTopEdge(m_operateMesh);
		if (!collapseEdge(m_operateMesh, m_operateMesh->halfedge_handle(em.m_edgeHandle, 0))) {
		    wxLogMessage("Collapse to end");
		    break;
		}
//        easierCollapseEdge(m_operateMesh);
	}
}