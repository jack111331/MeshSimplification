#include <Timer.h>
#include "Simplification.h"
#include "Eigen/Dense"

void SuitableVertexExtraction::propagateToTop(Tri_Mesh *mesh, int heapIdx) {
    ErrorMetric em = computeErrorMetric(mesh, m_pQueue[heapIdx].m_edgeHandle);
    while (heapIdx > 1) {
        if (m_pQueue[heapIdx] < m_pQueue[heapIdx / 2]) {
            ErrorMetric temp = m_pQueue[heapIdx];
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
void SuitableVertexExtraction::propagateToBottom(Tri_Mesh *mesh, int heapIdx) {
    ErrorMetric em = computeErrorMetric(mesh, m_pQueue[heapIdx].m_edgeHandle);
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
                ErrorMetric temp = m_pQueue[heapIdx];
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

void SuitableVertexExtraction::insertEdge(Tri_Mesh *mesh, OMT::EHandle edgeHandle) {
    ErrorMetric em = computeErrorMetric(mesh, edgeHandle);
    m_pQueue[++m_currentTailIdx] = em;
    mesh->property(m_heapIdxProp, edgeHandle) = m_currentTailIdx;

    propagateToTop(mesh, m_currentTailIdx);
}

ErrorMetric SuitableVertexExtraction::getTopEdge(Tri_Mesh *mesh) {
    ErrorMetric ret = m_pQueue[1];

    return ret;
}

ErrorMetric SuitableVertexExtraction::extractTopEdge(Tri_Mesh *mesh) {
    ErrorMetric ret = m_pQueue[1];

    m_pQueue[1] = m_pQueue[m_currentTailIdx--];
    mesh->property(m_heapIdxProp, m_pQueue[1].m_edgeHandle) = 1;

    propagateToBottom(mesh, 1);
    return ret;
}

void SuitableVertexExtraction::deleteEdge(Tri_Mesh *mesh, int heapIdx) {
    m_pQueue[heapIdx] = m_pQueue[m_currentTailIdx--];
    mesh->property(m_heapIdxProp, m_pQueue[heapIdx].m_edgeHandle) = heapIdx;

    propagateToTop(mesh, heapIdx);
    propagateToBottom(mesh, heapIdx);
}

void SuitableVertexExtraction::changeEdge(Tri_Mesh *mesh, int heapIdx) {
    ErrorMetric newErrorMetric = computeErrorMetric(mesh, m_pQueue[heapIdx].m_edgeHandle);
    if (m_pQueue[heapIdx].m_metric < newErrorMetric.m_metric) {
        m_pQueue[heapIdx] = newErrorMetric;

        propagateToTop(mesh, heapIdx);

    }
    else {
        m_pQueue[heapIdx] = newErrorMetric;

        propagateToBottom(mesh, heapIdx);
    }
}

void SuitableVertexExtraction::initErrorMetric(Tri_Mesh *mesh) {
    // Fix template type
    mesh->add_property(m_Q);
    mesh->add_property(m_heapIdxProp);

    for (OMT::VIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
    {
        // initialize
        updateVertexQ(mesh, v_it.handle());
    }
    /*
    for (OMT::VIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
    {
        // initialize
        mesh->property(Q, *v_it) = XForm<double>();

        OMT::Scalar *pointCoord = mesh->point(v_it).data();
        OMT::VHandle center = v_it.handle();
        OMT::HEHandle h = mesh->voh_begin(center).handle();
        OMT::HEHandle hStop = h;
        do {
            OMT::VHandle v = mesh->to_vertex_handle(h);
            OMT::Point qPoint = mesh->point(v);
            OMT::Scalar *q = qPoint.data();
            h = mesh->next_halfedge_handle(mesh->opposite_halfedge_handle(h));
            OMT::VHandle v = mesh->to_vertex_handle(h);

        } while (h != hStop);
    }
    */
}

void SuitableVertexExtraction::updateVertexQ(Tri_Mesh *mesh, OMT::VHandle vertexHandle) {
    OMT::Scalar *pointCoord = mesh->point(vertexHandle).data();
    OMT::Point vPoint = OMT::Point(pointCoord[0], pointCoord[1], pointCoord[2]);
    mesh->property(m_Q, vertexHandle) = MeshSimplification::XForm<double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (OMT::VFIter vf_it = mesh->vf_begin(vertexHandle); vf_it != mesh->vf_end(vertexHandle); ++vf_it) {
        OMT::Normal normal = mesh->normal(vf_it.handle()); // Face Handle
        normal.normalize();
        // | dot product, % cross product
        double d = -(normal | vPoint);
        mesh->property(m_Q, vertexHandle) = mesh->property(m_Q, vertexHandle) + MeshSimplification::XForm<double>(normal[0] * normal[0], normal[0] * normal[1], normal[0] * normal[2], normal[0] * d,
                                                                                              normal[1] * normal[0], normal[1] * normal[1], normal[1] * normal[2], normal[1] * d,
                                                                                              normal[2] * normal[0], normal[2] * normal[1], normal[2] * normal[2], normal[2] * d,
                                                                                              d * normal[0], d * normal[1], d * normal[2], d * d);
    }
}

void SuitableVertexExtraction::collapseEdge(Tri_Mesh *mesh, OMT::EHandle edgeHandle, MeshSimplification::Vec<4, double> targetVertex) {
//    Timer timer;
//    timer.updateCurrentTime();
    OMT::HEHandle halfedgeHandle = mesh->halfedge_handle(edgeHandle, 0);
    if (mesh->is_collapse_ok(halfedgeHandle)) {
        OMT::VHandle toEdgeVertexHandle = mesh->to_vertex_handle(halfedgeHandle);
        OMT::VHandle fromEdgeVertexHandle = mesh->from_vertex_handle(halfedgeHandle);

        std::set<OMT::HEHandle> edgeFaceSet;
        std::set<OMT::EHandle> deleteEdgeSet;
        std::set<OMT::VHandle> modifiedVertexSet;

        // collect edges to form face
        OMT::VHandle prevHV;
        OMT::HEHandle h = mesh->voh_begin(toEdgeVertexHandle).handle();
        OMT::HEHandle hStop = h;
        do {
            OMT::VHandle v = mesh->to_vertex_handle(h);

            // delete edge element collect
            OMT::HEHandle halfedgeHandle = mesh->find_halfedge(toEdgeVertexHandle, v);
            if (halfedgeHandle.is_valid()) {
                deleteEdgeSet.insert(mesh->edge_handle(halfedgeHandle));
            }
            // change edge and form face element collect
            if (prevHV.is_valid()) {
                OMT::HEHandle halfedgeHandle = mesh->find_halfedge(prevHV, v);
                if (halfedgeHandle.is_valid()) {
                    edgeFaceSet.insert(halfedgeHandle);
                }
            }
            prevHV = v;

            h = mesh->next_halfedge_handle(mesh->opposite_halfedge_handle(h));
        } while (h != hStop);

        if (prevHV.is_valid()) {
            OMT::VHandle v = mesh->to_vertex_handle(h);
            OMT::HEHandle halfedgeHandle = mesh->find_halfedge(prevHV, v);
            if (halfedgeHandle.is_valid()) {
                edgeFaceSet.insert(halfedgeHandle);
            }
        }


        prevHV.reset();
        h = mesh->voh_begin(fromEdgeVertexHandle).handle();
        hStop = h;
        do {
            OMT::VHandle v = mesh->to_vertex_handle(h);
            // delete edge element collect
            OMT::HEHandle halfedgeHandle = mesh->find_halfedge(fromEdgeVertexHandle, v);
            if (halfedgeHandle.is_valid()) {
                deleteEdgeSet.insert(mesh->edge_handle(halfedgeHandle));
            }

            // change edge and form face element collect
            if (prevHV.is_valid()) {
                OMT::HEHandle halfedgeHandle = mesh->find_halfedge(prevHV, v);
                if (halfedgeHandle.is_valid()) {
                    edgeFaceSet.insert(halfedgeHandle);
                }
            }
            prevHV = v;

            h = mesh->next_halfedge_handle(mesh->opposite_halfedge_handle(h));
        } while (h != hStop);
        if (prevHV.is_valid()) {
            OMT::VHandle v = mesh->to_vertex_handle(h);
            OMT::HEHandle halfedgeHandle = mesh->find_halfedge(prevHV, v);
            if (halfedgeHandle.is_valid()) {
                edgeFaceSet.insert(halfedgeHandle);
            }
        }

        // remove edges involved in one of the toEdgeHandle or fromEdgeHandle
        for (auto he_it = edgeFaceSet.begin(); he_it != edgeFaceSet.end();) {
            OMT::HEHandle setHalfEdgeHandle = *he_it;
            OMT::VHandle setToEdgeVertexHandle = mesh->to_vertex_handle(setHalfEdgeHandle);
            OMT::VHandle setFromEdgeVertexHandle = mesh->from_vertex_handle(setHalfEdgeHandle);
            if (setToEdgeVertexHandle == toEdgeVertexHandle || setToEdgeVertexHandle == fromEdgeVertexHandle || setFromEdgeVertexHandle == toEdgeVertexHandle || setFromEdgeVertexHandle == fromEdgeVertexHandle) {
                he_it = edgeFaceSet.erase(he_it);
            }
            else {
                ++he_it;
            }
        }
        // delete edge
        bool isDeleteConnectEdge = false;

        for (auto e : deleteEdgeSet) {
            OMT::HEHandle halfedgeHandle = mesh->halfedge_handle(e, 0);
            OMT::VHandle setToEdgeVertexHandle = mesh->to_vertex_handle(halfedgeHandle);
            OMT::VHandle setFromEdgeVertexHandle = mesh->from_vertex_handle(halfedgeHandle);
            if ((setToEdgeVertexHandle == toEdgeVertexHandle && setFromEdgeVertexHandle == fromEdgeVertexHandle) || (setToEdgeVertexHandle == fromEdgeVertexHandle && setFromEdgeVertexHandle == toEdgeVertexHandle)) {
                if (!isDeleteConnectEdge) {
                    deleteEdge(mesh, mesh->property(m_heapIdxProp, e));
                    isDeleteConnectEdge = true;
                }
            }
            else {
                deleteEdge(mesh, mesh->property(m_heapIdxProp, e));
            }
        }

//        mesh->request_face_status();
//        mesh->request_edge_status();
//        mesh->request_vertex_status();

        mesh->delete_vertex(toEdgeVertexHandle);
        mesh->delete_vertex(fromEdgeVertexHandle);
        OMT::Point pointCoord = OMT::Point(targetVertex[0], targetVertex[1], targetVertex[2]);
        OMT::VHandle newVertex = mesh->add_vertex(pointCoord);
        modifiedVertexSet.insert(newVertex);
        std::set<OMT::EHandle> addBackEdgeSet;
        for (auto &e : edgeFaceSet) {
            OMT::HEHandle setHalfEdgeHandle = e;
            OMT::VHandle setToEdgeVertexHandle = mesh->to_vertex_handle(setHalfEdgeHandle);
            OMT::VHandle setFromEdgeVertexHandle = mesh->from_vertex_handle(setHalfEdgeHandle);
            modifiedVertexSet.insert(setToEdgeVertexHandle);
            modifiedVertexSet.insert(setFromEdgeVertexHandle);
            OMT::FHandle newFace = mesh->add_face(setFromEdgeVertexHandle, newVertex, setToEdgeVertexHandle);
            for (OMT::FEIter fe_it = mesh->fe_begin(newFace); fe_it != mesh->fe_end(newFace); ++fe_it) {
                OMT::EHandle edgeHandle = fe_it.handle();
                if (edgeHandle != mesh->edge_handle(e)) {
                    addBackEdgeSet.insert(edgeHandle);
                }
            }
        }
        for (auto &e : addBackEdgeSet) {
            insertEdge(mesh, e);
        }

        // Update error metric heap
        for (auto v : modifiedVertexSet) {
            updateVertexQ(mesh, v);
        }

        std::set<OMT::EHandle> outEdgeSet;
        for (auto he : edgeFaceSet) {
            OMT::VHandle from_vertex = mesh->from_vertex_handle(he);
            OMT::HEHandle h = mesh->voh_begin(from_vertex).handle();
            OMT::HEHandle hStop = h;
            do {
                OMT::VHandle v = mesh->to_vertex_handle(h);
                if (v != newVertex) {
                    OMT::HEHandle halfedgeHandle = mesh->find_halfedge(from_vertex, v);
                    if (halfedgeHandle.is_valid()) {
                        outEdgeSet.insert(mesh->edge_handle(halfedgeHandle));
                    }
                }
                h = mesh->next_halfedge_handle(mesh->opposite_halfedge_handle(h));
            } while (h != hStop);

        }
        // change edge
        for (auto e : outEdgeSet) {
            changeEdge(mesh, mesh->property(m_heapIdxProp, e));
        }
    }
    else {
        deleteEdge(mesh, mesh->property(m_heapIdxProp, mesh->edge_handle(halfedgeHandle)));
    }
//    wxLogMessage("Elapsed: %f", timer.calculateDiffMilliSecondTime());
}

void SuitableVertexExtraction::flushMesh(Tri_Mesh *mesh) {
    // update all handle
    // std_API_Container_VHandlePointer &   vh_to_update,
    // std_API_Container_HHandlePointer &   hh_to_update,
    //  std_API_Container_FHandlePointer &  fh_to_update,
    std::vector<OMT::VHandle *> vhUpdateList;
    std::vector<OMT::HEHandle *> hhUpdateList;
    std::vector<OMT::FHandle *> fhUpdateList;
    /*
    for (int i = 1; i <= m_currentTailIdx; ++i) {
        hhUpdateList.push_back(&m_pQueue[i].m_halfedgeHandle);
    }
    */
    mesh->garbage_collection<std::vector<OMT::VHandle *> , std::vector<OMT::HEHandle *> , std::vector<OMT::FHandle *> >(vhUpdateList, hhUpdateList, fhUpdateList);

}

/*
double SuitableVertexExtraction::computeErrorMetric(Tri_Mesh *mesh, int vertexIdx) {
    // get one-ring
    OMT::VHandle center = mesh->vertex_handle(vertexIdx);
    OMT::Point pPoint = mesh->point(center);
    OMT::Scalar *p = pPoint.data();

    OMT::HEHandle h = mesh->voh_begin(center).handle();
    OMT::HEHandle hStop = h;
    double errorMatrix[4][4] = {};
    do {
        OMT::VHandle v = mesh->to_vertex_handle(h);
        OMT::Point qPoint = mesh->point(v);
        OMT::Scalar *q = qPoint.data();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                errorMatrix[i][j] += q[i] * q[j];
            }
        }
        h = mesh->next_halfedge_handle(mesh->opposite_halfedge_handle(h));
    } while (h != hStop);
    double ret = 0.0;
    double temp[4] = {};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            temp[i] += errorMatrix[i][j] * p[j];
        }
    }
    for (int i = 0; i < 4; ++i) {
        ret += p[i] * temp[i];
    }
    return ret;
}
*/
ErrorMetric SuitableVertexExtraction::computeErrorMetric(Tri_Mesh *mesh, OMT::EHandle edgeHandle) {
    // get one-ring
    OMT::HEHandle halfedgeHandle = mesh->halfedge_handle(edgeHandle, 0);
    OMT::VHandle toEdgeVertexHandle = mesh->to_vertex_handle(halfedgeHandle);
    OMT::VHandle fromEdgeVertexHandle = mesh->from_vertex_handle(halfedgeHandle);
    MeshSimplification::XForm<double> combineQ = mesh->property(m_Q, toEdgeVertexHandle) + mesh->property(m_Q, fromEdgeVertexHandle);
/*
    XForm<double> Q = XForm<double>(combineQ[0], combineQ[4], combineQ[8], 0,
        combineQ[4], combineQ[5], combineQ[9], 0,
        combineQ[8], combineQ[9], combineQ[10], 0,
        combineQ[12], combineQ[13], combineQ[14], 1);
        */
    Eigen::Matrix4d Q = (Eigen::Matrix4d() << combineQ[0], combineQ[4], combineQ[8], combineQ[12],
            combineQ[4], combineQ[5], combineQ[9], combineQ[13],
            combineQ[8], combineQ[9], combineQ[10], combineQ[14],
            0, 0, 0, 1).finished();
//  std::cerr << Q << std::endl;
    Eigen::FullPivLU<Eigen::Matrix4d> lu(Q);
    if (abs(lu.determinant()) > 1e-3) {
        Eigen::Matrix4d inverseQ = lu.inverse();
        /*
        double det = (inverseQ * Q).determinant();
        if (abs(det - 1) > 1e-6) {
            std::cerr << "check " << (inverseQ * Q).determinant() << inverseQ * Q << std::endl;
        }
        */
//      XForm<double> invQ = inv<double>(Q);
//      Vec<4, double> optimalV = Vec<4, double>(invQ[12], invQ[13], invQ[14], invQ[15]);
        MeshSimplification::Vec<4, double> optimalV = MeshSimplification::Vec<4, double>(inverseQ(0, 3), inverseQ(1, 3), inverseQ(2, 3), inverseQ(3, 3));
        double errorMetric = combineQ.vMult(optimalV);
        return { edgeHandle, errorMetric, optimalV };
    }
    else {
//      std::cerr << "Not invertible" << std::endl;
        OMT::Scalar *toEdgeVertexCoord = mesh->point(toEdgeVertexHandle).data();
        OMT::Scalar *fromEdgeVertexCoord = mesh->point(fromEdgeVertexHandle).data();

        MeshSimplification::Vec<4, double> optimalV = MeshSimplification::Vec<4, double>((toEdgeVertexCoord[0] + fromEdgeVertexCoord[0]) / 2, (toEdgeVertexCoord[1] + fromEdgeVertexCoord[1]) / 2, (toEdgeVertexCoord[2] + fromEdgeVertexCoord[2]) / 2, 1);
        double errorMetric = combineQ.vMult(optimalV);
        return { edgeHandle, errorMetric, optimalV };
    }
}