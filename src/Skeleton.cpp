#include "Skeleton.h"
#include <cmath>
#include <glad/glad.h>
#include "Eigen/Dense"

void SkeletonExtraction::calculateSkeleton() {
    size_t vertexCount;
    if (m_k == 0) {
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

    Eigen::SparseMatrix<double> A(2 * vertexCount, vertexCount);
    Eigen::SparseMatrix<double> B(2 * vertexCount, 3);

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
            OMT::Normal edgeVector_1 = m_operateMesh->calc_edge_vector(
                    m_operateMesh->opposite_halfedge_handle(fh_it.handle()));
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
        } else {
            double currentOneRingArea = 0.0;
            for (OMT::VFIter vf_it = m_operateMesh->vf_begin(currentVertex);
                 vf_it != m_operateMesh->vf_end(currentVertex); ++vf_it) {
                OMT::FHandle faceHandle = vf_it.handle();
                OMT::FVIter fv_it = m_operateMesh->fv_begin(faceHandle);
                const OMT::Point &P = m_operateMesh->point(fv_it);
                ++fv_it;
                const OMT::Point &Q = m_operateMesh->point(fv_it);
                ++fv_it;
                const OMT::Point &R = m_operateMesh->point(fv_it);
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
            } else if (isnan(weight)) {
                std::cerr << weight << std::endl;
                weight = 0.0;
            }
            OMT::VHandle v = m_operateMesh->to_vertex_handle(
                    m_operateMesh->halfedge_handle(std::get<0>(cotPropList[j])));
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
        OMT::Scalar *pointCoord = m_operateMesh->point(v).data();
        coef.push_back(Eigen::Triplet<double>(vertexCount + i, 0, m_eachVertexWH[i] * pointCoord[0]));
        coef.push_back(Eigen::Triplet<double>(vertexCount + i, 1, m_eachVertexWH[i] * pointCoord[1]));
        coef.push_back(Eigen::Triplet<double>(vertexCount + i, 2, m_eachVertexWH[i] * pointCoord[2]));
    }
    B.setFromTriplets(coef.begin(), coef.end());
    // Factorize to upper-triangle LL^T = U^T U form
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(A.transpose() * A);
    Eigen::SparseMatrix<double> X = solver.solve(A.transpose() * B);


    double currentFaceArea = calculateCurrentFaceArea();
    std::cout << "Face area=" << currentFaceArea << std::endl;
    if (currentFaceArea / (m_initialAverageFaceArea * m_originalMesh->n_faces()) <= 1e-6) {
        wxLogMessage("Total face area lower than threshold: %d", m_k);
    }

    // new X [[v_1.x, v_2.y, v_3.z], ...]
    for (OMT::VIter v_it = m_operateMesh->vertices_begin(); v_it != m_operateMesh->vertices_end(); ++v_it) {
        int idx = v_it.handle().idx();
        OMT::Vec3d pointCoord = {X.coeff(idx, 0), X.coeff(idx, 1), X.coeff(idx, 2)};
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
        const OMT::Point &P = m_originalMesh->point(fv_it);
        ++fv_it;
        const OMT::Point &Q = m_originalMesh->point(fv_it);
        ++fv_it;
        const OMT::Point &R = m_originalMesh->point(fv_it);
        double area = ((Q - P) % (R - P)).norm() * 0.5;
        fv_it = m_originalMesh->fv_begin(faceHandle);
        m_initialOneRingArea[fv_it.handle().idx()] += area;
        ++fv_it;
        m_initialOneRingArea[fv_it.handle().idx()] += area;
        ++fv_it;
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
        const OMT::Point &P = m_operateMesh->point(fv_it);
        ++fv_it;
        const OMT::Point &Q = m_operateMesh->point(fv_it);
        ++fv_it;
        const OMT::Point &R = m_operateMesh->point(fv_it);
        double area = ((Q - P) % (R - P)).norm() * 0.5;
        averageArea += area;
    }
    return averageArea;
}

double SkeletonExtraction::calculateAngle(const OMT::Point &a, const OMT::Point &b) {
    return acos(a.normalized() | b.normalized());
}

double SkeletonExtraction::calcCot(const OMT::Point &a, const OMT::Point &b) {
    if (a == b) {
        return std::numeric_limits<double>::infinity();
    }
    return ((a | b) / (a % b).norm());
}

//bool SkeletonExtraction::isCollapsable(Tri_Mesh *mesh, OMT::HEHandle halfedgeHandle) {
//    int fromAdjacentV = 0;
//    int fromAdjacentF = 0;
//    OMT::VHandle fromVertexHandle = mesh->from_vertex_handle(halfedgeHandle);
//    for (auto vv_it = mesh->vv_begin(fromVertexHandle); vv_it != mesh->vv_end(fromVertexHandle); ++vv_it) {
//        fromAdjacentV++;
//    }
//    for (auto vf_it = mesh->vf_begin(fromVertexHandle); vf_it != mesh->vf_end(fromVertexHandle); ++vf_it) {
//        fromAdjacentF++;
//    }
//    int toAdjacentV = 0;
//    int toAdjacentF = 0;
//    OMT::VHandle toVertexHandle = mesh->to_vertex_handle(halfedgeHandle);
//    for (auto vv_it = mesh->vv_begin(toVertexHandle); vv_it != mesh->vv_end(toVertexHandle); ++vv_it) {
//        toAdjacentV++;
//    }
//    for (auto vf_it = mesh->vf_begin(toVertexHandle); vf_it != mesh->vf_end(toVertexHandle); ++vf_it) {
//        toAdjacentF++;
//    }
//    return fromAdjacentV > 1 && fromAdjacentF > 0 && toAdjacentV > 1 && toAdjacentF > 0;
//}

void SkeletonExtraction::propagateToTop(Tri_Mesh *mesh, int heapIdx) {
    while (heapIdx > 1) {
        if (*m_pQueue[heapIdx] < *m_pQueue[heapIdx / 2]) {
            SKErrorMetric *temp = m_pQueue[heapIdx];
            int tempIdx = m_pQueueHeapIdx[temp];

            m_pQueueHeapIdx[m_pQueue[heapIdx]] = m_pQueueHeapIdx[m_pQueue[heapIdx / 2]];
            m_pQueue[heapIdx] = m_pQueue[heapIdx / 2];

            m_pQueueHeapIdx[m_pQueue[heapIdx / 2]] = tempIdx;
            m_pQueue[heapIdx / 2] = temp;

            heapIdx /= 2;
        } else {
            break;
        }
    }
}

void SkeletonExtraction::propagateToBottom(Tri_Mesh *mesh, int heapIdx) {
    // FIXME need to locate heapIdx
    while (heapIdx < m_pQueue.size()) {
        int minIdx = -1;
        if (heapIdx * 2 + 1 <= m_pQueue.size()) {
            if (*m_pQueue[heapIdx * 2 + 1] < *m_pQueue[heapIdx * 2]) {
                minIdx = heapIdx * 2 + 1;
            } else {
                minIdx = heapIdx * 2;
            }
        } else if (heapIdx * 2 < m_pQueue.size()) {
            minIdx = heapIdx * 2;
        }
        if (minIdx != -1) {
            if (*m_pQueue[minIdx] < *m_pQueue[heapIdx]) {
                SKErrorMetric *temp = m_pQueue[heapIdx];
                int tempIdx = m_pQueueHeapIdx[temp];

                m_pQueueHeapIdx[m_pQueue[heapIdx]] = m_pQueueHeapIdx[m_pQueue[minIdx]];
                m_pQueue[heapIdx] = m_pQueue[minIdx];

                m_pQueueHeapIdx[m_pQueue[minIdx]] = tempIdx;
                m_pQueue[minIdx] = temp;

                heapIdx = minIdx;
            } else {
                break;
            }
        } else {
            break;
        }
    }
}

void SkeletonExtraction::insertEdge(Tri_Mesh *mesh, SKErrorMetric *skErrorMetric) {
    m_pQueue.push_back(skErrorMetric);
    m_pQueueHeapIdx[skErrorMetric] = m_pQueue.size()-1;
    propagateToTop(mesh, m_pQueue.size()-1);
}

SKErrorMetric *SkeletonExtraction::getTopEdge(Tri_Mesh *mesh) {
    return m_pQueue[1];
}

void SkeletonExtraction::deleteEdge(Tri_Mesh *mesh, SKErrorMetric *skErrorMetric) {
    int heapIdx = m_pQueueHeapIdx[skErrorMetric];
    m_pQueueHeapIdx.erase(m_pQueueHeapIdx.find(skErrorMetric));

    m_pQueueHeapIdx[m_pQueue[m_pQueueHeapIdx.size()-1]] = heapIdx;
    m_pQueue[heapIdx] = m_pQueue[m_pQueueHeapIdx.size()-1];

    m_pQueue.pop_back();

    propagateToTop(mesh, heapIdx);
    propagateToBottom(mesh, heapIdx);
}

void SkeletonExtraction::changeEdge(Tri_Mesh *mesh, SKErrorMetric *skErrorMetric) {
    // both should ensure the m_pQueueHeapIdx is always correct
    propagateToTop(mesh, m_pQueueHeapIdx[skErrorMetric]);
    propagateToBottom(mesh, m_pQueueHeapIdx[skErrorMetric]);
}

void
SkeletonExtraction::initErrorMetric(Tri_Mesh *mesh, std::map<OMT::VHandle, std::vector<SKErrorMetric>> &outHalfedgeMap,
                                    std::map<OMT::VHandle, std::vector<SkeletonFace>> &outFaceMap) {
    // Fix template type
    mesh->add_property(m_K);
    mesh->add_property(m_KTotal);
//    mesh->add_property(m_heapIdxProp);

    // add one dummy pointer
    m_pQueue.push_back(nullptr);
    int v_it_counter = 0;
    for (OMT::VIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it) {
        wxLogMessage("Initing v_it=%d", v_it_counter++);
        // initialize
        outHalfedgeMap[*v_it] = std::vector<SKErrorMetric>();
        std::vector<SKErrorMetric> &halfedgeList = outHalfedgeMap[*v_it];

        OMT::Scalar *pointCoord = mesh->point(*v_it).data();
        OMT::Point vPoint = OMT::Point(pointCoord[0], pointCoord[1], pointCoord[2]);
        Eigen::Vector3d vP(vPoint[0], vPoint[1], vPoint[2]);
        mesh->property(m_K, *v_it).setZero();
        for (OMT::VOHEIter voh_it = mesh->voh_begin(*v_it); voh_it != mesh->voh_end(*v_it); ++voh_it) {
            OMT::VHandle toVertexHandle = mesh->to_vertex_handle(*voh_it);
            OMT::Point p = mesh->point(toVertexHandle);
            Eigen::Vector3d a(p[0] - vPoint[0], p[1] - vPoint[1], p[2] - vPoint[2]);
            a.normalize();
            Eigen::Vector3d b = a.cross(vP);
            Eigen::MatrixXd K(3, 4);
            K << 0, -a[2], a[1], -b[0],
                    a[2], 0, -a[0], -b[1],
                    -a[1], a[0], 0, -b[2];
            mesh->property(m_K, *v_it) += (K.transpose() * K);
            halfedgeList.push_back(SKErrorMetric(*v_it, toVertexHandle, 0));
//            m_pQueue.push_back(&halfedgeList[halfedgeList.size()-1]);
        }
        computeVertexMetric(mesh, outHalfedgeMap, *v_it);
    }
    v_it_counter = 0;

    for (OMT::VIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it) {
        wxLogMessage("Initing v_it - 2 = %d", v_it_counter++);

        for (auto &outHalfedge: outHalfedgeMap[*v_it]) {
            computeErrorMetric(mesh, outHalfedge, outHalfedgeMap);
        }
        outFaceMap[*v_it] = std::vector<SkeletonFace>();
        std::vector<SkeletonFace> &faceList = outFaceMap[*v_it];
        SkeletonFace skeletonFace;
        skeletonFace.m_from = *v_it;
        for (OMT::VFIter vf_it = mesh->vf_begin(*v_it); vf_it != mesh->vf_end(*v_it); ++vf_it) {
            OMT::FVIter fv_it = mesh->fv_iter(*vf_it);
            while (*fv_it != *v_it) {
                ++fv_it;
            }
            ++fv_it;
            skeletonFace.m_to[0] = *fv_it;
            ++fv_it;
            skeletonFace.m_to[1] = *fv_it;
            faceList.push_back(skeletonFace);
            m_totalFace++;
        }
    }
}

bool SkeletonExtraction::collapseEdge(Tri_Mesh *mesh,
                                      std::map<OMT::VHandle, std::vector<SKErrorMetric>> &outHalfedgeMap,
                                      std::map<OMT::VHandle, std::vector<SkeletonFace>> &outFaceMap) {
    // because openmesh's collapse() can't change the face connectivity, so I had to manually keep these info
    wxLogMessage("Collapse Edge");
    bool isFoundMin = false;
    double minMetric = 1e10;
    OMT::VHandle collapseVertex;
    std::vector<SKErrorMetric>::iterator collapseHalfedgeIter;
    for (auto &element: outHalfedgeMap) {
        std::vector<SKErrorMetric> &outHalfedgeList = element.second;
        for (auto outHalfedgeIter = outHalfedgeList.begin();
             outHalfedgeIter != outHalfedgeList.end(); ++outHalfedgeIter) {
            double metric = outHalfedgeIter->m_metric;
            if (metric < minMetric) {
                // Don't collapse the edge that would break the connected component
                // this halfedge implicitly imply that from and to vertex have edge connectivity at least 1, so we just have to check the face existence
                if (!outFaceMap[outHalfedgeIter->m_from].empty() && !outFaceMap[outHalfedgeIter->m_to].empty()) {
                    isFoundMin = true;
                    minMetric = metric;

                    collapseVertex = element.first;
                    collapseHalfedgeIter = outHalfedgeIter;
                }
            }
        }
        if (isFoundMin) {
            break;
        }
    }
    if (!isFoundMin) {
        return false;
    }
    OMT::VHandle fromVertexHandle = collapseHalfedgeIter->m_from, toVertexHandle = collapseHalfedgeIter->m_to;

    halfedgeCollapse(outHalfedgeMap, outFaceMap, collapseHalfedgeIter);

    // Update K value according to paper
    mesh->property(m_K, toVertexHandle) = (mesh->property(m_K, toVertexHandle) + mesh->property(m_K, fromVertexHandle));

    // Because the "to vertex"'s K value change, according to original formula, the halfedge error metric around "to vertex" should be changed too.
    for (auto toHalfedgeIter = outHalfedgeMap[toVertexHandle].begin(); toHalfedgeIter != outHalfedgeMap[toVertexHandle].end(); ++toHalfedgeIter) {
        computeVertexMetric(mesh, outHalfedgeMap, toHalfedgeIter->m_to);
    }
    std::vector<SKErrorMetric> &toOutHalfedgeList = outHalfedgeMap[toVertexHandle];
    for (auto outHalfedgeIter = toOutHalfedgeList.begin();
         outHalfedgeIter != toOutHalfedgeList.end(); ++outHalfedgeIter) {
        computeErrorMetric(mesh, *outHalfedgeIter, outHalfedgeMap);
        for (auto toHalfedgeIter = outHalfedgeMap[outHalfedgeIter->m_to].begin();
             toHalfedgeIter != outHalfedgeMap[outHalfedgeIter->m_to].end(); ++toHalfedgeIter) {
            computeErrorMetric(mesh, *toHalfedgeIter, outHalfedgeMap);
        }
    }
    return m_totalFace > 0;
}

void SkeletonExtraction::halfedgeCollapse(std::map<OMT::VHandle, std::vector<SKErrorMetric>> &outHalfedgeMap,
                                          std::map<OMT::VHandle, std::vector<SkeletonFace>> &outFaceMap,
                                          std::vector<SKErrorMetric>::iterator collapseEdgeIter) {
    // erase element in outHalfedgeMap
    OMT::VHandle fromVertexHandle = collapseEdgeIter->m_from;
    OMT::VHandle toVertexHandle = collapseEdgeIter->m_to;

    // The process is first remove the halfedge from "fromVertexHandle", and then add back the halfedge
    // like below
    // other vertex - "fromVertexHandle"
    // ->
    // other vertex - "toVertexHandle"
    // halfedge deal phase
    for (auto fromHalfedgeIter = outHalfedgeMap[fromVertexHandle].begin();
         fromHalfedgeIter != outHalfedgeMap[fromVertexHandle].end(); ++fromHalfedgeIter) {

        // remove all halfedge which one side of it is fromVertexHandle
        for (auto toOfFromHalfedgeIter = outHalfedgeMap[fromHalfedgeIter->m_to].begin();
             toOfFromHalfedgeIter != outHalfedgeMap[fromHalfedgeIter->m_to].end(); ++toOfFromHalfedgeIter) {
            if (toOfFromHalfedgeIter->m_to == fromVertexHandle) {
                outHalfedgeMap[fromHalfedgeIter->m_to].erase(toOfFromHalfedgeIter);
                break;
            }
        }

        // add back the other vertex's edge connectivity to toVertexHandle
        if (fromHalfedgeIter->m_to != toVertexHandle) {
            bool isCommonVertex = false;
            // ignore the common vertex of toVertexHandle and fromVertexHandle
            for (auto toHalfedgeIter = outHalfedgeMap[toVertexHandle].begin(); toHalfedgeIter != outHalfedgeMap[toVertexHandle].end(); ++toHalfedgeIter) {
                if (toHalfedgeIter->m_to == fromHalfedgeIter->m_to) {
                    isCommonVertex = true;
                    break;
                }
            }
            if (!isCommonVertex) {
                // remember to update this SKErrorMetric
                outHalfedgeMap[toVertexHandle].push_back(SKErrorMetric(toVertexHandle, fromHalfedgeIter->m_to, 0));
                outHalfedgeMap[fromHalfedgeIter->m_to].push_back(SKErrorMetric(fromHalfedgeIter->m_to, toVertexHandle, 0));
            }
        }
    }
    // Remove fromVertexHandle's halfedge info
    outHalfedgeMap.erase(outHalfedgeMap.find(fromVertexHandle));

    // face deal phase
    for (auto fromFaceIter = outFaceMap[fromVertexHandle].begin();fromFaceIter != outFaceMap[fromVertexHandle].end();++fromFaceIter) {
        // Remove adjacent face vertex of fromVertexHandle, fromVertexHandle involved face
        for (int i = 0;i < 2;++i) {
            for(auto toOfFromFaceIter = outFaceMap[fromFaceIter->m_to[i]].begin(); toOfFromFaceIter != outFaceMap[fromFaceIter->m_to[i]].end();) {
                if (toOfFromFaceIter->m_to[0] == fromVertexHandle || toOfFromFaceIter->m_to[1] == fromVertexHandle) {
                    toOfFromFaceIter = outFaceMap[fromFaceIter->m_to[i]].erase(toOfFromFaceIter);
                    --m_totalFace;
                }
                else {
                    ++toOfFromFaceIter;
                }
            }
        }
        // Add back adjacent face vertex of fromVertexHandle, change fromVertexHandle involved face to toVertexHandle involved face
        SkeletonFace skeletonFace;
        // ensure that these face didn't involve the toVertexHandle, or it will be duplicate face
        if ((fromFaceIter->m_to[0] != toVertexHandle) && (fromFaceIter->m_to[1] != toVertexHandle)) {
            skeletonFace.m_from = toVertexHandle;
            skeletonFace.m_to[0] = fromFaceIter->m_to[0];
            skeletonFace.m_to[1] = fromFaceIter->m_to[1];
            outFaceMap[toVertexHandle].push_back(skeletonFace);
            m_totalFace += 1;

            skeletonFace.m_from = fromFaceIter->m_to[0];
            skeletonFace.m_to[0] = fromFaceIter->m_to[1];
            skeletonFace.m_to[1] = toVertexHandle;
            outFaceMap[fromFaceIter->m_to[0]].push_back(skeletonFace);
            m_totalFace += 1;

            skeletonFace.m_from = fromFaceIter->m_to[1];
            skeletonFace.m_to[0] = toVertexHandle;
            skeletonFace.m_to[1] = fromFaceIter->m_to[0];
            outFaceMap[fromFaceIter->m_to[1]].push_back(skeletonFace);
            m_totalFace += 1;
        }
    }
    m_totalFace -= outFaceMap[fromVertexHandle].size();
    outFaceMap.erase(outFaceMap.find(fromVertexHandle));
}

void SkeletonExtraction::computeVertexMetric(Tri_Mesh *mesh, std::map<OMT::VHandle, std::vector<SKErrorMetric>> &outHalfedgeMap, OMT::VHandle vHandle) {
    OMT::Point p = mesh->point(vHandle);
    double totalAdjacent = 0;

    for (auto outHalfedgeIter = outHalfedgeMap[vHandle].begin(); outHalfedgeIter != outHalfedgeMap[vHandle].end(); ++outHalfedgeIter) {
        OMT::Point p01 = (p - mesh->point(outHalfedgeIter->m_to));
        totalAdjacent += sqrt(p01 | p01);
    }

    mesh->property(m_KTotal, vHandle) = totalAdjacent;
}

void SkeletonExtraction::computeErrorMetric(Tri_Mesh *mesh, SKErrorMetric &em, std::map<OMT::VHandle, std::vector<SKErrorMetric>> &outHalfedgeMap) {
    // get one-ring
    OMT::Point toVertexPoint = mesh->point(em.m_to);
    Eigen::Vector4d toP(toVertexPoint[0], toVertexPoint[1], toVertexPoint[2], 1.0);
    OMT::Point fromVertexPoint = mesh->point(em.m_from);
    Eigen::Vector4d fromP(fromVertexPoint[0], fromVertexPoint[1], fromVertexPoint[2], 1.0);


    double Fa = (toP.transpose() * mesh->property(m_K, em.m_to) * toP);
    Fa += (toP.transpose() * mesh->property(m_K, em.m_from) * toP);

    double Fb = mesh->property(m_KTotal, em.m_from);
    Fb *= (fromP - toP).norm();

    double F = m_wa * Fa + m_wb * Fb;
    em.m_metric = F;
}

void SkeletonExtraction::simplifyMesh() {

    if (!m_isInitializedQ) {
        initErrorMetric(m_operateMesh, m_outHalfedgeMap, m_outFaceMap);
        m_isInitializedQ = true;
    }
    wxLogMessage("Init success");
    int i = 0;
    while (m_totalFace > 0) {
        wxLogMessage("Collapsed to %u", i++);
        if (!collapseEdge(m_operateMesh, m_outHalfedgeMap, m_outFaceMap)) {
            wxLogMessage("Collapse to end");
            break;
        }
//        easierCollapseEdge(m_operateMesh);
    }
    m_operateMesh->m_edgeIndices = 0;
    m_operateMesh->m_faceIndices = 0;

    std::vector<uint32_t> edgeSkeletonIndices;

    for(const auto &outHalfedgeList: m_outHalfedgeMap) {
        for(const auto &outHalfedge: outHalfedgeList.second) {
            edgeSkeletonIndices.push_back(outHalfedge.m_from.idx());
            edgeSkeletonIndices.push_back(outHalfedge.m_to.idx());
            m_operateMesh->m_edgeIndices += 2;
        }
    }
    wxLogMessage("Remain edge: %u", m_operateMesh->m_edgeIndices);
    updateSkeletonEBO(edgeSkeletonIndices);
}

void SkeletonExtraction::updateSkeletonEBO(std::vector<uint32_t> &edgeSkeletonIndices) {
    glBindVertexArray(m_operateMesh->m_vao);
//    // Tri face EBO
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_operateMesh->m_triFaceEbo);
//    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * triFaceIndices.size(), triFaceIndices.data(), GL_STATIC_DRAW);

    // edge EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_operateMesh->m_edgeEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * edgeSkeletonIndices.size(), edgeSkeletonIndices.data(), GL_STATIC_DRAW);

}