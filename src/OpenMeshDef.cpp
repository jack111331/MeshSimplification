//
// Created by Edge on 2021/11/6.
//

#include "OpenMeshDef.h"
#include <glad/glad.h>
#include <Utility.h>
#include <wx/log.h>

namespace OMT
{
    /*======================================================================*/
    Model::Model()
    {
        request_vertex_status();
        request_edge_status();
        request_face_status();
    }
    Model::~Model()
    {
        release_vertex_status();
        release_edge_status();
        release_face_status();
    }
}

void Tri_Mesh::createVAO() {

    //
    glGenVertexArrays(1, &m_vao);

    // Mesh VBO
    glGenBuffers(1, &m_coordVbo);

    // Tri face EBO
    glGenBuffers(1, &m_triFaceEbo);

    // edge EBO
    glGenBuffers(1, &m_edgeEbo);

    updateVAO();

    glBindVertexArray(0);
}
void Tri_Mesh::updateVAO() {
    glBindVertexArray(m_vao);

    std::vector<Vec3f> vertices;
    std::vector<uint32_t> triFaceIndices;
    std::vector<uint32_t> edgeIndices;
    for(OMT::VIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it) {
        OMT::Point coord = point(*v_it);
        vertices.push_back(Vec3f(coord[0], coord[1], coord[2]));
    }

    for(OMT::EIter e_it = edges_sbegin(); e_it != edges_end(); ++e_it)
    {
        OMT::HEHandle halfedgeHandle = halfedge_handle(*e_it,0);
        edgeIndices.push_back(from_vertex_handle(halfedgeHandle).idx());
        edgeIndices.push_back(to_vertex_handle(halfedgeHandle).idx());
    }

    for(OMT::FIter f_it = faces_sbegin(); f_it != faces_end(); ++f_it)
    {
        OMT::FVIter fv_it = fv_iter(*f_it);
        triFaceIndices.push_back(fv_it->idx());++fv_it;
        triFaceIndices.push_back(fv_it->idx());++fv_it;
        triFaceIndices.push_back(fv_it->idx());
    }

    m_edgeIndices = edgeIndices.size();
    m_faceIndices = triFaceIndices.size();
    wxLogMessage("Edge Indices=%d, Face Indices=%d", m_edgeIndices, m_faceIndices);

    // Mesh VBO
    glBindBuffer(GL_ARRAY_BUFFER, m_coordVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3f) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // Tri face EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triFaceEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * triFaceIndices.size(), triFaceIndices.data(), GL_STATIC_DRAW);

    // edge EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_edgeEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * edgeIndices.size(), edgeIndices.data(), GL_STATIC_DRAW);

}
