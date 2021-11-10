//
// Created by Edge on 2021/7/9.
//

#ifndef RAY_MAINFRAME_H
#define RAY_MAINFRAME_H

#include "GLPanel.h"
#include "OpenMeshDef.h"
#include "wx/thread.h"
#include "Skeleton.h"

class MeshManager;

class SimplificationThread : public wxThread
{
public:
    SimplificationThread(MeshManager *meshManager, GLPanel *view, wxSlider *slider);
    virtual ~SimplificationThread();

    // thread execution starts here
    virtual void *Entry() wxOVERRIDE;

public:
    MeshManager *m_meshManager;
    GLPanel *m_3DView;
    wxSlider *m_slider;

};



class MyFrame : public wxFrame {
public:
    MyFrame(const wxString &title, const wxPoint &pos, const wxSize &size);

    virtual ~MyFrame();

    GLPanel *m_3DView;
    wxTextCtrl *m_log;
    wxLog *m_logOld;
    wxSlider *m_slider;
    wxPanel *m_leftPanel;
    wxButton *m_render;
    wxTextCtrl *m_laplaceTimes;
    wxTextCtrl *m_wH;
    wxTextCtrl *m_wL;
    wxButton *m_skeleton;
    wxTextCtrl *m_wA;
    wxTextCtrl *m_wB;
    wxButton *m_skeletonToLine;

    Tri_Mesh *m_mesh = nullptr;
    MeshManager *m_meshManager = nullptr;
    SkeletonExtraction *m_skeletonExtraction = nullptr;

private:
    SimplificationThread *CreateThread();

    void OnHello(wxCommandEvent &event);

    void OnExit(wxCommandEvent &event);

    void OnAbout(wxCommandEvent &event);

    void OnRender(wxCommandEvent &event);

    void OnSimplification(wxScrollEvent &event);

    void OnSkeleton(wxCommandEvent &event);

    void OnSkeletonToLine(wxCommandEvent &event);

    void OpenFile(wxCommandEvent &event);

    void OpenMeshFile(wxCommandEvent &event);


wxDECLARE_EVENT_TABLE();
};

enum {
    // declares an id which will be used to call our button
    ID_Hello = wxID_HIGHEST + 1,
    MENU_Exit,
    MENU_About,
    MENU_Open,
    MENU_OpenMesh,
    SLIDER_Simplification,
    BUTTON_Render,
    BUTTON_Skeleton,
    BUTTON_SkeletonToLine,
    TEXT_Field
};


#endif //RAY_MAINFRAME_H
